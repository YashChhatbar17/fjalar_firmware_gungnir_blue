#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "filter.h"
#include "aerodynamics.h"

LOG_MODULE_REGISTER(flight, CONFIG_APP_FLIGHT_LOG_LEVEL);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

#define PERIODIC_THREAD_PRIORITY 7
#define PERIODIC_THREAD_STACK_SIZE 4096

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_WINDOW_SIZE 11

#define BOOST_ACCEL_THRESHOLD 15.0
#define BOOST_SPEED_THRESHOLD 15.0

#define COAST_ACCEL_THRESHOLD 5.0
#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000

#define IMU_INIT_N 100
#define BARO_INIT_N 10
#define GPS_INIT_N 0

position_filter_t pos_kf;
attitude_filter_t att_kf;

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);
void periodic_thread(void *p1, void *p2, void *p3);

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE);
struct k_thread flight_thread_data;
k_tid_t flight_thread_id;

K_THREAD_STACK_DEFINE(periodic_thread_stack, PERIODIC_THREAD_STACK_SIZE);
struct k_thread periodic_thread_data;
k_tid_t periodic_thread_id;

ZBUS_LISTENER_DEFINE(pressure_zlis, NULL);
ZBUS_LISTENER_DEFINE(imu_zlis, NULL);
// ZBUS_CHAN_ADD_OBS(pressure_zchan, pressure_zobs, 1);
// ZBUS_CHAN_ADD_OBS(imu_zchan, imu_zobs, 1);

// ZBUS_SUBSCRIBER_DEFINE(imu_zchan);

void init_flight_state(fjalar_t *fjalar) {
    flight_thread_id = k_thread_create(
		&flight_thread_data,
		flight_thread_stack,
		K_THREAD_STACK_SIZEOF(flight_thread_stack),
		(k_thread_entry_t) flight_state_thread,
		fjalar, NULL, NULL,
		FLIGHT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(flight_thread_id, "flight state");

    periodic_thread_id = k_thread_create(
        &periodic_thread_data,
        periodic_thread_stack,
        K_THREAD_STACK_SIZEOF(periodic_thread_stack),
        periodic_thread,
        NULL, NULL, NULL,
        PERIODIC_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(periodic_thread_id, "periodic thread");

}

static init_t init = {0};

static inline bool init_ready(void){
    return init.n_imu  >= IMU_INIT_N  &&
           init.n_baro >= BARO_INIT_N &&
           init.n_gps  >= GPS_INIT_N;
}

/* converts sums -> means, writes them into fjalar  */
static void init_finish(fjalar_t *fjalar, position_filter_t *pos_kf){
    pos_kf->pressure_ground = init.p / BARO_INIT_N;
    LOG_INF("ground level pressure: %f", pos_kf->pressure_ground);

    pos_kf->lat0  = init.lat / GPS_INIT_N;
    pos_kf->lon0 = init.lon  / GPS_INIT_N;
    pos_kf->alt0 = init.alt / GPS_INIT_N;

    float ax = init.ax / IMU_INIT_N;
    float ay = init.ay / IMU_INIT_N;
    float az = init.az / IMU_INIT_N;

    pos_kf->g = sqrtf(ax*ax + ay*ay + az*az);
    LOG_INF("g: %f", pos_kf->g);

    pos_kf->seeded = false;

    init.position_init = true;
    LOG_INF("init done: lat0 = %f, lon0 = %f", pos_kf->lat0, pos_kf->lon0);
}


static void deploy_drogue(fjalar_t *fjalar) {
    fjalar->apogee_at = k_uptime_get_32();
    LOG_WRN("drogue deployed at %fm %fs", fjalar->altitude - fjalar->ground_level, (fjalar->apogee_at - fjalar->liftoff_at) / 1000.0f);
}

static void deploy_main(fjalar_t *fjalar) {
    LOG_WRN("Main deployed at %f", fjalar->altitude - fjalar->ground_level);
}

static void evaluate_state(fjalar_t *fjalar) {
    switch (fjalar->flight_state) {
    case STATE_IDLE:
        //we chillin'
        break;
    case STATE_LAUNCHPAD:
        if (fjalar->az > BOOST_ACCEL_THRESHOLD) {
            fjalar->flight_state = STATE_BOOST;
            fjalar->liftoff_at = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to acceleration");
        }
        if (fjalar->velocity > BOOST_SPEED_THRESHOLD) {
            fjalar->flight_state = STATE_BOOST;
            fjalar->liftoff_at = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to speed");
        }
        break;
    case STATE_BOOST:
        if (fjalar->az < COAST_ACCEL_THRESHOLD) {
            fjalar->flight_state = STATE_COAST;
            LOG_WRN("Changing state to COAST due to acceleration");
        }
        if (fjalar->velocity < 0) {
            LOG_WRN("Fake pressure increase due to sonic shock wave");
        }
        break;
    case STATE_COAST:
        if (fjalar->velocity < 0) {
            deploy_drogue(fjalar);
            fjalar->flight_state = STATE_FREE_FALL;
            LOG_WRN("Changing state to FREE_FALL due to speed");
        }
        break;
    case STATE_FREE_FALL:
        if (k_uptime_get_32() - fjalar->apogee_at > DROGUE_DEPLOYMENT_FAILURE_DELAY) {
            // vec3 acc = {fjalar->ax, fjalar->ay, fjalar->az};
        }
        break;
    case STATE_DROGUE_DESCENT:
        deploy_main(fjalar);
        break;
    case STATE_MAIN_DESCENT:
        break;
    case STATE_LANDED:
        break;
    }
}

// thread for numerical analysis
void periodic_thread(void *p1, void *p2, void *p3) {
    while (true) {
        Pmtx_analysis(&pos_kf);

        update_apogee_estimate(&pos_kf);


        k_msleep(50); // 50 ms = 20 Hz
    }
}

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    struct k_poll_event events[2] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &pressure_msgq),
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &imu_msgq),
    };

    fjalar->ground_level = 0;
    fjalar->ax = 0;
    fjalar->ay = 0;
    fjalar->az = 9.8;
    fjalar->altitude = 0;
    fjalar->velocity = 0;

    position_filter_init(&pos_kf);
    attitude_filter_init(&att_kf);

    struct pressure_queue_entry pressure;
    struct imu_queue_entry imu;
    struct gps_queue_entry gps;


    // k_poll(&events[0], 1, K_FOREVER);
    // k_poll(&events[1], 1, K_FOREVER);
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;

    #ifdef CONFIG_BOOT_STATE_LAUNCHPAD
    fjalar->flight_state = STATE_LAUNCHPAD;
    #endif
    #ifdef CONFIG_BOOT_STATE_IDLE
    fjalar->flight_state = STATE_IDLE;
    #endif

    while (true) {
        if (k_poll(events, 2, K_MSEC(1000))) {
            LOG_ERR("Stopped receiving measurements");
            continue;
        }

        if (k_msgq_get(&imu_msgq, &imu, K_NO_WAIT) == 0) {
            events[1].state = K_POLL_STATE_NOT_READY;

            // init mode
            if (!init.position_init) {
                if (init.n_imu < IMU_INIT_N) {
                    init.ax += imu.ax;
                    init.ay += imu.ay;
                    init.az += imu.az;
                    init.n_imu++;
                }
            } else {
                position_filter_accelerometer(&pos_kf, &att_kf, imu.ax, imu.ay, imu.az, imu.t); // needs magnetometer
                //attitude_filter_gyroscope(&att_kf, imu.gx, imu.gy, imu.gz, imu.t); // needs correction step with estimated accel
                attitude_filter_accelerometer(&att_kf, imu.ax, imu.ay, imu.az, imu.t);
            }

            /* Give an error if the acceleration in z at launchpad is not around 9.8
            if (fjalar->flight_state == STATE_LAUNCHPAD
            || fjalar->flight_state == STATE_IDLE
            ) {
            if (fjalar->az < 0
            && (fjalar->flight_state == STATE_LAUNCHPAD)) {
                LOG_ERR("Acceleration is negative on the launchpad");
            }
            */

            // Please add EKF to make these be accurate
            fjalar->ax = imu.ax;
            fjalar->ay = imu.ay;
            fjalar->az = imu.az;

            //LOG_DBG("Acceleration: %f %f %f", fjalar->ax, fjalar->ay, fjalar->az);
        }

        if (k_msgq_get(&pressure_msgq, &pressure, K_NO_WAIT) == 0) {
            events[0].state = K_POLL_STATE_NOT_READY;

            // init
            if (!init.position_init) {
                if (init.n_baro < BARO_INIT_N) {
                    init.p += pressure.pressure*1000;
                    init.n_baro++;
                }
            } else {
                position_filter_barometer(&pos_kf, pressure.pressure, pressure.t);
            }

            fjalar->altitude = filter_get_altitude(&pos_kf);
            fjalar->velocity = filter_get_velocity(&pos_kf);

            if (fjalar->flight_state == STATE_LAUNCHPAD) {
                fjalar->ground_level = fjalar->altitude;
            }
        }


        #if DT_ALIAS_EXISTS(gps_uart)
        {
            struct gps_queue_entry gps;
            if (k_msgq_get(&gps_msgq, &gps, K_NO_WAIT) == 0) {

                // Reject frames that contain NaN or Inf in ANY field
                if (isfinite(gps.lat) &&
                    isfinite(gps.lon) &&
                    isfinite(gps.alt)) {
                    // init position
                    if (!init.position_init) {
                        if (init.n_gps < GPS_INIT_N) {
                            init.lat += gps.lat;
                            init.lon += gps.lon;
                            init.alt += gps.alt;
                            init.n_gps++;
                        }
                    } else {
                        position_filter_gps(&pos_kf, gps.lat, gps.lon, gps.alt, gps.t);
                    }

                } else {
                    LOG_WRN("GPS sample dropped (NaN/Inf)");
                }
            }
        }
        #endif

        if (!init.position_init && init_ready()) {
            init_finish(fjalar, &pos_kf);
        }

        evaluate_state(fjalar);
        //LOG_DBG("state %d", fjalar->flight_state);
        // k_msleep(1);
    }
}

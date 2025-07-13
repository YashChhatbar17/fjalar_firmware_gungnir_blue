/*
The flight state script serves the following purpose:
1) Given some information about the rocket, determine its flight state
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"

LOG_MODULE_REGISTER(flight, CONFIG_APP_FLIGHT_LOG_LEVEL);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_WINDOW_SIZE 11

#define BOOST_ACCEL_THRESHOLD 15.0
#define BOOST_SPEED_THRESHOLD 15.0

#define COAST_ACCEL_THRESHOLD 5.0
#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000






void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE);
struct k_thread flight_thread_data;
k_tid_t flight_thread_id;

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
}

// fix the information logic
static void deploy_drogue(init_t *init, state_t *state, position_filter_t *pos_kf) { 
    state->apogee_time = k_uptime_get_32();
    LOG_WRN("drogue deployed at %.2f m %.2f s", pos_kf->X_data[2] - init->alt0, (state->apogee_time - state->liftoff_time) / 1000.0f);
}


// fix the information logic
static void deploy_main(init_t *init, state_t *state, position_filter_t *pos_kf) {
    LOG_WRN("Main deployed at %.2f m", pos_kf->X_data[2] - init->alt0);
}

// update state machine
static void evaluate_state(init_t *init, state_t *state, position_filter_t *pos_kf) {
    float az = pos_kf->X_data[8];
    float vz = pos_kf->X_data[5];
    float z  = pos_kf->X_data[2];

    switch (state->flight_state) {
    case STATE_IDLE:
        //we chillin'
        break;
    case STATE_LAUNCHPAD:
        if (az > BOOST_ACCEL_THRESHOLD) {
            state->flight_state = STATE_BOOST;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to acceleration");
        }
        if (vz > BOOST_SPEED_THRESHOLD) {
            state->flight_state = STATE_BOOST;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("Changing state to BOOST due to speed");
        }
        break;
    case STATE_BOOST:
        if (az < COAST_ACCEL_THRESHOLD) {
            state->flight_state = STATE_COAST;
            LOG_WRN("Changing state to COAST due to acceleration");
        }
        
        if (vz < 0) {
            LOG_WRN("Fake pressure increase due to sonic shock wave"); // look this over, create condition inside of state machine 
        }
        break;
    case STATE_COAST:
        if (vz < 0) {
            deploy_drogue(init, state, pos_kf);
            state->flight_state = STATE_FREE_FALL;
            LOG_WRN("Changing state to FREE_FALL due to speed");
        }
        break;
    case STATE_FREE_FALL:
        if (k_uptime_get_32() - state->apogee_time > DROGUE_DEPLOYMENT_FAILURE_DELAY) {
            // vec3 acc = {state->ax, state->ay, state->az};
        }
        break;
    case STATE_DROGUE_DESCENT:
        deploy_main(init, state, pos_kf);
        break;
    case STATE_MAIN_DESCENT:
        break;
    case STATE_LANDED:
        break;
    }
}


void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;
    position_filter_t *pos_kf = fjalar->ptr_pos_kf;
    attitude_filter_t *att_kf = fjalar->ptr_att_kf;
    aerodynamics_t    *aerodynamics = fjalar->ptr_aerodynamics;
    state_t           *state = fjalar->ptr_state;



    while (true) {
        evaluate_state(init, state, pos_kf);
        //LOG_DBG("state %d", fjalar->flight_state);
        k_msleep(10);
    }
}

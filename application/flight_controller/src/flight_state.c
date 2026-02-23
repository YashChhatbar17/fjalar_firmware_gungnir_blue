/*
The flight_state module is responsible for determining the rocket's current
flight phase based on filtered sensor data, aerodynamic estimates, and external
commands (LoRa). It runs as a dedicated thread at ~100 Hz.

Its tasks:

1) Evaluate sensor-derived quantities (acceleration, velocity, altitude).
2) Detect critical events (launch, burnout, apogee, chute deployments, landing).
3) Manage the high-level flight state machine (IDLE → BOOST → COAST → ...).
4) Trigger actuation commands (pyro firing) when needed.
5) Provide derived event flags for logging, telemetry, and backup logic.
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
#include "actuation.h"
#include "com_lora.h"

LOG_MODULE_REGISTER(flight, LOG_LEVEL_INF); //Logging

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 4096

// (These windows are not used directly in this file but may be used by filters elsewhere.)
#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40
#define IMU_WINDOW_SIZE 11

// Delay to declare failure if drogue does not deploy (not used here currently).
#define DROGUE_DEPLOYMENT_FAILURE_DELAY 8000


/* -------------------------------------------------------------------------- */
/*                          THREAD INITIALIZATION                              */
/* -------------------------------------------------------------------------- */

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1); //Forward declaration

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE); //RAM allocation
struct k_thread flight_thread_data; //Allocates the memory at a fixed address;
k_tid_t flight_thread_id; //Variable of type k_tid_t that stores a thread identifier

// Registered listeners for pressure and IMU data (handled elsewhere)
ZBUS_LISTENER_DEFINE(pressure_zlis, NULL);
ZBUS_LISTENER_DEFINE(imu_zlis, NULL);

/*
Starts the flight state management thread and passes the fjalar "god object"
into it. The fjalar struct gives the thread access to all subsystems:
filters, sensors, actuation, communication, etc.
*/
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


/* -------------------------------------------------------------------------- */
/*                          PYRO DEPLOYMENT HELPERS                            */
/* -------------------------------------------------------------------------- */

/*
Deploy the DROGUE parachute at apogee.
This fires pyro channel 1. The logic for triggering the actual hardware
is handled inside actuation.c via set_pyro().
*/
static void deploy_reefed(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) { 
    set_pyro(fjalar, 1, true);  // 1 = drogue chute pyro channel
    LOG_WRN("Drogue deployed at %.2f m %.2f s",
            pos_kf->X_data[2],                                  // altitude
            (state->apogee_time - state->liftoff_time) / 1000.0f);
}

/*
Deploy the MAIN parachute during descent.
This fires pyro channel 2.
*/
static void deploy_main(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) {
    set_pyro(fjalar, 2, true);  // 2 = main chute pyro channel
    LOG_WRN("Main deployed at %.2f m", pos_kf->X_data[2]);
}


/* -------------------------------------------------------------------------- */
/*                      MAIN FLIGHT STATE EVALUATION                           */
/* -------------------------------------------------------------------------- */

/*
This function implements the rocket's flight state machine.
It uses:
 - Kalman-filtered altitude (z)
 - velocity (vz)
 - acceleration (az, a_norm)
 - aerodynamic estimates (Mach number, thrust flag)
 - commands from LoRa
 - initialization state

The state machine flow is:

IDLE → AWAITING_INIT → INITIATED → AWAITING_LAUNCH 
→ BOOST → COAST → DROGUE_DESCENT → MAIN_DESCENT → LANDED
*/
static void evaluate_state(
    fjalar_t *fjalar,
    init_t *init,
    state_t *state,
    position_filter_t *pos_kf,
    aerodynamics_t *aerodynamics,
    lora_t *lora)
{
    // Extract common flight quantities from the state estimator.
    float az = pos_kf->X_data[8];     // Vertical acceleration (estimated)
    float vz = pos_kf->X_data[5];     // Vertical velocity (estimated)
    float z  = pos_kf->X_data[2];     // Altitude (estimated)

    float a_norm = pos_kf->a_norm;    // Magnitude of acceleration vector
    float v_norm = pos_kf->v_norm;    // Magnitude of velocity vector

    switch (state->flight_state) {

    /* ---------------------------- STATE_IDLE ---------------------------- */
    case STATE_IDLE:
        /*
        In this state the rocket is powered on and waiting for a LoRa command
        telling it to begin initialization procedures (sensor calibration, etc.).
        */
        if (lora->LORA_READY_INITIATE_FJALAR){
            state->flight_state = STATE_AWAITING_INIT;

            // Begin initialization routines. These prepare sensors and filters.
            init_init(&fjalar_god);
            init_sensors(&fjalar_god);

            LOG_WRN("State: IDLE → AWAITING_INIT (LoRa INIT command)");
        }
        break;

    /* ------------------------- STATE_AWAITING_INIT ----------------------- */
    case STATE_AWAITING_INIT:
        /*
        Wait until all initialization routines (IMU calibration, barometer
        stabilization, etc.) signal completion.
        */
        if (init->init_completed){
            state->flight_state = STATE_INITIATED;
            LOG_WRN("State: AWAITING_INIT → INITIATED (init complete)");
        }
        break;

    /* --------------------------- STATE_INITIATED ------------------------- */
    case STATE_INITIATED:
        /*
        Rocket is fully initialized and waiting for a "ready for launch" command.
        This is a deliberate second confirmation to avoid accidental liftoff.
        */
        if (lora->LORA_READY_LAUNCH_FJALAR){
            state->flight_state = STATE_AWAITING_LAUNCH;
            LOG_WRN("State: INITIATED → AWAITING_LAUNCH (LoRa LAUNCH command)");
        }
        break;

    /* ------------------------ STATE_AWAITING_LAUNCH ---------------------- */
    case STATE_AWAITING_LAUNCH:
        /*
        Detect launch using acceleration magnitude or estimated speed.
        Filtering logic may zero velocity early in flight, so both checks exist.
        */

        // Acceleration + altitude gate to avoid false positives on ground bumps
        if (a_norm > BOOST_ACCEL_THRESHOLD && z > 3){
            state->flight_state = STATE_BOOST;
            state->event_launch = true;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("State: AWAITING_LAUNCH → BOOST (acceleration trigger)");
        }

        // Speed trigger fallback (in case accel filtering suppresses early thrust)
        if (v_norm > BOOST_SPEED_THRESHOLD){ 
            state->flight_state = STATE_BOOST;
            state->event_launch = true;
            state->liftoff_time = k_uptime_get_32();
            LOG_WRN("State: AWAITING_LAUNCH → BOOST (speed trigger)");
        }
        break;

    /* ------------------------------ STATE_BOOST -------------------------- */
    case STATE_BOOST:
        /*
        Detect burnout when vertical acceleration drops and the aerodynamics
        module reports thrust has ended.
        */
        if (az < COAST_ACCEL_THRESHOLD && !aerodynamics->thrust_bool) {
            state->flight_state = STATE_COAST;
            state->event_burnout = true;
            LOG_WRN("State: BOOST → COAST (burnout detected)");
        }
        break;

    /* ------------------------------ STATE_COAST -------------------------- */
    case STATE_COAST:
        /*
        Detect apogee: upward velocity transitions to negative.
        This is the most reliable trigger for drogue deployment.
        */
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            deploy_main(fjalar, init, state, pos_kf); //Should probably change to deploy_main
            state->flight_state = STATE_MAIN_DESCENT;
            state->event_apogee = true;

            LOG_WRN("State: COAST → MAIN_DESCENT (apogee detected)");
        }
        break;

    /* --------------------------- STATE_DROGUE_DESCENT -------------------- */
    case STATE_MAIN_DESCENT:
        /*
        Deploy main parachute at a safe low altitude threshold.
        */
        if (z < 200) {
            deploy_reefed(fjalar, init, state, pos_kf);
            state->flight_state = STATE_REEFED_DESCENT;
            LOG_WRN("State: MAIN_DESCENT → REEFED_DESCENT (200 m AGL)");
        }
        break;

    /* --------------------------- STATE_MAIN_DESCENT ---------------------- */
    case STATE_REEFED_DESCENT:
        /*
        Detect landing by checking if acceleration ~ 1 g and movement is minimal.
        This is approximate but usually reliable with filtered acceleration.
        */
        if (a_norm > init->g_accelerometer - 2 &&
            a_norm < init->g_accelerometer + 2)
        {
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State: MAIN_DESCENT → LANDED (acceleration stable ~1g)");
        }
        break;

    /* ------------------------------ STATE_LANDED ------------------------- */
    case STATE_LANDED:
        // Nothing further — flight is complete.
        LOG_INF("flight state: STATE_LANDED");
        break;
    }
}


/* -------------------------------------------------------------------------- */
/*                        EVENT FLAG EVALUATION                               */
/* -------------------------------------------------------------------------- */

/*
This function updates boolean event flags derived from the filtered state.
These are useful for telemetry, safety systems, or post-flight analysis.
*/
static void evaluate_event(fjalar_t *fjalar, state_t *state, position_filter_t *pos_kf) {
    float z = pos_kf->X_data[2];

    //state->event_above_acs_threshold = (z > 1500);  // Example threshold for ACS activation. Probably not needed. 
    state->event_drogue_deployed     = fjalar->pyro1_sense; //Probably not needed
    state->event_main_deployed       = fjalar->pyro2_sense;
}


/* -------------------------------------------------------------------------- */
/*                VELOCITY CLASSIFICATION (MACH STATE)                        */
/* -------------------------------------------------------------------------- */

/*
Classifies the rocket's velocity regime based on Mach number.
Used for aerodynamic coefficient selection or logging.

Hysteresis is used to prevent rapid oscillation near regime boundaries.
*/
static void evaluate_velocity(aerodynamics_t *aerodynamics, state_t *state) {
    float M = aerodynamics->mach_number;

    switch(state->velocity_class){

    case VELOCITY_SUBSONIC:
        if (M > 0.8){
            state->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class → transonic");
        }
        break;

    case VELOCITY_TRANSONIC:
        if (M > 1.2){
            state->velocity_class = VELOCITY_SUPERSONIC;
            LOG_WRN("velocity class → supersonic");
        }
        if (M < 0.8){
            state->velocity_class = VELOCITY_SUBSONIC;
            LOG_WRN("velocity class → subsonic");
        }
        break;

    case VELOCITY_SUPERSONIC:
        if (M < 1.2){
            state->velocity_class = VELOCITY_TRANSONIC;
            LOG_WRN("velocity class → transonic");
        }
        break;
    }
}


/* -------------------------------------------------------------------------- */
/*                            MAIN THREAD LOOP                                */
/* -------------------------------------------------------------------------- */

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    // Unpack subsystem pointers from the god struct
    init_t            *init         = fjalar->ptr_init;
    position_filter_t *pos_kf       = fjalar->ptr_pos_kf;
    attitude_filter_t *att_kf       = fjalar->ptr_att_kf;
    aerodynamics_t    *aerodynamics = fjalar->ptr_aerodynamics;
    state_t           *state        = fjalar->ptr_state;
    lora_t            *lora         = fjalar->ptr_lora;

    // Initial states for a freshly powered rocket
    state->flight_state  = STATE_IDLE;
    state->velocity_class = VELOCITY_SUBSONIC;

    while (true) {

        // Update the main state machine
        evaluate_state(fjalar, init, state, pos_kf, aerodynamics, lora);

        // Update derived event flags
        evaluate_event(fjalar, state, pos_kf);

        // Update Mach regime classification
        evaluate_velocity(aerodynamics, state);

        k_msleep(10);  // ~100 Hz loop rate
    }
}

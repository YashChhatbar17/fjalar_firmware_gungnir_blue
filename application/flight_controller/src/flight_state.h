#pragma once

#include "fjalar.h"
#include "filter.h"
#include "aerodynamics.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;

enum fjalar_flight_state { // code these in to state machine
    STATE_IDLE,
    STATE_AWAITING_INIT,
    STATE_INITIATED,
    STATE_AWAITING_LAUNCH,
    STATE_BOOST,
    STATE_COAST,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
};

enum fjalar_velocity_class {
    VELOCITY_SUBSONIC,
    VELOCITY_TRANSONIC,
    VELOCITY_SUPERSONIC
};

#define BOOST_ACCEL_THRESHOLD 15.0
#define BOOST_SPEED_THRESHOLD 15.0
#define COAST_ACCEL_THRESHOLD 5.0

// Flight state output message for message queue
typedef struct flight_state_output_msg {
    uint32_t timestamp;
    enum fjalar_flight_state flight_state;
    enum fjalar_velocity_class velocity_class;

    uint32_t liftoff_time;
    uint32_t apogee_time;

    bool event_launch;
    bool event_burnout;
    bool event_above_acs_threshold;
    bool event_apogee;
    bool event_drogue_deployed;
    bool event_main_deployed;
    bool event_landed;
} flight_state_output_msg;

// Declare the message queue (defined in flight_state.c)
extern struct k_msgq flight_state_output_msgq;

void init_flight_state(fjalar_t *fjalar);



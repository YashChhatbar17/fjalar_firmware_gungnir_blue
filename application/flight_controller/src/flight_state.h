#pragma once

#include "fjalar.h"
#include "filter.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;

enum fjalar_flight_state { // code these in to state machine
    STATE_IDLE,
    STATE_AWAITING_INIT,
    STATE_INITIATED,
    STATE_AWAITING_LAUNCH,
    STATE_BOOST,
    STATE_COAST,
    STATE_MAIN_DESCENT, //Switched from drogue to main
    STATE_REEFED_DESCENT, //Changed from main to reefed
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



typedef struct state {
    enum fjalar_flight_state flight_state;
    enum fjalar_velocity_class velocity_class;
    
    uint32_t liftoff_time;
    uint32_t apogee_time;

    bool event_launch;
    bool event_burnout;
    //bool event_above_acs_threshold;
    bool event_apogee;
    bool event_drogue_deployed;
    bool event_main_deployed;
    bool event_landed;
} state_t;

void init_flight_state(fjalar_t *fjalar);



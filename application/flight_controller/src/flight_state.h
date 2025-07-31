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
    STATE_LAUNCHPAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
};

enum fjalar_flight_event { // potentially make event machine, but why?
    EVENT_LAUNCH,
    EVENT_BURNOUT,
    EVENT_ABOVE_ACS_THRESHOLD,
    EVENT_APOGEE,
    EVENT_PRIMARY_DEPLOY,
    EVENT_SECONDARY_DEPLOY,
    EVENT_LANDED
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
    enum fjalar_flight_event flight_event;

    enum fjalar_velocity_class velocity_class;
    
    uint32_t liftoff_time;
    uint32_t apogee_time;

    bool drogue_deployed; // use in state machine maybe remove and only keep in fjalar, or edit communications.c (noooooo must I learn new code)
    bool main_deployed; // use in state machine

} state_t;

void init_flight_state(fjalar_t *fjalar);



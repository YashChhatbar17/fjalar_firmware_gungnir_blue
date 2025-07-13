#pragma once

#include "fjalar.h"
#include "filter.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;

typedef struct state {
    int flight_state; // to implement for state machine
    
    uint32_t liftoff_time; // need to add an update to this inside of state machine
    uint32_t apogee_time; // need to add an update to this inside of state machine

    bool drogue_deployed; // use in state machine maybe remove and only keep in fjalar, or edit communications.c (noooooo must I learn new code)
    bool main_deployed; // use in state machine
} state_t;

void init_flight_state(fjalar_t *fjalar);



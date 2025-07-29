#pragma once

#include "fjalar.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;

typedef struct control {
    float control_output; // rename, idk what you guys need
} control_t;

void init_control(fjalar_t *fjalar);

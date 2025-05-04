#pragma once

#include "fjalar.h"
void init_flight_state(fjalar_t *fjalar);

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);
void periodic_thread(void *p1, void *p2, void *p3);

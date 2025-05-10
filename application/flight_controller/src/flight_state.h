#pragma once

#include "fjalar.h"
#include "filter.h"

void init_flight_state(fjalar_t *fjalar);
void init_finish(fjalar_t *fjalar, position_filter_t *pos_kf, attitude_filter_t *att_kf, init_t *init);

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);
void periodic_thread(void *p1, void *p2, void *p3);


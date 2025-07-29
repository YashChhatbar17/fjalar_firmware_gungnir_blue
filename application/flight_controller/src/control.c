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
#include "control.h"

#define CONTROL_THREAD_PRIORITY 7
#define CONTROL_THREAD_STACK_SIZE 4096

LOG_MODULE_REGISTER(control, CONFIG_APP_FLIGHT_LOG_LEVEL);

void control_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(control_thread_stack, CONTROL_THREAD_STACK_SIZE);
struct k_thread control_thread_data;
k_tid_t control_thread_id;

void init_control(fjalar_t *fjalar) {
    control_thread_id = k_thread_create(
		&control_thread_data,
		control_thread_stack,
		K_THREAD_STACK_SIZEOF(control_thread_stack),
		(k_thread_entry_t) control_thread,
		fjalar, NULL, NULL,
		CONTROL_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(control_thread_id, "flight state");
}



void control_thread(fjalar_t *fjalar, void *p2, void *p1) {
    init_t            *init  = fjalar->ptr_init;
    position_filter_t *pos_kf = fjalar->ptr_pos_kf;
    attitude_filter_t *att_kf = fjalar->ptr_att_kf;
    aerodynamics_t    *aerodynamics = fjalar->ptr_aerodynamics;
    state_t           *state = fjalar->ptr_state;
    control_t         *control = fjalar->ptr_control;

    control->control_output = 10.0; // just to test if struct is working

    while (true){
        LOG_INF("conttrol loop running"); // test logging
        LOG_INF("output: %f", control->control_output); // test struct

        k_msleep(10); // 100 Hz
    }
}
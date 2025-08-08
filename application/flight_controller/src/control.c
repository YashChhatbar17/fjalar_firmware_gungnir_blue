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

    static float integral = 0.0f;
    static float last_error = 0.0f;
    while (true){

        float altitude_AGL = pos_kf->X_data[2];

        if (state->flight_state == STATE_COAST && altitude_AGL > 1500.0f) {
            float predicted_apogee = aerodynamics->expected_apogee;
            float error = predicted_apogee - TARGET_APOGEE_AGL;

            integral += error * SAMPLING_TIME_S;

            if (integral > PID_INTEGRAL_MAX) {
                    integral = PID_INTEGRAL_MAX;
            } else if (integral < -PID_INTEGRAL_MAX) {
                    integral = -PID_INTEGRAL_MAX;
                }
            float derivative = (error - last_error)/SAMPLING_TIME_S;
            float output = (PID_P_GAIN * error) + (PID_I_GAIN * integral) + (PID_D_GAIN * derivative);
            last_error = error;

             // Clamp the final output to range 0 to 1
            if (output > PID_OUTPUT_MAX) {
                    output = PID_OUTPUT_MAX;
            } else if (output < PID_OUTPUT_MIN) {
                    output = PID_OUTPUT_MIN;
                }

            // Block of code converting linear movement of airbrakes on the rails to degrees for the motor    
            float desired_deployment_mm = output * x_max;
            float A = desired_deployment_mm + x_ret;
            float num = (A * A) - (link_L * link_L - crank_R * crank_R);
            float den = 2 * crank_R * A;
            float arg = num / den;

            if (arg > 1.0f) arg = 1.0f;
            if (arg < -1.0f) arg = 1.0f;

            float theta = asinf(arg) * (180.0f / 3.14159f);
            control->control_output = theta;

            //LOG_INF("output: %f", control->control_output);
            //LOG_INF("CONTROL ACTIVE: Alt=%.1f, Pred_Ap=%.1f, Err=%.1f, PID_Out=%.2f, Angle=%.2f", altitude_AGL, predicted_apogee, error, output, control->control_output);

        } else {
            //to prevent integral windup
            integral = 0.0f;
            last_error = 0.0f;
            control->control_output = 0.0f;
            }

        k_msleep(10); // 100 Hz
    }
}
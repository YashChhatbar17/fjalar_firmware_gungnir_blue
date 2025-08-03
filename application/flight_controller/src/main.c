/*
This is the main script, its purpose is to:
1) Declare the structures which will be used to store information throughout the script.
2) Initiate the different scripts by starting their threads and distributing their structure pointers through the god struct (fjalar).
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <math.h>

#include "fjalar.h"
#include "sensors.h"
#include "flight_state.h"
#include "com_master.h"
#include "actuation.h"
#include "hello.h"
#include "init.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "can_com.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_MAIN_LOG_LEVEL);

static init_t            init_obj;
static position_filter_t position_filter_obj;
static attitude_filter_t attitude_filter_obj;
static aerodynamics_t    aerodynamics_obj;
static state_t           state_obj;  
static can_t			 can_obj;

fjalar_t fjalar_god = {
	.ptr_init         = &init_obj,
	.ptr_pos_kf       = &position_filter_obj,
	.ptr_att_kf       = &attitude_filter_obj,
	.ptr_aerodynamics = &aerodynamics_obj,
	.ptr_state        = &state_obj,
	.ptr_can	      = &can_obj
};

int main(void) {
	#ifdef CONFIG_DELAYED_START
	const int delay = 5;
	for (int i = 0; i < delay; i++) {
		printk("%d\n", delay - i);
		k_msleep(1000);
	}
	#endif
	printk("Started\n");

	fjalar_god.sudo = false;
	uint8_t cpp_buf[64];
	hello_from_cpp(cpp_buf, sizeof(cpp_buf));
	printk("%s", cpp_buf);

	init_sensors(&fjalar_god);
	init_init(&fjalar_god); // will init filter when done
	init_flight_state(&fjalar_god);
	init_aerodynamics(&fjalar_god);
	init_communication(&fjalar_god);
	init_actuation(&fjalar_god);
	init_can(&fjalar_god);
	return 0;
}
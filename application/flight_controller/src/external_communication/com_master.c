/* 
This is the communication script, its purpose is to:
1) Send and recieve information to and from a telemetry modem (commonly used with our own modem, Brage), 
it uses a protocol specified in schema.proto.
2) Communicate with uart and usb, this is very important since it makes debugging possible and we can feed it simulated 
sensor information via usb/uart.
*/


#include <zephyr/console/tty.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <protocol.h>

#include "fjalar.h"
#include "commands.h"
#include "com_master.h"
#include "com_flash.h"
#include "com_lora.h"
#include "com_uart.h"
#include "com_usb.h"

LOG_MODULE_REGISTER(com_master, CONFIG_APP_COMMUNICATION_LOG_LEVEL);


#define SAMPLER_THREAD_PRIORITY 7
#define SAMPLER_THREAD_STACK_SIZE 2048

volatile bool terminate_communication = false;



K_THREAD_STACK_DEFINE(sampler_thread_stack, SAMPLER_THREAD_STACK_SIZE);
struct k_thread sampler_thread_data;
k_tid_t sampler_thread_id;
void sampler_thread(fjalar_t *fjalar, state_t *state, void *p2, void *p3);

_Static_assert(sizeof(struct padded_buf) % 4 == 0, "padded buffer length is not aligned");


void init_communication(fjalar_t *fjalar) {
	terminate_communication = false;

	sampler_thread_id = k_thread_create(
		&sampler_thread_data,
		sampler_thread_stack,
		K_THREAD_STACK_SIZEOF(sampler_thread_stack),
		(k_thread_entry_t) sampler_thread,
		fjalar, fjalar->ptr_state, NULL,
		SAMPLER_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(sampler_thread_id, "sampler");

	
	init_com_flash(fjalar);
	init_com_lora(fjalar);
	init_com_uart(fjalar);
	init_com_usb(fjalar);
}

int deinit_communication() {
	terminate_communication = true;
	int e = 0;
	#if DT_ALIAS_EXISTS(lora)
	e |= k_thread_join(&lora_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(data_usb)
	e |= k_thread_join(&usb_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(data_flash)
	e |= k_thread_join(&flash_thread_data, K_MSEC(1000));
	#endif

	#if DT_ALIAS_EXISTS(external_uart)
	e |= k_thread_join(&uart_thread_data, K_MSEC(1000));
	#endif

	e |= k_thread_join(&sampler_thread_data, K_MSEC(1000));
	return e;
}

void send_message(fjalar_t *fjalar, state_t *state, fjalar_message_t *msg, enum message_priority prio) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	switch(prio) {
		case MSG_PRIO_LOW:
			if (state->flight_state == STATE_IDLE || state->flight_state == STATE_LANDED) {
				break;
			}
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				//LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;

		case MSG_PRIO_HIGH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(lora)
			if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)) {
				LOG_ERR("could not insert into lora msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				//LOG_INF("could not insert data usb msgq");
			}
			#endif
			break;
	}
}

void send_response(fjalar_t *fjalar, fjalar_message_t *msg, enum com_channels channel) {
	struct padded_buf pbuf;
	if (msg->has_data == false) {
		msg->has_data = true;
		LOG_WRN("msg had has_data false");
	}
	int size = encode_fjalar_message(msg, pbuf.buf);
	LOG_DBG("encoded size %d", size);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	switch (channel) {
		case COM_CHAN_FLASH:
			#if DT_ALIAS_EXISTS(data_flash)
			if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
				LOG_INF("could not insert into flash msgq");
			}
			#endif
			break;

		case COM_CHAN_LORA:
			#if DT_ALIAS_EXISTS(lora)
			if (k_msgq_put(&lora_msgq, &pbuf, K_NO_WAIT)) {
				LOG_ERR("could not insert into lora msgq");
			}
			#endif
			break;

		case COM_CHAN_EXT_UART:
			#if DT_ALIAS_EXISTS(external_uart)
			if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert into uart msgq");
			}
			#endif
			break;

		case COM_CHAN_USB:
			#if DT_ALIAS_EXISTS(data_usb)
			if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)) {
				LOG_WRN("could not insert data usb msgq");
			}
			#endif
			break;
	}
}

void store_message(fjalar_t *fjalar, fjalar_message_t *msg) {
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("storing message w/ size %d", size);
	if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)) {
		LOG_INF("Could not store message");
	}
}

void sampler_thread(fjalar_t *fjalar, state_t *state, void *p2, void *p3) { // remove state ptr?
	while (true) {
		/*
		fjalar_message_t msg;
		msg.time = k_uptime_get_32();
		msg.has_data = true;
		msg.data.which_data = FJALAR_DATA_TELEMETRY_PACKET_TAG;
		msg.data.data.telemetry_packet.altitude = fjalar->altitude - fjalar->ground_level;
		msg.data.data.telemetry_packet.az = fjalar->az;
		msg.data.data.telemetry_packet.flight_state = state->flight_state;
		msg.data.data.telemetry_packet.velocity = fjalar->velocity;
		msg.data.data.telemetry_packet.battery = fjalar->battery_voltage;
		msg.data.data.telemetry_packet.latitude = fjalar->latitude;
		msg.data.data.telemetry_packet.longitude = fjalar->longitude;
		msg.data.data.telemetry_packet.flash_address = fjalar->flash_address;
		msg.data.data.telemetry_packet.pyro1_connected = fjalar->pyro1_sense;
		msg.data.data.telemetry_packet.pyro2_connected = fjalar->pyro2_sense;
		msg.data.data.telemetry_packet.pyro3_connected = fjalar->pyro3_sense;
		msg.data.data.telemetry_packet.sudo = fjalar->sudo;

		msg.data.data.hil_out.airbrake_percentage = 0.0f;
		msg.data.data.hil_out.main_deployed = fjalar->main_deployed;
		msg.data.data.hil_out.drogue_deployed = fjalar->drogue_deployed;
		send_message(fjalar, fjalar->ptr_state, &msg, MSG_PRIO_HIGH);
		*/
		k_msleep(1000);
	}
}
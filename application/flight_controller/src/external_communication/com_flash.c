#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>

#include "init.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "control.h"
#include "com_can.h"
#include "com_master.h"
#include "com_flash.h"



#define FLASH_THREAD_PRIORITY 7
#define FLASH_THREAD_STACK_SIZE 2048
#define FLASH_MSG_ENQUEUE_THREAD_PRIORITY 7
#define FLASH_MSG_ENQUEUE_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_flash, CONFIG_APP_COMMUNICATION_LOG_LEVEL);

K_MUTEX_DEFINE(flash_mutex);

K_THREAD_STACK_DEFINE(flash_thread_stack, FLASH_THREAD_STACK_SIZE);
struct k_thread flash_thread_data;
k_tid_t flash_thread_id;
K_THREAD_STACK_DEFINE(flash_msg_enqueue_thread_stack, FLASH_MSG_ENQUEUE_THREAD_STACK_SIZE);
struct k_thread flash_msg_enqueue_thread_data;
k_tid_t flash_msg_enqueue_thread_id;

void flash_thread(fjalar_t *fjalar, void *p2, void *p3);
void flash_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_flash(fjalar_t *fjalar){
    flash_thread_id = k_thread_create(
        &flash_thread_data,
        flash_thread_stack,
        K_THREAD_STACK_SIZEOF(flash_thread_stack),
        (k_thread_entry_t) flash_thread,
        fjalar, NULL, NULL,
        FLASH_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(flash_thread_id, "flash");

	flash_msg_enqueue_thread_id = k_thread_create(
		&flash_msg_enqueue_thread_data,
		flash_msg_enqueue_thread_stack,
		K_THREAD_STACK_SIZEOF(flash_msg_enqueue_thread_stack),
		(k_thread_entry_t) flash_msg_enqueue_thread,
		fjalar, NULL, NULL,
		FLASH_MSG_ENQUEUE_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(flash_thread_id, "flash enqueue");
}

K_MSGQ_DEFINE(flash_msgq, sizeof(struct padded_buf), 32, 4);

void flash_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return;
	}
	const uint8_t flash_reset_value = 0xff;
	fjalar->flash_address = 0;
	fjalar->flash_size = DT_PROP(DT_ALIAS(data_flash), size) / 8;
	int chunk_size = 1024;
	int num_cleared_values = 0;
	int chunk_start = 0;
	int ret;
	uint8_t buf[chunk_size];
	k_mutex_lock(&flash_mutex, K_FOREVER);
	while (true) {
		chunk_size = MIN(chunk_size, fjalar->flash_size - fjalar->flash_address);
		if (chunk_size < 1) {
			goto end;
		}
		ret = flash_read(flash_dev, fjalar->flash_address, buf, chunk_size);
		if (ret != 0) {
			LOG_ERR("Flash startup read fail");
			return;
		}
		k_usleep(10);
		chunk_start = fjalar->flash_address;
		fjalar->flash_address = fjalar->flash_address + chunk_size;
		for (int i = 0; i < chunk_size; i++) {
			if (buf[i] == flash_reset_value) {
				num_cleared_values += 1;
			} else {
				num_cleared_values = 0;
			}
			if (num_cleared_values == chunk_size) {
				fjalar->flash_address = chunk_start + i - num_cleared_values + 1;
				LOG_INF("found good address at %d", fjalar->flash_address);
				goto end;
			}
		}

	}
	end:
	k_mutex_unlock(&flash_mutex);
	LOG_WRN("initialized flash index to %d", fjalar->flash_address);
	k_msleep(10);
	while (true) {
		int ret;
		struct padded_buf pbuf;
		ret = k_msgq_get(&flash_msgq, &pbuf, K_FOREVER);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			k_mutex_lock(&flash_mutex, K_FOREVER);
			if (fjalar->flash_address + size >= fjalar->flash_size) {
				k_mutex_unlock(&flash_mutex);
				continue;
			}
			LOG_INF("wrote to flash address %d", fjalar->flash_address);
			int ret = flash_write(flash_dev, fjalar->flash_address, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not write to flash");
			} else {
				fjalar->flash_address += size;
			}
			k_mutex_unlock(&flash_mutex);
		} else {
			LOG_ERR("flash msgq error");
		}
	}
}

void flash_msg_enqueue(fjalar_message_t *msg){
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	if (k_msgq_put(&flash_msgq, &pbuf, K_NO_WAIT)){LOG_ERR("could not insert into flash msgq");}
}

void flash_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3){
	init_t            *init  = fjalar->ptr_init;
    position_filter_t *pos_kf = fjalar->ptr_pos_kf;
    attitude_filter_t *att_kf = fjalar->ptr_att_kf;
    aerodynamics_t    *aerodynamics = fjalar->ptr_aerodynamics;
    state_t           *state = fjalar->ptr_state;
    control_t         *control = fjalar->ptr_control;
    can_t             *can = fjalar->ptr_can;
	
	while (true){
		// imu 
		fjalar_message_t msg_imu = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_IMU_READING_TAG,
				.data.imu_reading = {
					.ax = pos_kf->raw_imu_ax,
					.ay = pos_kf->raw_imu_ay,
					.az = pos_kf->raw_imu_az,
					.gx = att_kf->raw_imu_gx,
					.gy = att_kf->raw_imu_gy,
					.gz = att_kf->raw_imu_gz,
				},
			},
		};
		flash_msg_enqueue(&msg_imu);

		// baro
		fjalar_message_t msg_baro = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_PRESSURE_READING_TAG,
				.data.pressure_reading = {
					.pressure = pos_kf->raw_baro_p
				},
			},
		};
		flash_msg_enqueue(&msg_baro);
		
		// gps
		fjalar_message_t msg_gps = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_GNSS_POSITION_TAG,
				.data.gnss_position = {
					.latitude = pos_kf->raw_gps_lat,
					.longitude = pos_kf->raw_gps_lon,
					.altitude = pos_kf->raw_gps_alt,
				},
			},
		};
		flash_msg_enqueue(&msg_gps);

		// StateEstimate
		fjalar_message_t msg_state_estimate = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_STATE_ESTIMATE_TAG,
				.data.state_estimate = {
					.x     = pos_kf->X_data[0],
					.y     = pos_kf->X_data[1],
					.z     = pos_kf->X_data[2],
					.vx    = pos_kf->X_data[3],
					.vy    = pos_kf->X_data[4],
					.vz    = pos_kf->X_data[5],
					.roll  = att_kf->X_data[0],
					.pitch = att_kf->X_data[1],
					.yaw   = att_kf->X_data[2],
				},
			},
		};
		flash_msg_enqueue(&msg_state_estimate);

		// FlightState
		fjalar_message_t msg_flight_state = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_FLIGHT_STATE_TAG,
				.data.flight_state = state->flight_state
			},
		};
		flash_msg_enqueue(&msg_flight_state);

		// FlightEvent
		fjalar_message_t msg_flight_event = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_FLIGHT_EVENT_TAG,
				.data.flight_event = {
					.launch              = state->event_launch,
					.burnout             = state->event_burnout,
					.apogee              = state->event_apogee,
					.drogue_deploy       = state->event_drogue_deployed,
					.main_deploy         = state->event_main_deployed,
					.landed              = state->event_landed,
					.above_acs_threshold = state->event_above_acs_threshold,
				},
			},
		};
		flash_msg_enqueue(&msg_flight_event);

		// CANBus
		fjalar_message_t msg_can_bus = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_CAN_BUS_TAG,
				.data.can_bus = {
					.can_started           = can->can_started,
					.loki_latest_rx_time   = can->loki_latest_rx_time,
					.loki_latest_tx_time   = can->loki_latest_tx_time,
					//.sigurd_latest_rx_time = sigurd_context->sigurd_latest_rx_time,
					//.sigurd_latest_tx_time = can->loki_latest_tx_time,
				},
			},
		};
		flash_msg_enqueue(&msg_can_bus);
		
		// PyroStatus
		fjalar_message_t msg_pyro_status = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data = FJALAR_DATA_PYRO_STATUS_TAG,
				.data.pyro_status = {
					.pyro1_connected = fjalar->pyro1_sense,
					.pyro2_connected = fjalar->pyro2_sense,
					.pyro3_connected = fjalar->pyro3_sense,
				},
			},
		};
		flash_msg_enqueue(&msg_pyro_status);

		// LokiToFjalar
		fjalar_message_t msg_loki = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data         = FJALAR_DATA_CAN_LOKI_TO_FJALAR_TAG,
				.data.can_loki_to_fjalar = {
					.loki_state           = can->loki_state,
					.loki_substate        = can->loki_sub_state,
					.loki_current_angle   = can->loki_angle,
					.loki_battery_voltage = can->loki_battery_voltage,
				},
			},
		};
		flash_msg_enqueue(&msg_loki);
		

		k_msleep(1000);
	}
}


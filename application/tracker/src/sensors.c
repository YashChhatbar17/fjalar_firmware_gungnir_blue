#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <math.h>



#include <minmea.h>

#include "tracker.h"

LOG_MODULE_REGISTER(sensors, CONFIG_APP_SENSORS_LOG_LEVEL);

#define GPS_THREAD_STACK_SIZE 2048
#define GPS_THREAD_PRIORITY 7

#define VBAT_THREAD_PRIORITY 7
#define VBAT_THREAD_STACK_SIZE 2048

#if DT_ALIAS_EXISTS(gps_uart)
K_THREAD_STACK_DEFINE(gps_thread_stack, GPS_THREAD_STACK_SIZE);
struct k_thread gps_thread_data;
k_tid_t gps_thread_id;
#endif

K_THREAD_STACK_DEFINE(vbat_thread_stack, VBAT_THREAD_STACK_SIZE);
struct k_thread vbat_thread_data;
k_tid_t vbat_thread_id;

void gps_thread(tracker_t *tracker, void *p2, void *p3);
void vbat_thread(tracker_t *fjalar, void *p2, void *p3);

void init_sensors(tracker_t *tracker) {
	#if DT_ALIAS_EXISTS(gps_uart)
	gps_thread_id = k_thread_create(
		&gps_thread_data,
		gps_thread_stack,
		K_THREAD_STACK_SIZEOF(gps_thread_stack),
		(k_thread_entry_t) gps_thread,
		tracker, NULL, NULL,
		GPS_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(gps_thread_id, "gps");
	#endif

    #if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	vbat_thread_id = k_thread_create(
		&vbat_thread_data,
		vbat_thread_stack,
		K_THREAD_STACK_SIZEOF(vbat_thread_stack),
		(k_thread_entry_t) vbat_thread,
		tracker, NULL, NULL,
		VBAT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(vbat_thread_id, "bat_adc");
	#endif
}

#if DT_NODE_EXISTS(DT_ALIAS(gps_uart))
void handle_nmea(tracker_t *tracker, char *buf, int len) {
	LOG_DBG("Handling nmea message");
	switch (minmea_sentence_id(buf, true)) {
		case MINMEA_INVALID:
			LOG_ERR("got invalid nmea message \"%.*s\"", len, buf);
			break;
		case MINMEA_UNKNOWN:
			LOG_DBG("got unknown nmea message (quectel probably) \"%.*s\"", len, buf);
			break;

		case MINMEA_SENTENCE_RMC:
			LOG_DBG("got RMC nmea message");
			struct minmea_sentence_rmc rmc;
			minmea_parse_rmc(&rmc, buf);
			tracker->longitude = minmea_tocoord(&rmc.longitude);
			tracker->latitude = minmea_tocoord(&rmc.latitude);
			break;

		default:
			LOG_DBG("got unhandled nmea message \"%.*s\"", len, buf);
			break;
	}
}

void gps_uart_cb(const struct device *dev, void *user_data) {
	static uint8_t nmea_buf[255];
	static int nmea_index = 0;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	while (true) {
		uint8_t fifo_buf[255];
		int fifo_len = uart_fifo_read(dev, fifo_buf, sizeof(fifo_buf));
		if (fifo_len <= 0) {
			break;
		}

		for (int i = 0; i < fifo_len; i++) {
			char byte = fifo_buf[i];
			if (byte == '$') {
				nmea_index = 0;
			}
			if (byte == '\n' || byte == '\r') {
				if (nmea_index != 0) {
					nmea_buf[nmea_index] = '\0';
					handle_nmea(&tracker_god, nmea_buf, nmea_index); // TODO: don't use tracker god
				}
				nmea_index = 0;
				continue;
			}
			if (nmea_index < sizeof(nmea_buf) - 1) {
				nmea_buf[nmea_index] = byte;
				nmea_index++;
				continue;
			}
			continue;
		}
	}
}

void gps_thread(tracker_t *tracker, void *p2, void *p3) {
    const struct device *const gps_dev = DEVICE_DT_GET(DT_ALIAS(gps_uart));
    if (!device_is_ready(gps_dev)) {
		LOG_ERR("gps is not ready");
		return;
	}
    int ret;
    struct uart_config uart_config = {
		.baudrate = 9600,
		.data_bits = UART_CFG_DATA_BITS_8,
		.flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits = UART_CFG_STOP_BITS_1
	};
	ret = uart_configure(gps_dev, &uart_config);
	uart_irq_callback_set(gps_dev, gps_uart_cb);
	uart_irq_rx_enable(gps_dev);

	const char pulse_msg[] = "$PMTK285,2,100*3E\r\n"; // blink
	for (int i = 0; i < strlen(pulse_msg); i++) {
		uart_poll_out(gps_dev, pulse_msg[i]);
	}

	const char mode_msg[] = "$PMTK886,3*2B\r\n"; // balloon mode
	for (int i = 0; i < strlen(mode_msg); i++) {
		uart_poll_out(gps_dev, mode_msg[i]);
	}
}
#endif

// Function to calculate the Haversine distance between two points
float haversine_distance(float lat1, float lon1, float lat2, float lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 = 3.1415 / 180.0 * lat1;
    lon1 = 3.1415 / 180.0 * lon1;
    lat2 = 3.1415 / 180.0 * lat2;
    lon2 = 3.1415 / 180.0 * lon2;

    // Calculate the differences in latitude and longitude
    float dlat = lat2 - lat1;
    float dlon = lon2 - lon1;

    // Calculate the Haversine distance
    float a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float distance = 6371000.0 * c;

    return distance;
}

#if DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
void vbat_thread(tracker_t *tracker, void *p2, void *p3) {
	#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
	static const struct adc_dt_spec adc_channels[] = {
		DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
					DT_SPEC_AND_COMMA)
	};
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!device_is_ready(adc_channels[i].dev)) {
			LOG_ERR("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return;
		}
		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			LOG_ERR("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}

	while (true) {
		k_msleep(100);
		uint16_t sample;
		struct adc_sequence sequence;
		struct adc_sequence_options options;
		options.interval_us = 0;
		options.extra_samplings = 0;
		options.callback = NULL;
		sequence.buffer = &sample;
		sequence.buffer_size = sizeof(sample);
		sequence.options = &options;
		err = adc_sequence_init_dt(&adc_channels[0], &sequence);
		if (err) {
			LOG_ERR("Could not init adc channel");
			continue;
		}
		err = adc_read(adc_channels[0].dev, &sequence);
		if (err) {
			LOG_ERR("Could not read adc channel");
			continue;
		}
		uint32_t mv = sample;
		float volt;
		// const float numerator = DT_PROP(DT_PATH(zephyr_user), battery_numerator);
		// const float denominator = DT_PROP(DT_PATH(zephyr_user), battery_denominator);
		err = adc_raw_to_millivolts_dt(&adc_channels[0],
						       &mv);
		if (err) {
			LOG_ERR("Could not convert adc raw");
		}
		volt = (mv / 1000.0);
		tracker->battery_voltage = volt;
		LOG_DBG("battery voltage raw: %f scaled: %f", mv / 1000.0, volt);
	}
}
#endif

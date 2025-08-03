#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/lora.h>

#include "com_master.h"
#include "com_lora.h"

K_MSGQ_DEFINE(lora_msgq, sizeof(struct padded_buf), 5, 4);

#define LORA_THREAD_PRIORITY 7
#define LORA_THREAD_STACK_SIZE 2048

#define LORA_TRANSMIT true
#define LORA_RECEIVE false

LOG_MODULE_REGISTER(com_lora, CONFIG_APP_COMMUNICATION_LOG_LEVEL);

K_THREAD_STACK_DEFINE(lora_thread_stack, LORA_THREAD_STACK_SIZE);
struct k_thread lora_thread_data;
k_tid_t lora_thread_id;

void lora_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_lora(fjalar_t *fjalar){
    lora_thread_id = k_thread_create(
        &lora_thread_data,
        lora_thread_stack,
        K_THREAD_STACK_SIZEOF(lora_thread_stack),
        (k_thread_entry_t) lora_thread,
        fjalar, NULL, NULL,
        LORA_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(lora_thread_id, "lora");
} 

int lora_configure(const struct device *dev, bool transmit) {
	static uint8_t current_mode = -1;
	if (current_mode == transmit) {
		LOG_DBG("Mode already configured");
		return 0;
	}
	struct lora_modem_config config = PROTOCOL_ZEPHYR_LORA_CONFIG;
	config.tx = transmit;
	config.tx_power = 10;
	int ret = lora_config(dev, &config);
	if (ret < 0) {
		LOG_ERR("Could not configure lora %d", ret);
		return ret;
	}
	if (transmit) {
		LOG_DBG("LoRa configured to tx");
	} else {
		LOG_DBG("LoRa configured to rx");
	}
	current_mode = transmit;
	return ret;
}

struct lora_rx {
	uint8_t buf[PROTOCOL_BUFFER_LENGTH];
	uint16_t size;
	int16_t rssi;
	int8_t snr;
} __attribute__((aligned(4)));

K_MSGQ_DEFINE(lora_rx_msgq, sizeof(struct lora_rx), 5, 4);

void lora_cb(const struct device *dev, uint8_t *buf, uint16_t size, int16_t rssi, int8_t snr, void *user) {
	struct lora_rx rx;
	rx.size = MIN(size, PROTOCOL_BUFFER_LENGTH);
	memcpy(rx.buf, buf, rx.size);
	rx.rssi = rssi;
	rx.snr = snr;
	k_msgq_put(&lora_rx_msgq, &rx, K_NO_WAIT);
}

void lora_thread(fjalar_t *fjalar, void* p2, void* p3) {
	const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora));
	if (!device_is_ready(lora_dev)) {
		LOG_ERR("LoRa is not ready");
		return;
	}
	struct lora_rx rx;
	while (true) {
		int ret;
		struct padded_buf pbuf;
		struct k_poll_event events[2] = {
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_msgq),
			K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
											K_POLL_MODE_NOTIFY_ONLY,
											&lora_rx_msgq),
		};

		ret = lora_configure(lora_dev, LORA_RECEIVE);
		if (ret) {
			LOG_ERR("LoRa rx configure failed");
		} else {
			LOG_DBG("LoRa rxing");
		}
		// TODO: The void user data was recently added, use this to pass data instead of the global variable
		lora_recv_async(lora_dev, lora_cb, NULL);
		// k_poll(&events[1], 2, K_MSEC(10000)); //poll only rx first to not interrupt messages
		k_poll(events, 2, K_FOREVER);

		ret = k_msgq_get(&lora_rx_msgq, &rx, K_NO_WAIT);
		if (ret == 0) {
			events[1].state = K_POLL_STATE_NOT_READY;
			LOG_INF("received LoRa msg");
			struct protocol_state ps;
			reset_protocol_state(&ps);
			handle_fjalar_buf(&ps, fjalar, rx.buf, rx.size, COM_CHAN_LORA);
		}

		ret = k_msgq_get(&lora_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			events[0].state = K_POLL_STATE_NOT_READY;
			lora_recv_async(lora_dev, NULL, NULL);
			ret = lora_configure(lora_dev, LORA_TRANSMIT);
			if (ret) {
				LOG_ERR("LORA tx configure failed");
			}else {
				LOG_DBG("LoRa txing");
			}
			int size = get_encoded_message_length(pbuf.buf);
			ret = lora_send(lora_dev, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not send LoRa message");
			} else {
				LOG_DBG("Sent lora packet with size %d", size);
			}
		}
	}
}
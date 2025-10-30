#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include "com_master.h"
#include "com_usb.h"


#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 2048
#define USB_MSG_ENQUEUE_THREAD_PRIORITY 7
#define USB_MSG_ENQUEUE_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_usb, LOG_LEVEL_INF);
K_MSGQ_DEFINE(usb_msgq, sizeof(struct padded_buf), 32, 4);


K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_thread_data;
k_tid_t usb_thread_id;
K_THREAD_STACK_DEFINE(usb_msg_enqueue_thread_stack, USB_MSG_ENQUEUE_THREAD_STACK_SIZE);
struct k_thread usb_msg_enqueue_thread_data;
k_tid_t usb_msg_enqueue_thread_id;

void usb_thread(fjalar_t *fjalar, void *p2, void *p3);
void usb_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_usb(fjalar_t *fjalar){
    usb_thread_id = k_thread_create(
        &usb_thread_data,
        usb_thread_stack,
        K_THREAD_STACK_SIZEOF(usb_thread_stack),
        (k_thread_entry_t) usb_thread,
        fjalar, NULL, NULL,
        USB_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(usb_thread_id, "data usb");

	usb_msg_enqueue_thread_id = k_thread_create(
		&usb_msg_enqueue_thread_data,
		usb_msg_enqueue_thread_stack,
		K_THREAD_STACK_SIZEOF(usb_msg_enqueue_thread_stack),
		(k_thread_entry_t) usb_msg_enqueue_thread,
		fjalar, NULL, NULL,
		USB_MSG_ENQUEUE_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(usb_msg_enqueue_thread_id, "usb enqueue");

}

void usb_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *usb_dev = DEVICE_DT_GET(DT_ALIAS(data_usb));
	if (!device_is_ready(usb_dev)) {
		LOG_ERR("data usb not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(usb_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_USB);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("USB read error");
		}

		// tx
		struct padded_buf pbuf;
		ret = k_msgq_get(&usb_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(usb_dev, pbuf.buf[i]);
			}
		} 
		k_msleep(1);
	}
}

void usb_msg_enqueue(fjalar_message_t *msg){
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	if (k_msgq_put(&usb_msgq, &pbuf, K_NO_WAIT)){LOG_ERR("could not insert into usb msgq");}
}

void usb_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3){
	while (true){
		// create msg
		// FjalarInfo
		/*
		fjalar_info_t msg_fjalar_info = {
			.time = k_uptime_get_32(),
			.has_data = true,
			.data = {
				.which_data         = FJALAR_DATA_FJALAR_INFO_TAG,
				.data.fjalar_info = {
					fjalar_battery_voltage = fjalar->battery_voltage;
					flash_address = fjalar->flash_adress;
					sudo = fjalar->sudo;
				},
			},
		};
		usb_msg_enqueue(&msg_fjalar_info);
		*/
	
		// encode and put into msgq
		//usb_msg_enqueue(&msg);
		k_msleep(100);
	}
}
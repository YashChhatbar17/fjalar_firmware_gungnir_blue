#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include "com_master.h"
#include "com_usb.h"


#define USB_THREAD_PRIORITY 7
#define USB_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_usb, CONFIG_APP_COMMUNICATION_LOG_LEVEL);
K_MSGQ_DEFINE(usb_msgq, sizeof(struct padded_buf), 32, 4);


K_THREAD_STACK_DEFINE(usb_thread_stack, USB_THREAD_STACK_SIZE);
struct k_thread usb_thread_data;
k_tid_t usb_thread_id;

void usb_thread(fjalar_t *fjalar, void *p2, void *p3);

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
#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

#include "com_master.h"
#include "com_uart.h"


#define UART_THREAD_PRIORITY 7
#define UART_THREAD_STACK_SIZE 2048
#define UART_MSG_ENQUEUE_THREAD_PRIORITY 7
#define UART_MSG_ENQUEUE_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_uart, CONFIG_APP_COMMUNICATION_LOG_LEVEL);

K_MSGQ_DEFINE(uart_msgq, sizeof(struct padded_buf), 32, 4);

K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SIZE);
struct k_thread uart_thread_data;
k_tid_t uart_thread_id;
K_THREAD_STACK_DEFINE(uart_msg_enqueue_thread_stack, UART_MSG_ENQUEUE_THREAD_STACK_SIZE);
struct k_thread uart_msg_enqueue_thread_data;
k_tid_t uart_msg_enqueue_thread_id;

void uart_thread(fjalar_t *fjalar, void *p2, void *p3);
void uart_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_uart(fjalar_t *fjalar){
    uart_thread_id = k_thread_create(
        &uart_thread_data,
        uart_thread_stack,
        K_THREAD_STACK_SIZEOF(uart_thread_stack),
        (k_thread_entry_t) uart_thread,
        fjalar, NULL, NULL,
        UART_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(uart_thread_id, "external uart");

	uart_msg_enqueue_thread_id = k_thread_create(
		&uart_msg_enqueue_thread_data,
		uart_msg_enqueue_thread_stack,
		K_THREAD_STACK_SIZEOF(uart_msg_enqueue_thread_stack),
		(k_thread_entry_t) uart_msg_enqueue_thread,
		fjalar, NULL, NULL,
		UART_MSG_ENQUEUE_THREAD_PRIORITY, 0, K_NO_WAIT
	);
	k_thread_name_set(uart_msg_enqueue_thread_id, "uart enqueue");

}

void uart_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(external_uart));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("External uart not ready");
		return;
	}
	struct protocol_state ps;
	reset_protocol_state(&ps);
	while(1) {
		// rx
		int ret;
		uint8_t byte;
		ret = uart_poll_in(uart_dev, &byte);
		if (ret == 0) {
			handle_fjalar_buf(&ps, fjalar, &byte, 1, COM_CHAN_EXT_UART);
			continue;
		} else
		if (ret != -1) {
			LOG_ERR("UART read error");
		}

		// tx
		struct padded_buf pbuf;
		ret = k_msgq_get(&uart_msgq, &pbuf, K_NO_WAIT);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			for (int i = 0; i < size; i++) {
				uart_poll_out(uart_dev, pbuf.buf[i]);
			}
		}
		k_msleep(1);
	}
}

void uart_msg_enqueue(fjalar_message_t *msg){
	struct padded_buf pbuf;
	int size = encode_fjalar_message(msg, pbuf.buf);
	if (size < 0) {
		LOG_ERR("encode_fjalar_message failed");
		return;
	}
	LOG_DBG("sending message w/ size %d", size);
	if (k_msgq_put(&uart_msgq, &pbuf, K_NO_WAIT)){LOG_ERR("could not insert into uart msgq");}
}

void uart_msg_enqueue_thread(fjalar_t *fjalar, void *p2, void *p3){
	while (true){
		// create msg

		// encode and put into msgq
		//uart_msg_enqueue(&msg);
		k_msleep(100);
	}
}
#pragma once

#include <zephyr/kernel.h>

#include "fjalar.h"
#include "flight_state.h"
#include "protocol.h"

enum message_priority {
	MSG_PRIO_LOW,
	MSG_PRIO_MEDIUM,
	MSG_PRIO_HIGH,
	MSG_PRIO_MAX,
};

struct padded_buf {uint8_t buf[PROTOCOL_BUFFER_LENGTH];} __attribute__((aligned(4)));
extern struct k_msgq flash_msgq;
extern struct k_msgq uart_msgq;
extern struct k_msgq usb_msgq;
extern struct k_msgq lora_msgq;


void init_communication(fjalar_t *fjalar);
void send_message(fjalar_t *fjalar, state_t *state, fjalar_message_t *msg, enum message_priority prio);
void send_response(fjalar_t *fjalar, fjalar_message_t *msg, enum com_channels channel);

void clear_flash(fjalar_t *fjalar);
void read_flash(fjalar_t *fjalar, uint8_t *buf, uint32_t index, uint8_t len);
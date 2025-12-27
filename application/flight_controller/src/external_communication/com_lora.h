#pragma once

extern struct k_thread lora_thread_data;
extern k_tid_t lora_thread_id;

extern struct k_msgq lora_msgq;

void init_com_lora(fjalar_t *fjalar);

/* LoRa command IDs */
typedef enum {
    LORA_CMD_INIT   = 0x01,
    LORA_CMD_LAUNCH = 0x02,
} lora_cmd_t;
/* ACK packet */
typedef struct __packed {
    uint8_t cmd;     // Command being acknowledged
    uint8_t status;  // 1 = success, 0 = failure
} lora_ack_t;

/* ACK sender */
void lora_send_ack(uint8_t cmd, bool success);

/* Command handler */
void lora_command_handler(uint8_t cmd);

typedef struct lora {
    bool LORA_READY_INITIATE_FJALAR;
    bool LORA_READY_LAUNCH_FJALAR;
} lora_t;
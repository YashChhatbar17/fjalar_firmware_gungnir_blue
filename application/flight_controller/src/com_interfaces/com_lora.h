#pragma once

extern struct k_thread lora_thread_data;
extern k_tid_t      lora_thread_id;

extern struct k_msgq lora_msgq;

void init_com_lora(fjalar_t *fjalar);


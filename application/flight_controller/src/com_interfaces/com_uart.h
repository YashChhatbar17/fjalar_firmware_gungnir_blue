#pragma once

extern struct k_thread uart_thread_data;
extern k_tid_t      uart_thread_id;

extern struct k_msgq uart_msgq;

void init_com_uart(fjalar_t *fjalar);
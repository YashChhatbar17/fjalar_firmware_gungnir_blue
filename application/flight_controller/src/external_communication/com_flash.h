#pragma once


extern struct k_thread flash_thread_data;
extern k_tid_t      flash_thread_id;

extern struct k_mutex flash_mutex;

void init_com_flash(fjalar_t *fjalar);
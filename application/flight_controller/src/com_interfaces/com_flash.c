#include <zephyr/kernel.h>
#include <protocol.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>

#include "com_master.h"
#include "com_flash.h"



#define FLASH_THREAD_PRIORITY 7
#define FLASH_THREAD_STACK_SIZE 2048

LOG_MODULE_REGISTER(com_flash, CONFIG_APP_COMMUNICATION_LOG_LEVEL);

K_MUTEX_DEFINE(flash_mutex);

K_THREAD_STACK_DEFINE(flash_thread_stack, FLASH_THREAD_STACK_SIZE);
struct k_thread flash_thread_data;
k_tid_t flash_thread_id;

void flash_thread(fjalar_t *fjalar, void *p2, void *p3);

void init_com_flash(fjalar_t *fjalar){
    flash_thread_id = k_thread_create(
        &flash_thread_data,
        flash_thread_stack,
        K_THREAD_STACK_SIZEOF(flash_thread_stack),
        (k_thread_entry_t) flash_thread,
        fjalar, NULL, NULL,
        FLASH_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(flash_thread_id, "data flash");
}

K_MSGQ_DEFINE(flash_msgq, sizeof(struct padded_buf), 32, 4);

void read_flash(fjalar_t *fjalar, uint8_t *buf, uint32_t index, uint8_t len) {
	LOG_DBG("Reading flash");
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_read(flash_dev, index, buf, len);
	k_mutex_unlock(&flash_mutex);
}

void clear_flash(fjalar_t *fjalar) {
	LOG_WRN("Clearing flash");
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	k_mutex_lock(&flash_mutex, K_FOREVER);
	flash_erase(flash_dev, 0, fjalar->flash_size);
	fjalar->flash_address = 0;
	k_mutex_unlock(&flash_mutex);
}

void flash_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return;
	}
	const uint8_t flash_reset_value = 0xff;
	fjalar->flash_address = 0;
	fjalar->flash_size = DT_PROP(DT_ALIAS(data_flash), size) / 8;
	int chunk_size = 1024;
	int num_cleared_values = 0;
	int chunk_start = 0;
	int ret;
	uint8_t buf[chunk_size];
	k_mutex_lock(&flash_mutex, K_FOREVER);
	while (true) {
		chunk_size = MIN(chunk_size, fjalar->flash_size - fjalar->flash_address);
		if (chunk_size < 1) {
			goto end;
		}
		ret = flash_read(flash_dev, fjalar->flash_address, buf, chunk_size);
		if (ret != 0) {
			LOG_ERR("Flash startup read fail");
			return;
		}
		k_usleep(10);
		chunk_start = fjalar->flash_address;
		fjalar->flash_address = fjalar->flash_address + chunk_size;
		for (int i = 0; i < chunk_size; i++) {
			if (buf[i] == flash_reset_value) {
				num_cleared_values += 1;
			} else {
				num_cleared_values = 0;
			}
			if (num_cleared_values == chunk_size) {
				fjalar->flash_address = chunk_start + i - num_cleared_values + 1;
				LOG_INF("found good address at %d", fjalar->flash_address);
				goto end;
			}
		}

	}
	end:
	k_mutex_unlock(&flash_mutex);
	LOG_WRN("initialized flash index to %d", fjalar->flash_address);
	k_msleep(10);
	while (true) {
		int ret;
		struct padded_buf pbuf;
		ret = k_msgq_get(&flash_msgq, &pbuf, K_FOREVER);
		if (ret == 0) {
			int size = get_encoded_message_length(pbuf.buf);
			k_mutex_lock(&flash_mutex, K_FOREVER);
			if (fjalar->flash_address + size >= fjalar->flash_size) {
				k_mutex_unlock(&flash_mutex);
				continue;
			}
			LOG_INF("wrote to flash address %d", fjalar->flash_address);
			int ret = flash_write(flash_dev, fjalar->flash_address, pbuf.buf, size);
			if (ret) {
				LOG_ERR("Could not write to flash");
			} else {
				fjalar->flash_address += size;
			}
			k_mutex_unlock(&flash_mutex);
		} else {
			LOG_ERR("flash msgq error");
		}
	}
}
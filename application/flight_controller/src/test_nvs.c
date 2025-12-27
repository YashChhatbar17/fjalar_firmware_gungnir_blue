#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(test_nvs, LOG_LEVEL_INF);

#define FLIGHT_DATA_ID 1
#define NVS_OFFSET 0x08080000     // Pick an unused flash offset
#define NVS_SECTOR_SIZE 4096
#define NVS_SECTOR_COUNT 4

static struct nvs_fs fs;

// Initialize NVS
static int cmd_test_nvs_init(const struct shell *sh, size_t argc, char **argv) {
    fs.flash_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    fs.offset = NVS_OFFSET;
    fs.sector_size = NVS_SECTOR_SIZE;
    fs.sector_count = NVS_SECTOR_COUNT;

    int ret = nvs_mount(&fs);
    if (ret) {
        shell_error(sh, "NVS mount failed: %d", ret);
        return ret;
    }

    shell_print(sh, "✓ NVS mounted at offset 0x%08X", NVS_OFFSET);
    return 0;
}

// Write a test entry
static int cmd_test_nvs_write(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) {
        shell_error(sh, "Usage: nvs write <data>");
        return -1;
    }

    int ret = nvs_write(&fs, FLIGHT_DATA_ID, argv[1], strlen(argv[1]));
    if (ret < 0) {
        shell_error(sh, "NVS write failed: %d", ret);
        return ret;
    }

    shell_print(sh, "✓ Written to NVS: %s", argv[1]);
    return 0;
}

// Read the test entry
static int cmd_test_nvs_read(const struct shell *sh, size_t argc, char **argv) {
    char buf[128];
    int ret = nvs_read(&fs, FLIGHT_DATA_ID, buf, sizeof(buf));
    if (ret < 0) {
        shell_error(sh, "NVS read failed: %d", ret);
        return ret;
    }

    buf[ret] = '\0'; // Null-terminate
    shell_print(sh, "✓ Read from NVS: %s", buf);
    return 0;
}

// Clear test entry
static int cmd_test_nvs_clear(const struct shell *sh, size_t argc, char **argv) {
    int ret = nvs_delete(&fs, FLIGHT_DATA_ID);
    if (ret) {
        shell_error(sh, "NVS delete failed: %d", ret);
        return ret;
    }

    shell_print(sh, "✓ NVS entry cleared");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_nvs,
    SHELL_CMD(init, NULL, "Mount internal flash for NVS", cmd_test_nvs_init),
    SHELL_CMD(write, NULL, "Write data to NVS", cmd_test_nvs_write),
    SHELL_CMD(read, NULL, "Read data from NVS", cmd_test_nvs_read),
    SHELL_CMD(clear, NULL, "Delete data from NVS", cmd_test_nvs_clear),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(nvs, &sub_nvs, "Internal flash NVS test commands", NULL);

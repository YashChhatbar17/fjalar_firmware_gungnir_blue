#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/lora.h>
#include "fjalar.h"
#include "flight_state.h"
#include "com_lora.h"
#include "init.h"

LOG_MODULE_REGISTER(lora_test, LOG_LEVEL_INF);

extern fjalar_t fjalar_god;
struct device *lora_dev;  // global definition
/* ---------------------------------------------------- */
/* Helpers: simulate LoRa commands and update state    */
/* ---------------------------------------------------- */
static void simulate_lora_init(void) {
    LOG_INF("Simulating LoRa INIT command...");
    fjalar_god.ptr_lora->LORA_READY_INITIATE_FJALAR = true;
    k_msleep(200);  // let flight_state_thread react
    LOG_INF("Flight state now: %d (expected: STATE_AWAITING_INIT)",
            fjalar_god.ptr_state->flight_state);
}

static void simulate_lora_launch(void) {
    LOG_INF("Simulating LoRa LAUNCH command...");
    fjalar_god.ptr_lora->LORA_READY_LAUNCH_FJALAR = true;
    k_msleep(200);
    LOG_INF("Flight state now: %d (expected: STATE_AWAITING_LAUNCH)",
            fjalar_god.ptr_state->flight_state);
}


/* ---------------------------------------------------- */
/* Shell command: run LoRa self-contained test         */
/* ---------------------------------------------------- */
static int cmd_lora_test(const struct shell *sh, size_t argc, char **argv) {
    shell_print(sh, "\n=== Running LoRa self-contained test ===\n");

    simulate_lora_init();
    k_msleep(50);

    simulate_lora_launch();
    k_msleep(50);

    shell_print(sh, "=== LoRa test complete ===");
    shell_print(sh, "Final flight state: %d", fjalar_god.ptr_state->flight_state);
    return 0;
}

/* ---------------------------------------------------- */
/* Register shell command                               */
/* ---------------------------------------------------- */
SHELL_CMD_REGISTER(lora_test, NULL, "Run self-contained LoRa INIT/LAUNCH test", cmd_lora_test);

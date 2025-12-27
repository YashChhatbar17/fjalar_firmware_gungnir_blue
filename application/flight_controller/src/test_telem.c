#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

LOG_MODULE_REGISTER(test_telem, LOG_LEVEL_INF);

/* --------------------------
   Mocked message structures
   -------------------------- */
struct filter_output_msg {
    uint32_t timestamp;
    float position[3];   // x, y, z
    float v_norm;        // velocity
};

struct flight_state_output_msg {
    uint8_t flight_state;
};

/* --------------------------
   Mocked message queues
   -------------------------- */
#define MSGQ_SIZE 8
static struct filter_output_msg filter_output_msgq[MSGQ_SIZE];
static struct flight_state_output_msg flight_state_output_msgq[MSGQ_SIZE];
static int filter_index = 0;
static int state_index = 0;

/* Fill queues with mock data */
static void populate_mock_data(void) {
    srand(time(NULL));
    for (int i = 0; i < MSGQ_SIZE; i++) {
        filter_output_msgq[i].timestamp = i * 100;
        filter_output_msgq[i].position[2] = 100.0f + i * 5.0f;   // altitude
        filter_output_msgq[i].v_norm = 10.0f + i * 2.0f;          // velocity

        flight_state_output_msgq[i].flight_state = i % 5;         // mock state
    }
}

/* Peek latest message */
static int k_msgq_peek_filter(struct filter_output_msg *msg) {
    if (filter_index >= MSGQ_SIZE) filter_index = 0;
    *msg = filter_output_msgq[filter_index++];
    return 0;   // 0 = success
}

static int k_msgq_peek_state(struct flight_state_output_msg *msg) {
    if (state_index >= MSGQ_SIZE) state_index = 0;
    *msg = flight_state_output_msgq[state_index++];
    return 0;
}

/* --------------------------
   Telemetry thread
   -------------------------- */
static bool telemetry_enabled = false;
static K_THREAD_STACK_DEFINE(telem_stack, 2048);
static struct k_thread telem_thread_data;

void telemetry_thread(void *p1, void *p2, void *p3) {
    struct filter_output_msg filter_msg;
    struct flight_state_output_msg state_msg;

    while (telemetry_enabled) {
        if (k_msgq_peek_filter(&filter_msg) == 0 &&
            k_msgq_peek_state(&state_msg) == 0) {

            printk("%u,%d,%.2f,%.2f\n",
                filter_msg.timestamp,
                state_msg.flight_state,
                filter_msg.position[2],
                filter_msg.v_norm);
        }
        k_msleep(100);  // 10 Hz
    }
}

/* --------------------------
   Shell commands
   -------------------------- */
static int cmd_telem_start(const struct shell *sh, size_t argc, char **argv) {
    if (telemetry_enabled) {
        shell_error(sh, "Telemetry already running");
        return -1;
    }

    telemetry_enabled = true;
    k_thread_create(&telem_thread_data, telem_stack,
                    K_THREAD_STACK_SIZEOF(telem_stack),
                    telemetry_thread, NULL, NULL, NULL,
                    7, 0, K_NO_WAIT);
    shell_print(sh, "Telemetry started (10 Hz)");
    return 0;
}

static int cmd_telem_stop(const struct shell *sh, size_t argc, char **argv) {
    if (!telemetry_enabled) {
        shell_error(sh, "Telemetry not running");
        return -1;
    }

    telemetry_enabled = false;
    k_thread_join(&telem_thread_data, K_FOREVER);
    shell_print(sh, "Telemetry stopped");
    return 0;
}

static int cmd_telem_snapshot(const struct shell *sh, size_t argc, char **argv) {
    struct filter_output_msg filter_msg;
    struct flight_state_output_msg state_msg;

    if (k_msgq_peek_filter(&filter_msg) != 0 ||
        k_msgq_peek_state(&state_msg) != 0) {
        shell_error(sh, "No data available");
        return -1;
    }

    shell_print(sh, "Telemetry Snapshot:");
    shell_print(sh, "Timestamp: %u", filter_msg.timestamp);
    shell_print(sh, "State: %d", state_msg.flight_state);
    shell_print(sh, "Altitude: %.2f m", filter_msg.position[2]);
    shell_print(sh, "Velocity: %.2f m/s", filter_msg.v_norm);

    return 0;
}

/* --------------------------
   Shell registration
   -------------------------- */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_telem,
    SHELL_CMD(start, NULL, "Start telemetry stream", cmd_telem_start),
    SHELL_CMD(stop, NULL, "Stop telemetry stream", cmd_telem_stop),
    SHELL_CMD(snapshot, NULL, "Print telemetry snapshot", cmd_telem_snapshot),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(telem, &sub_telem, "Telemetry commands", NULL);

/* --------------------------
   Init function to populate mock data
   -------------------------- */
static int telemetry_init(const struct device *dev) {
    ARG_UNUSED(dev);
    populate_mock_data();
    return 0;
}

SYS_INIT(telemetry_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

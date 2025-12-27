//
// Created by laura on 12/10/25.
//

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "fjalar.h"
#include "com_can.h"
//#include "filter.h"   // defines struct filter_output_msg
#include "flight_state.h"

LOG_MODULE_REGISTER(test_can, LOG_LEVEL_INF);

// --- Mock filter output queue and struct ---
struct filter_output_msg {
    float position[3]; // x, y, z
    float v_norm;      // velocity magnitude
};

// Define the message queue
K_MSGQ_DEFINE(filter_output_msgq, sizeof(struct filter_output_msg), 10, 4);

// Prefill the queue with dummy data for testing
static void fill_test_filter_data(void) {
    struct filter_output_msg msg = {
        .position = {100.0f, 0.0f, 50.0f},
        .v_norm = 12.3f
    };
    k_msgq_put(&filter_output_msgq, &msg, K_NO_WAIT);
}


// ---- Mock filter output struct and queue (contained in this file) ----
struct flight_state_output_msg {
    uint8_t flight_state;
    uint8_t velocity_class;
    bool event_launch;
    bool event_burnout;
    bool event_apogee;
    bool event_drogue_deployed;
    bool event_main_deployed;
    bool event_landed;
};

// Mock queue
K_MSGQ_DEFINE(flight_state_output_msgq, sizeof(struct flight_state_output_msg), 10, 4);

// Optional: prefill with dummy data
static void fill_test_state_data(void) {
    struct flight_state_output_msg msg = {
        .flight_state = 1,
        .velocity_class = 2,
        .event_launch = true,
        .event_burnout = false,
        .event_apogee = false,
        .event_drogue_deployed = false,
        .event_main_deployed = false,
        .event_landed = false
    };
    k_msgq_put(&flight_state_output_msgq, &msg, K_NO_WAIT);
}


extern fjalar_t fjalar_god;
extern const struct device *const can_dev;


// CAN message IDs (adjust to match your protocol)
#define CAN_ID_STATE_UPDATE    0x100
#define CAN_ID_TELEMETRY       0x101
#define CAN_ID_AIRBRAKE_CMD    0x102
#define CAN_ID_EVENT           0x103

// Send state update message
static int cmd_can_send_state(const struct shell *sh, size_t argc, char **argv) {
    struct flight_state_output_msg state_msg;

    if (k_msgq_peek(&flight_state_output_msgq, &state_msg) != 0) {
        shell_error(sh, "No state data available");
        return -1;
    }

    struct can_frame frame = {
        .id = CAN_ID_STATE_UPDATE,
        .dlc = 8,
        .data = {
            state_msg.flight_state,
            state_msg.velocity_class,
            (state_msg.event_launch ? 0x01 : 0) |
            (state_msg.event_burnout ? 0x02 : 0) |
            (state_msg.event_apogee ? 0x04 : 0) |
            (state_msg.event_drogue_deployed ? 0x08 : 0) |
            (state_msg.event_main_deployed ? 0x10 : 0) |
            (state_msg.event_landed ? 0x20 : 0),
            0, 0, 0, 0, 0
        }
    };

    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret == 0) {
        shell_print(sh, "✓ CAN state update sent:");
        shell_print(sh, "  ID: 0x%03X", frame.id);
        shell_print(sh, "  State: %d", frame.data[0]);
        shell_print(sh, "  Events: 0x%02X", frame.data[2]);
    } else {
        shell_error(sh, "✗ CAN send failed: %d", ret);
    }

    return ret;
}

// Send telemetry message
static int cmd_can_send_telemetry(const struct shell *sh, size_t argc, char **argv) {
    struct filter_output_msg filter_msg;

    if (k_msgq_peek(&filter_output_msgq, &filter_msg) != 0) {
        shell_error(sh, "No filter data available");
        return -1;
    }

    // Pack altitude (as int16 in meters)
    int16_t altitude = (int16_t)filter_msg.position[2];

    // Pack velocity (as int16 in m/s * 10)
    int16_t velocity = (int16_t)(filter_msg.v_norm * 10);

    struct can_frame frame = {
        .id = CAN_ID_TELEMETRY,
        .dlc = 8,
        .data = {
            (altitude >> 8) & 0xFF,
            altitude & 0xFF,
            (velocity >> 8) & 0xFF,
            velocity & 0xFF,
            0, 0, 0, 0
        }
    };

    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret == 0) {
        shell_print(sh, "✓ CAN telemetry sent:");
        shell_print(sh, "  Altitude: %.1f m", filter_msg.position[2]);
        shell_print(sh, "  Velocity: %.1f m/s", filter_msg.v_norm);
    } else {
        shell_error(sh, "✗ CAN send failed: %d", ret);
    }

    return ret;
}

// Monitor CAN bus
static int cmd_can_monitor(const struct shell *sh, size_t argc, char **argv) {
    shell_print(sh, "Monitoring CAN bus (Ctrl+C to stop)...");
    shell_print(sh, "Waiting for messages...\n");

    struct can_frame frame;
    int count = 0;

    for (int i = 0; i < 10; i++) {  // Monitor for 10 messages or 10 seconds
        int ret = can_recv(can_dev, &frame, K_SECONDS(1));
        if (ret == 0) {
            count++;
            shell_print(sh, "RX: ID=0x%03X DLC=%d Data=", frame.id, frame.dlc);
            for (int j = 0; j < frame.dlc; j++) {
                shell_fprintf(sh, SHELL_NORMAL, "%02X ", frame.data[j]);
            }
            shell_fprintf(sh, SHELL_NORMAL, "\n");
        }
    }

    shell_print(sh, "\nReceived %d messages", count);
    return 0;
}

// Send airbrake command (to Loki)
static int cmd_can_airbrake(const struct shell *sh, size_t argc, char **argv) {
    if (argc < 2) {
        shell_error(sh, "Usage: can airbrake <0-100>");
        return -1;
    }

    uint8_t position = atoi(argv[1]);
    if (position > 100) {
        shell_error(sh, "Position must be 0-100");
        return -1;
    }

    struct can_frame frame = {
        .id = CAN_ID_AIRBRAKE_CMD,
        .dlc = 1,
        .data = {position}
    };

    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret == 0) {
        shell_print(sh, "✓ Airbrake command sent: %d%%", position);
    } else {
        shell_error(sh, "✗ CAN send failed: %d", ret);
    }

    return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_can_test,
    SHELL_CMD(state, NULL, "Send state update", cmd_can_send_state),
    SHELL_CMD(telem, NULL, "Send telemetry", cmd_can_send_telemetry),
    SHELL_CMD(monitor, NULL, "Monitor CAN bus", cmd_can_monitor),
    SHELL_CMD(airbrake, NULL, "Send airbrake command", cmd_can_airbrake),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(can, &sub_can_test, "CAN bus testing", NULL);
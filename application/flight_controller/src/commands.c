/*
This is the commands script; its purpose is to:
1) Read bytes from the communication channel, assemble them into Fjalar protocol messages.
2) Dispatch each complete message to the appropriate handler based on its tag.
3) Enforce permissions (sudo mode) for critical operations.
4) Update the system state (flight mode, sudo flag), trigger hardware actions (pyros), 
   and manage flash memory (clear/read).
5) Send acknowledgements or data responses back over the same channel.

It sits between the low-level communication module and the higher-level flight logic,
translating incoming commands into state changes and actuator outputs.
*/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <protocol.h>

#include "commands.h"
#include "communication.h"
#include "fjalar.h"
#include "actuation.h"
#include "flight_state.h"
#include "hilsensor.h"

LOG_MODULE_REGISTER(commands, CONFIG_APP_COMMANDS_LOG_LEVEL);

void handle_set_sudo(set_sudo_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_ready_up(ready_up_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel);
void handle_enter_idle(enter_idle_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel);
void handle_trigger_pyro(trigger_pyro_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_clear_flash(clear_flash_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_read_flash(read_flash_t *msg, fjalar_t *fjalar, enum com_channels channel);
void handle_hil_in(hil_in_t *msg, fjalar_t *fjalar, enum com_channels channel);

void handle_fjalar_buf(struct protocol_state *ps, fjalar_t *fjalar, uint8_t *buf, int len, enum com_channels channel) {
    fjalar_message_t msg;
    for (int i = 0; i < len; i++) {
        int ret;
        ret = parse_fjalar_message(ps, buf[i], &msg);
        if (ret == 1) {
            handle_fjalar_message(&msg, fjalar, fjalar->ptr_state, channel);
        } else if (ret == -1) {
            reset_protocol_state(ps);
        }
    }
}

void handle_fjalar_message(fjalar_message_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel) {
    //LOG_INF("handling msg with id %d", msg->data.which_data);
    switch (msg->data.which_data) {
        case FJALAR_DATA_SET_SUDO_TAG:
            handle_set_sudo(&msg->data.data.set_sudo, fjalar, channel);
            break;

        case FJALAR_DATA_READY_UP_TAG:
            handle_ready_up(&msg->data.data.ready_up, fjalar, state, channel);
            break;

        case FJALAR_DATA_ENTER_IDLE_TAG:
            handle_enter_idle(&msg->data.data.enter_idle, fjalar, fjalar->ptr_state, channel);
            break;

        case FJALAR_DATA_TRIGGER_PYRO_TAG:
            handle_trigger_pyro(&msg->data.data.trigger_pyro, fjalar, channel);
            break;

        case FJALAR_DATA_CLEAR_FLASH_TAG:
            handle_clear_flash(&msg->data.data.clear_flash, fjalar, channel);
            break;

        case FJALAR_DATA_READ_FLASH_TAG:
            handle_read_flash(&msg->data.data.read_flash, fjalar, channel);
            break;
            
        case FJALAR_DATA_HIL_IN_TAG:
            handle_hil_in(&msg->data.data.hil_in, fjalar, channel);
            break;
        default:
            LOG_ERR("Unsupported message: %d", msg->data.which_data);
    }
}

void handle_set_sudo(set_sudo_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    fjalar->sudo = msg->enabled;
    LOG_WRN("set sudo %d", msg->enabled);
}

// this function no longer serves purpose...
void handle_ready_up(ready_up_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel) {
    bool succesful;
    LOG_INF("handle_ready_up RUN!, this function does nothing, fix.");
    /*
    if (fjalar->sudo == true || state->flight_state == STATE_IDLE) {
        state->flight_state = STATE_LAUNCHPAD;
        succesful = true;
        LOG_INF("changing to ready");
    } else {
        succesful = false;
        LOG_INF("couldn't change to ready");
    }
    */
}

void handle_enter_idle(enter_idle_t *msg, fjalar_t *fjalar, state_t *state, enum com_channels channel) {
    bool succesful;
    if (fjalar->sudo == true || state->flight_state == STATE_LAUNCHPAD) {
        state->flight_state = STATE_IDLE;
        succesful = true;
        LOG_INF("changing to idle");
    } else {
        succesful = false;
        LOG_INF("couldn't change to idle");
    }
}

void handle_trigger_pyro(trigger_pyro_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    if (fjalar->sudo != true) {
        LOG_WRN("Tried to enable pyro %d when not in sudo mode", msg->pyro);
        return;
    }
        set_pyro(fjalar, msg->pyro, 1);
        k_msleep(500);
        set_pyro(fjalar, msg->pyro, 0);
}

void handle_clear_flash(clear_flash_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    if (fjalar->sudo != true) {
        LOG_WRN("Tried to clear flash when not in sudo mode");
        return;
    }
    clear_flash(fjalar);
}

void handle_read_flash(read_flash_t *msg, fjalar_t *fjalar, enum com_channels channel) {
    struct fjalar_message resp;
    if (msg->length > sizeof(resp.data.data.flash_data.data.bytes)) {
        LOG_ERR("Requested flash read can't fit in buffer");
        return;
    }
    resp.has_data = true;
    resp.data.which_data = FJALAR_DATA_FLASH_DATA_TAG;
    resp.data.data.flash_data.data.size = msg->length;
    resp.data.data.flash_data.start_index = msg->start_index;
    resp.time = k_uptime_get_32();
    read_flash(fjalar, resp.data.data.flash_data.data.bytes, msg->start_index, msg->length);
    send_response(fjalar, &resp, channel);
}

void handle_hil_in(hil_in_t *msg, fjalar_t *fjalar, enum com_channels channel){
    #if DT_NODE_EXISTS(DT_ALIAS(hilsensor))
    const struct device *const hilsensor_dev = DEVICE_DT_GET(DT_ALIAS(hilsensor));

    hil_data_t hil_data = {
        .ax = msg->ax,
        .ay = msg->ay,
        .az = msg->az,
        .gx = msg->gx,
        .gy = msg->gy,
        .gz = msg->gz,
        .p = msg->p,
        .lon = msg->lon,
        .lat = msg->lat,
        .alt = msg->alt,
        .time = k_uptime_get_32(),
    };

    hilsensor_feed(hilsensor_dev, &hil_data);
    #endif
}
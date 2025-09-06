#pragma once

#include "protocol.h"

#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

enum screen_frames {
    FRAME_INFO,
    FRAME_TELEMETRY,
    FRAME_TRACKING,
    FRAME_ENTER_INIT,
    FRAME_GET_READY,
    FRAME_MAX
};

typedef struct {
    volatile float lat;
    volatile float lon;
    volatile float alt;
    volatile flight_state_t state;
} rocket_data_t;


typedef struct {
    volatile enum screen_frames current_frame;
    // volatile struct telemetry_packet telemetry;
    volatile int32_t local_rssi;
    volatile float latitude;
    volatile float longitude;
    volatile float battery_voltage;
    volatile rocket_data_t rocket;
} tracker_t;

extern tracker_t tracker_god;

#pragma once
#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;
typedef struct can can_t;




typedef struct fjalar {
    float altitude; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float ground_level; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float velocity; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float ax; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float ay; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float az; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool drogue_deployed; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool main_deployed; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    uint32_t liftoff_at; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    uint32_t apogee_at; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool sudo; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    uint32_t flash_address; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    uint32_t flash_size; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float battery_voltage; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float latitude; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    float longitude; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool pyro1_sense; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool pyro2_sense; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    bool pyro3_sense; // make update_fjalar function that updates these (to be sent with telemetry using communications.c)
    
    init_t            *ptr_init;
    position_filter_t *ptr_pos_kf;
    attitude_filter_t *ptr_att_kf;
    aerodynamics_t    *ptr_aerodynamics;
    state_t           *ptr_state;
    can_t             *ptr_can;
} fjalar_t;


extern fjalar_t fjalar_god;

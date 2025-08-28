#pragma once
#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;
typedef struct control control_t;
typedef struct can can_t;
typedef struct lora lora_t;


typedef struct fjalar { // used in [comes from]
    bool drogue_deployed;  // HIL, [fjalar]
    bool main_deployed; // HIL [fjalar]
    uint32_t liftoff_at; // state machine [state]
    uint32_t apogee_at; // state machine [state]
    bool sudo;
    uint32_t flash_address;
    uint32_t flash_size;
    float battery_voltage;
    bool pyro1_sense;
    bool pyro2_sense;
    bool pyro3_sense;

    init_t            *ptr_init;
    position_filter_t *ptr_pos_kf;
    attitude_filter_t *ptr_att_kf;
    aerodynamics_t    *ptr_aerodynamics;
    state_t           *ptr_state;
    control_t         *ptr_control;
    can_t             *ptr_can;
    lora_t            *ptr_lora;
} fjalar_t;


extern fjalar_t fjalar_god;

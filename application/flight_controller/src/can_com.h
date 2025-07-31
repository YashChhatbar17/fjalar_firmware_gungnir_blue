#include "fjalar.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct position_filter position_filter_t;
typedef struct attitude_filter attitude_filter_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;

typedef struct can {

} can_t;

typedef struct {
    uint8_t  loki_state;
    uint8_t  loki_sub_state;
    float    loki_angle;
    float    battery_voltage;
} loki_context_t;

extern const struct device *const can_dev;

void init_can(fjalar_t *fjalar);

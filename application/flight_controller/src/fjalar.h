#pragma once
#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

enum fjalar_flight_state {
    STATE_IDLE,
    STATE_LAUNCHPAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_FREE_FALL,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
};

enum fjalar_flight_event {
    EVENT_LAUNCH,
    EVENT_BURNOUT,
    EVENT_APOGEE,
    EVENT_PRIMARY_DEPLOY,
    EVENT_SECONDARY_DEPLOY,
    EVENT_LANDED
};

typedef struct {
    enum fjalar_flight_state flight_state;
    float altitude;
    float ground_level;
    float velocity;
    float ax;
    float ay;
    float az;
    bool drogue_deployed;
    bool main_deployed;
    uint32_t liftoff_at;
    uint32_t apogee_at;
    bool sudo;
    uint32_t flash_address;
    uint32_t flash_size;
    float battery_voltage;
    float latitude;
    float longitude;
    bool pyro1_sense;
    bool pyro2_sense;
    bool pyro3_sense;

    int new_x_index;
    int new_y_index;
    int new_z_index;
    int new_x_sign;
    int new_y_sign;
    int new_z_sign;
} fjalar_t;

#define IMU_INIT_N 3
#define BARO_INIT_N 3
#define GPS_INIT_N 0

typedef struct {
    // arrays
    float ax[IMU_INIT_N], ay[IMU_INIT_N], az[IMU_INIT_N], gx[IMU_INIT_N], gy[IMU_INIT_N], gz[IMU_INIT_N];
    float p[BARO_INIT_N];
    float lat[GPS_INIT_N], lon[GPS_INIT_N], alt[GPS_INIT_N];
    // mean and variances
    double mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, mean_p, mean_lon, mean_lat, mean_alt;
    double var_ax, var_ay, var_az, var_gx, var_gy, var_gz, var_p, var_lon, var_lat, var_alt;

    uint16_t n_imu, n_baro, n_gps;
    bool position_init;
} init_t;

extern fjalar_t fjalar_god;

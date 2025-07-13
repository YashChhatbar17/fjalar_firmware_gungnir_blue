#pragma once

#include <zsl/matrices.h>
#include "fjalar.h"

typedef struct fjalar fjalar_t;
typedef struct init init_t;
typedef struct aerodynamics aerodynamics_t;
typedef struct state state_t;

typedef struct position_filter {
    zsl_real_t P_data[81];
    struct zsl_mtx P;
    zsl_real_t X_data[9];
    struct zsl_mtx X;
    zsl_real_t Q_data[81];
    struct zsl_mtx Q;
    zsl_real_t R_data[1];
    struct zsl_mtx R;
    zsl_real_t T_data[9];
    struct zsl_mtx T;
    zsl_real_t Ptwosigma_data[9];
    struct zsl_mtx Ptwosigma;


    float a_norm;
    float v_norm;

    uint32_t previous_update_accelerometer;
    bool seeded;
} position_filter_t;

typedef struct attitude_filter {
    zsl_real_t P_data[9];
    struct zsl_mtx P;
    zsl_real_t X_data[9];
    struct zsl_mtx X;
    zsl_real_t Q_data[9];
    struct zsl_mtx Q;
    zsl_real_t R_data[9];
    struct zsl_mtx R;

    float phi;
    float theta;
    float psi;

    uint32_t previous_update_gyroscope;
    bool seeded;
} attitude_filter_t;

void init_filter(fjalar_t *fjalar);


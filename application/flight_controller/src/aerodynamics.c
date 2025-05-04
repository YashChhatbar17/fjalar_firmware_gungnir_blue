#include <zephyr/logging/log.h>
#include <math.h>
#include "aerodynamics.h"
#include "filter.h"

LOG_MODULE_REGISTER(aerodynamics, CONFIG_APP_AERODYNAMICS_LOG_LEVEL);

float cb_update(position_filter_t *pos_kf, float v_xy, float v_z, float z){
    float v = sqrt((v_xy*v_xy)+(v_z*v_z));

    // drag coefficient
    float c_d;
    if (v>5){
        c_d = cd_at(v);
    } else{
        c_d = 0.44;
    }

    float a = (air_density_at(z)*c_d*AREA*v*v) / (2*MASS_DRY);
    float c_b = air_density_at(z)*v*v / (2*a);
    return c_b;
}

void update_apogee_estimate(position_filter_t *pos_kf){
    float dt   = 0.01;
    float a_z0  = pos_kf->X_data[8];

    float x    = pos_kf->X_data[0];
    float y    = pos_kf->X_data[1];
    float z    = pos_kf->X_data[2];
    float xy   = sqrtf(x*x + y*y);

    float v_x  = pos_kf->X_data[3];
    float v_y  = pos_kf->X_data[4];
    float v_z  = pos_kf->X_data[5];
    float v_xy = sqrtf(v_x*v_x + v_y*v_y);
    float v_xyz = sqrtf(v_xy*v_xy + v_z*v_z);

    float a_xy;
    float a_z;

    //LOG_INF("xy: %f", xy);
    //LOG_INF("v_xy: %f", v_xy);

    LOG_INF("z: %f", z);
    //LOG_INF("v_z: %f", v_z); // ERROR: Kolla så att accel, velocity och position är korrekt under data.


    if (a_z0<0 && v_z>0 && z>100){
        while (v_z>0){
            //float a_z  = -GRAVITY -((air_density_at(z) * cd_at(v_xyz) * AREA) / (2 * MASS_DRY)) * v_z * sqrtf(v_xy*v_xy + v_z*v_z);
            //float a_xy = -((air_density_at(z) * cd_at(v_xyz) * AREA) / (2 * MASS_DRY)) * v_xy * sqrtf(v_xy*v_xy + v_z*v_z);

            float c_b = cb_update(pos_kf, v_xy, v_z, z);


            a_xy  = 0.0f;
            a_z   = -GRAVITY-(air_density_at(z)*(v_z*v_z))/(2*c_b);
            v_xy += dt * a_xy;
            v_z  += dt * a_z;
            xy   += dt * v_xy;
            z    += dt * v_z;
        }
        float apogee = z;
        LOG_INF("apogee estimate: %f", apogee);
        pos_kf->expected_apogee = apogee;
    }
}

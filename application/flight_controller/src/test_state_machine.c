//
// Created by laura on 12/1/25.
//

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "fjalar.h"
#include "filter.h"
#include "aerodynamics.h"
#include "flight_state.h"
#include "com_lora.h"
#include "init.h"

LOG_MODULE_REGISTER(test_sm, LOG_LEVEL_INF);

extern fjalar_t fjalar_god;

/* ---------------------------------------------------- */
/* Helper: inject simulated sensor data directly        */
/* ---------------------------------------------------- */
static void inject_sensor_data(const struct shell *sh)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;
    aerodynamics_t    *a  = fjalar_god.ptr_aerodynamics;

    //shell_print(sh, "Injecting sensor: raw_imu_az=%.1f, X_data[5]=%.1f", pf->raw_imu_az, pf->X_data[5]);
    /* a_norm and v_norm are directly used by state machine */
    pf->a_norm = fabsf(pf->raw_imu_az);
    pf->v_norm = fabsf(pf->X_data[5]);   /* assuming Z velocity in state */
    //shell_print(sh, "We got: a_norm=%.1f, v_norm=%.1f", pf->a_norm, pf->v_norm);

    a->drag_norm = pf->a_norm - a->g_physics;
}

/* ---------------------------------------------------- */
/* Shell: get current flight state                      */
/* ---------------------------------------------------- */
static int cmd_state_get(const struct shell *sh, size_t argc, char **argv)
{
    state_t *s = fjalar_god.ptr_state;

    const char *state_names[] = {
        "STATE_IDLE",
        "STATE_AWAITING_INIT",
        "STATE_INITIATED",
        "STATE_AWAITING_LAUNCH",
        "STATE_BOOST",
        "STATE_COAST",
        "STATE_DROGUE_DESCENT",
        "STATE_MAIN_DESCENT",
        "STATE_LANDED"
    };

    shell_print(sh, "Current State: %s (%d)",
                state_names[s->flight_state],
                s->flight_state);

    shell_print(sh, "Events:");
    shell_print(sh, "  launch: %d", s->event_launch);
    shell_print(sh, "  burnout: %d", s->event_burnout);
    shell_print(sh, "  above_acs: %d", s->event_above_acs_threshold);
    shell_print(sh, "  apogee: %d", s->event_apogee);
    shell_print(sh, "  drogue: %d", s->event_drogue_deployed);
    shell_print(sh, "  main: %d", s->event_main_deployed);
    shell_print(sh, "  landed: %d", s->event_landed);

    return 0;
}

/* ---------------------------------------------------- */
/* LORA commands                                        */
/* ---------------------------------------------------- */
static int cmd_lora_init(const struct shell *sh, size_t argc, char **argv)
{
    fjalar_god.ptr_lora->LORA_READY_INITIATE_FJALAR = true;
    shell_print(sh, "LoRa INIT sent");
    return 0;
}

static int cmd_lora_launch(const struct shell *sh, size_t argc, char **argv)
{
    fjalar_god.ptr_lora->LORA_READY_LAUNCH_FJALAR = true;
    shell_print(sh, "LoRa LAUNCH sent");
    return 0;
}

/* ---------------------------------------------------- */
/* INIT completion                                      */
/* ---------------------------------------------------- */
static int cmd_complete_init(const struct shell *sh, size_t argc, char **argv)
{
    fjalar_god.ptr_init->init_completed = true;
    shell_print(sh, "Init completed");
    return 0;
}

/* ---------------------------------------------------- */
/* SIMULATION COMMANDS                                  */
/* ---------------------------------------------------- */
static int cmd_sim_boost(const struct shell *sh, size_t argc, char **argv)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;
    aerodynamics_t    *a  = fjalar_god.ptr_aerodynamics;

    //while (pf->a_norm < 15.0f || pf->X_data[2] < 3) { // this is the velocity powered one, since it's const v
    //    pf->X_data[2] = 10.0f;   // raw Z velocity
    //    pf->a_norm = 30.0f;
    //    a->thrust_bool = true;
   //     inject_sensor_data(sh);
    //    k_msleep(100);             // allow filter to update
    //}

	for(int i=0; i < 150; i++) { // this is the velocity powered one, since it's const v
		pf->X_data[5] = 50.0f;
		pf->v_norm = pf->v_norm + pf->X_data[5];
        a->thrust_bool = true;
        inject_sensor_data(sh);
        k_msleep(100);             // allow filter to update
    }

    shell_print(sh, "BOOST simulated: a=%.1f, v=%.1f, z=%.1f",
                pf->a_norm, pf->v_norm, pf->X_data[2]);
    return 0;
}

static int cmd_sim_coast(const struct shell *sh, size_t argc, char **argv)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;
    aerodynamics_t    *a  = fjalar_god.ptr_aerodynamics;
    a->thrust_bool = false;

    while (pf->a_norm >= -2.0 || pf->X_data[8] >=-2.0 || a->thrust_bool) {
        pf->X_data[8] = -5.0f;   // raw Z velocity
		pf->a_norm = pf->a_norm + pf->X_data[8];
		pf->X_data[5] = 1.0f;
		pf->v_norm = pf->v_norm + pf->X_data[5];
		a->thrust_bool = false;
        inject_sensor_data(sh);
        k_msleep(100);             // allow filter to update
    }

	shell_print(sh, "COAST simulated: az=%.1f, thrust=%s", pf->X_data[8], a->thrust_bool ? "true" : "false");
    return 0;
}

static int cmd_sim_apogee(const struct shell *sh, size_t argc, char **argv)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;
    aerodynamics_t    *a  = fjalar_god.ptr_aerodynamics;

    while (pf->X_data[5] > 0.0 || pf->X_data[2] > 200 ) {
        pf->raw_imu_az = -9.81f;      // vertical acceleration
        pf->X_data[5]  = -30.0f;      // vertical velocity
        pf->X_data[2]  = 500.0f;      // altitude
        a->thrust_bool = 0;            // no thrust during drogue
        inject_sensor_data(sh);
        k_msleep(100);
        shell_print(sh, "Simulated drogue descent: vz=%.1f m/s, z=%.1f m",
            pf->X_data[5], pf->X_data[2]);
    }

    shell_print(sh, "APOGEE simulated");
    return 0;
}

static int cmd_sim_main(const struct shell *sh, size_t argc, char **argv)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;

    pf->X_data[2] = 180.0f;      /* altitude */
    pf->X_data[5] = -25.0f;

    shell_print(sh, "MAIN deployment altitude reached");
    return 0;
}

static int cmd_sim_landed(const struct shell *sh, size_t argc, char **argv)
{
    position_filter_t *pf = fjalar_god.ptr_pos_kf;
    // a_norm > init->g_accelerometer-2 && a_norm < init->g_accelerometer+2
    for (int i = 0; i < 100; i++) { // this is the velocity powered one, since it's const v
        pf->raw_imu_az = fjalar_god.ptr_init->g_accelerometer;
        pf->X_data[5]  = fjalar_god.ptr_init->g_accelerometer +10;
        pf->X_data[2]  = fjalar_god.ptr_init->g_accelerometer - 10;
        inject_sensor_data(sh);
        k_msleep(100);             // allow filter to update
    }


    inject_sensor_data(sh);

    shell_print(sh, "LANDED simulated");
    return 0;
}

// Shell command: Run full sequence
static int cmd_run_sequence(const struct shell *sh, size_t argc, char **argv) {
    shell_print(sh, "\n=== Running Full State Machine Sequence ===\n");

    // STATE_IDLE → STATE_AWAITING_INIT
    shell_print(sh, "Step 1: STATE_IDLE → STATE_AWAITING_INIT");
    cmd_lora_init(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_AWAITING_INIT → STATE_INITIATED
    shell_print(sh, "\nStep 2: STATE_AWAITING_INIT → STATE_INITIATED");
    cmd_complete_init(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_INITIATED → STATE_AWAITING_LAUNCH
    shell_print(sh, "\nStep 3: STATE_INITIATED → STATE_AWAITING_LAUNCH");
    cmd_lora_launch(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_AWAITING_LAUNCH → STATE_BOOST
    shell_print(sh, "\nStep 4: STATE_AWAITING_LAUNCH → STATE_BOOST");
    cmd_sim_boost(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_BOOST → STATE_COAST
    shell_print(sh, "\nStep 5: STATE_BOOST → STATE_COAST");
    cmd_sim_coast(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_COAST → STATE_DROGUE_DESCENT
    shell_print(sh, "\nStep 6: STATE_COAST → STATE_DROGUE_DESCENT");
    cmd_sim_apogee(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_DROGUE_DESCENT → STATE_MAIN_DESCENT
    shell_print(sh, "\nStep 7: STATE_DROGUE_DESCENT → STATE_MAIN_DESCENT");
    cmd_sim_main(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);
    k_msleep(1000);

    // STATE_MAIN_DESCENT → STATE_LANDED
    shell_print(sh, "\nStep 8: STATE_MAIN_DESCENT → STATE_LANDED");
    cmd_sim_landed(sh, 0, NULL);
    k_msleep(100);
    cmd_state_get(sh, 0, NULL);

    shell_print(sh, "\n=== Sequence Complete ===\n");
    return 0;
}

// Register shell commands
SHELL_STATIC_SUBCMD_SET_CREATE(sub_state,
    SHELL_CMD(get, NULL, "Get current state", cmd_state_get),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lora,
    SHELL_CMD(init, NULL, "Send LoRa INIT command", cmd_lora_init),
    SHELL_CMD(launch, NULL, "Send LoRa LAUNCH command", cmd_lora_launch),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_sim,
    SHELL_CMD(boost, NULL, "Simulate boost phase", cmd_sim_boost),
    SHELL_CMD(coast, NULL, "Simulate coast phase", cmd_sim_coast),
    SHELL_CMD(apogee, NULL, "Simulate apogee", cmd_sim_apogee),
    SHELL_CMD(main, NULL, "Simulate main deployment", cmd_sim_main),
    SHELL_CMD(landed, NULL, "Simulate landing", cmd_sim_landed),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_test,
    SHELL_CMD(state, &sub_state, "State commands", NULL),
    SHELL_CMD(lora, &sub_lora, "LoRa commands", NULL),
    SHELL_CMD(sim, &sub_sim, "Simulation commands", NULL),
    SHELL_CMD(sequence, NULL, "Run full sequence", cmd_run_sequence),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(test, &sub_test, "State machine testing", NULL);

//Change drogue state to reefed state. Deploy the main (the only parachute)
//at some specific altitude, e.g., 500 m.  
//Replace DROGUE_DESCENT with REEFED_DESCENT and switch the order of main and reefed. 
   /* ------------------------------ STATE_COAST -------------------------- */
    case STATE_COAST:
        /*
        Detect apogee: upward velocity transitions to negative.
        This is the most reliable trigger for drogue deployment.
        */
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            deploy_main(fjalar, init, state, pos_kf);
            state->flight_state = STATE_MAIN_DESCENT;
            state->event_apogee = true;

            LOG_WRN("State: COAST → MAIN_DESCENT (apogee detected)");
        }
        break;

    /* --------------------------- STATE_DROGUE_DESCENT -------------------- */
    case STATE_MAIN_DESCENT:
        /*
        Deploy main parachute at a safe altitude threshold.
        */
        if (z < 500) {
            deploy_reefing(fjalar, init, state, pos_kf);
            state->flight_state = STATE_REEFED_DESCENT;
            LOG_WRN("State: MAIN_DESCENT → REEFED_DESCENT (500 m AGL)");
        }
        break;

    /* --------------------------- STATE_MAIN_DESCENT ---------------------- */
    case STATE_REEFED_DESCENT:
        /*
        Detect landing by checking if acceleration ~ 1 g and movement is minimal.
        This is approximate but usually reliable with filtered acceleration.
        */
        if (a_norm > init->g_accelerometer - 2 &&
            a_norm < init->g_accelerometer + 2)
        {
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State: REEFED_DESCENT → LANDED (acceleration stable ~1g)");
        }
        break;




    static void deploy_main(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) {
    set_pyro(fjalar, 2, true);  // 2 = main chute pyro channel
    LOG_WRN("Constricted parachute deployed at %.2f m", pos_kf->X_data[2]);

    //Must change the deploy_drogue function to deploy_reefing
    static void deploy_reefing(fjalar_t *fjalar, init_t *init, state_t *state, position_filter_t *pos_kf) { 
    set_pyro(fjalar, 1, true);  // 1 = drogue chute pyro channel
    LOG_WRN("Full parachute deployed at %.2f m %.2f s",
            pos_kf->X_data[2],                                  // altitude
            (state->apogee_time - state->liftoff_time) / 1000.0f);
}












//Option 1: Deploy main at apogee. This requires STATE_DROGUE_DESCENT to be remove in flight_state.h
/* ------------------------------ STATE_COAST -------------------------- */
    case STATE_COAST:
        /*
        Detect apogee: upward velocity transitions to negative.
        This is the most reliable trigger for drogue deployment.
        */
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            deploy_main(fjalar, init, state, pos_kf);
            state->flight_state = STATE_MAIN_DESCENT;
            state->event_apogee = true;

            LOG_WRN("State: COAST → MAIN_DESCENT (apogee detected)");
        }
        break;

    /* --------------------------- STATE_MAIN_DESCENT ---------------------- */
    case STATE_MAIN_DESCENT:
        /*
        Detect landing by checking if acceleration ~ 1 g and movement is minimal.
        This is approximate but usually reliable with filtered acceleration.
        */
        if (a_norm > init->g_accelerometer - 2 &&
            a_norm < init->g_accelerometer + 2)
        {
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State: MAIN_DESCENT → LANDED (acceleration stable ~1g)");
        }
        break;

//Option 2: Keep drogue state. Just remove parachute deployment. Deploy the main (the only parachute)
//at some specific altitude, e.g., 500 m.  
   /* ------------------------------ STATE_COAST -------------------------- */
    case STATE_COAST:
        /*
        Detect apogee: upward velocity transitions to negative.
        This is the most reliable trigger for drogue deployment.
        */
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            state->flight_state = STATE_DROGUE_DESCENT;
            state->event_apogee = true;

            LOG_WRN("State: COAST → DROGUE_DESCENT (apogee detected)");
        }
        break;

    /* --------------------------- STATE_DROGUE_DESCENT -------------------- */
    case STATE_DROGUE_DESCENT:
        /*
        Deploy main parachute at a safe altitude threshold.
        */
        if (z < 500) {
            deploy_main(fjalar, init, state, pos_kf);
            state->flight_state = STATE_MAIN_DESCENT;
            LOG_WRN("State: DROGUE_DESCENT → MAIN_DESCENT (500 m AGL)");
        }
        break;

    /* --------------------------- STATE_MAIN_DESCENT ---------------------- */
    case STATE_MAIN_DESCENT:
        /*
        Detect landing by checking if acceleration ~ 1 g and movement is minimal.
        This is approximate but usually reliable with filtered acceleration.
        */
        if (a_norm > init->g_accelerometer - 2 &&
            a_norm < init->g_accelerometer + 2)
        {
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State: MAIN_DESCENT → LANDED (acceleration stable ~1g)");
        }
        break;

//Option 3: Remove drogue state. Deploy the main (the only parachute) at some specific altitude, e.g., 500 m.
//This requires STATE_DROGUE_DESCENT to be remove in flight_state.h. 
   /* ------------------------------ STATE_COAST -------------------------- */
    case STATE_COAST:
        /*
        Detect apogee: upward velocity transitions to negative.
        This is the most reliable trigger for drogue deployment.
        */
        if (vz < 0) {
            state->apogee_time = k_uptime_get_32();
            state->event_apogee = true;
            LOG_WRN("Apogee dected");
        }


        if (vz < 0 && z < 500) {
            deploy_main(fjalar, init, state, pos_kf);
            state->flight_state = STATE_MAIN_DESCENT;
            LOG_WRN("State: COAST → MAIN_DESCENT");
        }
        break;
    /* --------------------------- STATE_MAIN_DESCENT ---------------------- */
    case STATE_MAIN_DESCENT:
        /*
        Detect landing by checking if acceleration ~ 1 g and movement is minimal.
        This is approximate but usually reliable with filtered acceleration.
        */
        if (a_norm > init->g_accelerometer - 2 &&
            a_norm < init->g_accelerometer + 2)
        {
            state->flight_state = STATE_LANDED;
            state->event_landed = true;
            LOG_WRN("State: MAIN_DESCENT → LANDED (acceleration stable ~1g)");
        }
        break;
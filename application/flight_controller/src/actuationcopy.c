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
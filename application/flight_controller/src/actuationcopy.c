/*
================================================================================
ACTUATION MODULE - Hardware Output Control for Fjalar Flight Computer
================================================================================
This module manages all physical outputs on the rocket:
1) Pyrotechnic channels - Controls parachute deployment charges
2) LED indicators - Visual system health feedback
3) Buzzer - Audio status feedback for ground operations

SAFETY CRITICAL: This code directly controls pyrotechnic charges. Any bugs
could result in premature deployment (rocket destroyed) or failed deployment
(rocket crashes). Handle with extreme care.
================================================================================
*/

#include <zephyr/kernel.h>          // RTOS kernel functions (threads, timers)
#include <zephyr/logging/log.h>     // Logging system for debugging
#include <zephyr/drivers/gpio.h>    // GPIO control (digital pins)
#include <zephyr/drivers/pwm.h>     // PWM for buzzer tone generation
#include <math.h>                   // Math functions (abs() used in buzzer)
#include <stdlib.h>                 // Standard library functions

#include "fjalar.h"                 // Main system structure definitions
#include "melodies.h"               // Musical note data for buzzer
#include "flight_state.h"           // Flight state machine definitions

// Register this module with the logging system at INFO level
// This allows us to see important events but filters out debug spam
LOG_MODULE_REGISTER(actuation, LOG_LEVEL_INF);

/*
================================================================================
THREAD CONFIGURATION
================================================================================
Zephyr RTOS uses cooperative threads with fixed priorities (0-15, lower = higher)
All actuation threads run at priority 7 (medium priority):
- Higher than non-critical tasks
- Lower than time-critical sensor/control loops
Each thread gets 2KB of stack space (sufficient for small embedded systems)
*/

#define LED_THREAD_PRIORITY 7           // Priority for LED blinking
#define LED_THREAD_STACK_SIZE 2048      // 2KB stack for LED thread

#define BUZZER_THREAD_PRIORITY 7        // Priority for buzzer control
#define BUZZER_THREAD_STACK_SIZE 2048   // 2KB stack for buzzer thread

#define PYRO_THREAD_PRIORITY 7          // Priority for pyro monitoring
#define PYRO_THREAD_STACK_SIZE 2048     // 2KB stack for pyro thread

/*
================================================================================
LED THREAD INFRASTRUCTURE
================================================================================
The LED thread provides a simple "heartbeat" to confirm the MCU is running.
If the LED stops blinking, you know the system has crashed or hung.
*/

// Allocate stack memory for LED thread (statically allocated at compile time)
K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);

// Thread control structure (holds thread state, priority, stack pointer, etc.)
struct k_thread led_thread_data;

// Thread ID used to reference this thread in other parts of the code
k_tid_t led_thread_id;

// Forward declaration of the LED thread function (defined later)
void led_thread(fjalar_t *fjalar, void *, void *);

/*
================================================================================
BUZZER THREAD INFRASTRUCTURE
================================================================================
The buzzer plays different melodies based on rocket state, providing audio
feedback during ground operations so crew knows the system status without
needing to check telemetry or serial output.
*/

K_THREAD_STACK_DEFINE(buzzer_thread_stack, BUZZER_THREAD_STACK_SIZE);
struct k_thread buzzer_thread_data;
k_tid_t buzzer_thread_id;
void buzzer_thread(fjalar_t *fjalar, state_t *state, void *);

/*
================================================================================
PYRO THREAD INFRASTRUCTURE
================================================================================
SAFETY CRITICAL: This thread monitors pyrotechnic channel continuity.
It continuously checks if e-matches (electric igniters) are connected.
This prevents launch if a parachute isn't properly installed.
*/

K_THREAD_STACK_DEFINE(pyro_thread_stack, PYRO_THREAD_STACK_SIZE);
struct k_thread pyro_thread_data;
k_tid_t pyro_thread_id;
void pyro_thread(fjalar_t *fjalar, void *, void *);

/*
================================================================================
GLOBAL TERMINATION FLAG
================================================================================
volatile: Tells compiler this variable can change outside normal program flow
(e.g., in interrupt handlers or other threads). Prevents aggressive optimization
that might cache the value and miss updates.

Used to gracefully shut down all actuation threads during system cleanup.
*/
volatile bool terminate_actuation = false;

/*
================================================================================
FUNCTION: init_actuation
PURPOSE: Initialize and start all actuation threads
CALLED BY: main() during system startup
PARAMETERS: fjalar - pointer to main system structure (contains all subsystem data)
================================================================================
*/
void init_actuation(fjalar_t *fjalar) {
    // Clear termination flag (in case we're reinitializing after a previous run)
    terminate_actuation = false;
    
    /*
    ============================================================================
    CREATE LED THREAD
    ============================================================================
    k_thread_create() parameters:
    1. &led_thread_data - pointer to thread control structure
    2. led_thread_stack - pointer to allocated stack memory
    3. K_THREAD_STACK_SIZEOF() - size of stack in bytes
    4. (k_thread_entry_t) led_thread - function to execute as thread
    5. fjalar - first argument passed to thread function
    6. NULL - second argument (unused)
    7. NULL - third argument (unused)
    8. LED_THREAD_PRIORITY - thread priority (7 = medium)
    9. 0 - thread options (none)
    10. K_NO_WAIT - start thread immediately (don't delay)
    */
    led_thread_id = k_thread_create(
        &led_thread_data,
        led_thread_stack,
        K_THREAD_STACK_SIZEOF(led_thread_stack),
        (k_thread_entry_t) led_thread,
        fjalar, NULL, NULL,
        LED_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    
    // Set human-readable name for debugging (shows up in thread dumps)
    k_thread_name_set(led_thread_id, "led");

    /*
    ============================================================================
    CREATE PYRO THREAD
    ============================================================================
    SAFETY CRITICAL: This thread MUST start successfully or we have no
    continuity monitoring. In production code, you'd want to check the
    return value and halt if this fails.
    */
    pyro_thread_id = k_thread_create(
        &pyro_thread_data,
        pyro_thread_stack,
        K_THREAD_STACK_SIZEOF(pyro_thread_stack),
        (k_thread_entry_t) pyro_thread,
        fjalar, NULL, NULL,
        PYRO_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(pyro_thread_id, "pyro");

    /*
    ============================================================================
    CREATE BUZZER THREAD (CONDITIONAL)
    ============================================================================
    Only create buzzer thread if hardware actually has a buzzer connected.
    DT_ALIAS_EXISTS checks the device tree (hardware description) at compile time.
    This allows same code to run on hardware with or without buzzers.
    */
    #if DT_ALIAS_EXISTS(buzzer)
    buzzer_thread_id = k_thread_create(
        &buzzer_thread_data,
        buzzer_thread_stack,
        K_THREAD_STACK_SIZEOF(buzzer_thread_stack),
        (k_thread_entry_t) buzzer_thread,
        fjalar,              // Main system structure
        fjalar->ptr_state,   // Direct pointer to flight state (optimization)
        NULL,
        BUZZER_THREAD_PRIORITY, 0, K_NO_WAIT
    );
    k_thread_name_set(buzzer_thread_id, "buzzer");
    #endif
}

/*
================================================================================
FUNCTION: deinit_actuation
PURPOSE: Gracefully shut down all actuation threads
CALLED BY: System shutdown or reset routines
RETURNS: 0 on success, non-zero if any thread fails to terminate
================================================================================
This is important for ground testing - allows you to safely reset the system
without rebooting the entire flight computer.
*/
int deinit_actuation() {
    int e;  // Error accumulator
    
    // Set flag that all threads should check and exit when they see it
    terminate_actuation = true;
    
    /*
    k_thread_join() blocks until the specified thread exits (or timeout expires)
    K_MSEC(1000) = wait up to 1 second for each thread to terminate
    
    Using |= accumulates errors: if any thread fails to join, e will be non-zero
    
    NOTE: Current thread implementations have infinite loops with no
    terminate_actuation checks, so these joins will always timeout.
    This is a bug - threads should check the flag and exit gracefully.
    */
    e = k_thread_join(&led_thread_data, K_MSEC(1000));
    e |= k_thread_join(&pyro_thread_data, K_MSEC(1000));
    e |= k_thread_join(&buzzer_thread_data, K_MSEC(1000));
    
    // Return accumulated error status (0 = all threads terminated successfully)
    return e;  // BUG: Function doesn't explicitly return e
}

/*
================================================================================
FUNCTION: led_thread
PURPOSE: Blink LED at 1Hz to provide visual "heartbeat" that system is running
RUNS: Continuously in its own thread until system shutdown
PARAMETERS: fjalar - main system structure (unused here but maintains consistency)
            p2, p3 - unused parameters (required by Zephyr thread signature)
================================================================================
*/
void led_thread(fjalar_t *fjalar, void *p2, void *p3) {
    /*
    Get LED GPIO specification from device tree
    DT_ALIAS(io_led) looks up "io_led" alias in device tree
    Device tree defines which physical pin the LED is connected to
    This allows same code to work on different hardware
    */
    const struct gpio_dt_spec io_led_dt = GPIO_DT_SPEC_GET(DT_ALIAS(io_led), gpios);
    
    int ret = 0;  // Return value for error checking
    
    /*
    Configure the LED pin as output, initially ACTIVE (HIGH/ON)
    GPIO_OUTPUT_ACTIVE means the pin drives HIGH when "active"
    Some LEDs are active-low (ON when pin is LOW), configured in device tree
    */
    ret |= gpio_pin_configure_dt(&io_led_dt, GPIO_OUTPUT_ACTIVE);
    
    // Check if configuration succeeded
    if (ret) {
        LOG_ERR("Could not configure the IO-led");
        // NOTE: Thread continues anyway - should probably return here
    }

    /*
    Main LED loop - runs forever
    In production, should check terminate_actuation flag
    */
    while (true) {
        // Toggle LED state (if ON, turn OFF; if OFF, turn ON)
        gpio_pin_toggle_dt(&io_led_dt);
        
        // Sleep for 1000ms (1 second)
        // While sleeping, other threads can run (cooperative multitasking)
        k_msleep(1000);
    }
}

/*
================================================================================
FUNCTION: set_pyro
PURPOSE: Control a specific pyrotechnic channel (fire or safe)
SAFETY CRITICAL: This directly controls parachute deployment charges
PARAMETERS: 
    fjalar - main system structure (currently unused)
    pyro - which channel to control (1=drogue, 2=main, 3=aux)
    state - true = fire the charge, false = turn off (safe)
================================================================================
TYPICAL USAGE:
    set_pyro(&fjalar_god, 1, true);   // Fire drogue parachute
    k_msleep(1000);                    // Hold for 1 second (ensure ignition)
    set_pyro(&fjalar_god, 1, false);  // Turn off (e-match already burned out)

HARDWARE CIRCUIT:
    MCU GPIO (3.3V) → MOSFET Gate
                   → MOSFET Drain/Source → E-match → Ground
                   ↑
               Battery (9-12V, 1-2A)

The GPIO doesn't directly drive the e-match. It controls a MOSFET that
switches high-current battery power to the e-match.
================================================================================
*/
void set_pyro(fjalar_t *fjalar, int pyro, bool state) {
    switch (pyro) {
        case 1: // Drogue parachute (deploys at apogee)
            /*
            Get GPIO specification for pyro channel 1 from device tree
            Each case declares its own gpio_dt_spec because they're only
            used within that case scope (compiler optimization)
            */
            const struct gpio_dt_spec pyro1_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1), gpios);
            
            /*
            Set the GPIO pin to requested state
            state = true  → pin goes HIGH → MOSFET conducts → current flows through e-match
            state = false → pin goes LOW  → MOSFET off → no current
            */
            gpio_pin_set_dt(&pyro1_dt, state);
            break;

        case 2: // Main parachute (deploys at lower altitude)
            const struct gpio_dt_spec pyro2_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2), gpios);
            gpio_pin_set_dt(&pyro2_dt, state);
            break;

        case 3: // Auxiliary channel (backup/staging/airbrakes/etc.)
            const struct gpio_dt_spec pyro3_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3), gpios);
            gpio_pin_set_dt(&pyro3_dt, state);
            break;
            
        default:
            /*
            If someone calls set_pyro(&fjalar, 99, true), log error instead of
            causing undefined behavior. Important for debugging and safety.
            */
            LOG_ERR("tried to enable invalid pyro %d", pyro);
    }
    
    /*
    NOTE: There's no return value or error checking. In production code, you'd
    want to verify the GPIO operation succeeded and return success/failure.
    
    TIMING: Typical firing sequence:
    - Call set_pyro(X, true)
    - E-match heats up: ~10-50ms
    - Black powder ignites: ~50-100ms
    - Pressure builds: ~100-200ms
    - Separation occurs: ~200-300ms
    - Parachute deploys: ~300-500ms
    Total: ~0.5 seconds from GPIO HIGH to parachute out
    */
}

/*
================================================================================
FUNCTION: pyro_thread
PURPOSE: Continuously monitor pyrotechnic channel continuity
RUNS: Forever in its own thread, checking every 500ms
SAFETY CRITICAL: Pre-flight continuity checks prevent launch with missing igniters
PARAMETERS: fjalar - main system structure (stores continuity status)
            p2, p3 - unused
================================================================================
CONTINUITY CHECKING:
The hardware has a voltage divider on each pyro channel that allows the MCU
to detect if an e-match is connected:
- E-match connected: voltage divider creates specific voltage → GPIO reads HIGH
- No e-match: open circuit → GPIO reads LOW

This is essential pre-flight verification. Launching without continuity means
parachutes won't deploy (rocket destroyed, potential injury).
================================================================================
*/
void pyro_thread(fjalar_t *fjalar, void *p2, void *p3) {
    /*
    ============================================================================
    GET GPIO SPECIFICATIONS FOR ALL CHANNELS
    ============================================================================
    6 total pins: 3 for firing, 3 for sensing continuity
    */
    
    // Output pins - control MOSFETs that fire the charges
    const struct gpio_dt_spec pyro1_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1), gpios);
    const struct gpio_dt_spec pyro2_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2), gpios);
    const struct gpio_dt_spec pyro3_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3), gpios);
    
    // Input pins - read continuity status (is e-match connected?)
    const struct gpio_dt_spec pyro1_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro1_sense), gpios);
    const struct gpio_dt_spec pyro2_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro2_sense), gpios);
    const struct gpio_dt_spec pyro3_sense_dt = GPIO_DT_SPEC_GET(DT_ALIAS(pyro3_sense), gpios);

    int ret = 0;  // Accumulates error codes from GPIO operations
    
    /*
    ============================================================================
    SAFETY INITIALIZATION - CRITICAL SECTION
    ============================================================================
    TRIPLE-SAFE all pyro channels during startup:
    1. Set pins LOW explicitly
    2. Configure as OUTPUT_INACTIVE
    3. Set pins LOW again
    
    This prevents accidental deployment if:
    - GPIO has random state at power-on
    - Previous software left pins HIGH
    - Hardware glitch during initialization
    
    LAYERED SAFETY is essential for pyrotechnics. One safety measure might fail,
    but three independent measures make accidental deployment extremely unlikely.
    */
    
    // First safety layer: Force pins LOW before configuring
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);
    ret |= gpio_pin_set_dt(&pyro3_dt, 0);
    
    /*
    Second safety layer: Configure pins as OUTPUT_INACTIVE
    INACTIVE means the pin starts in "safe" state (LOW for these pins)
    
    COMMENT NOTES:
    "dangerous (ACTIVE)" - Original code had GPIO_OUTPUT_ACTIVE which would
    make pins HIGH during configuration → could fire charges during boot!
    This was changed to INACTIVE for safety.
    
    The "?" after the other two suggests uncertainty - they should definitely
    be INACTIVE for safety.
    */
    ret |= gpio_pin_configure_dt(&pyro1_dt, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&pyro2_dt, GPIO_OUTPUT_INACTIVE);
    ret |= gpio_pin_configure_dt(&pyro3_dt, GPIO_OUTPUT_INACTIVE);
    
    // Third safety layer: Force pins LOW again after configuration
    ret |= gpio_pin_set_dt(&pyro1_dt, 0);
    ret |= gpio_pin_set_dt(&pyro2_dt, 0);
    ret |= gpio_pin_set_dt(&pyro3_dt, 0);

    /*
    ============================================================================
    CONFIGURE CONTINUITY SENSE PINS
    ============================================================================
    These are input pins that read the voltage divider on each channel.
    No special initialization needed - just configure as inputs.
    */
    ret |= gpio_pin_configure_dt(&pyro1_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro2_sense_dt, GPIO_INPUT);
    ret |= gpio_pin_configure_dt(&pyro3_sense_dt, GPIO_INPUT);
    
    // Check if any GPIO operation failed
    if (ret) {
        LOG_ERR("Could not configure pyro pins");
        // NOTE: Thread continues anyway - should probably halt system here
        // Failed pyro configuration is a critical safety issue
    }

    /*
    ============================================================================
    MAIN CONTINUITY MONITORING LOOP
    ============================================================================
    Runs forever, checking continuity every 500ms
    Should check terminate_actuation flag for graceful shutdown
    */
    while (true) {
        /*
        Read continuity status from each sense pin
        gpio_pin_get_dt() returns:
        - 0 if pin is LOW (no e-match connected)
        - 1 if pin is HIGH (e-match connected)
        
        Store results in fjalar structure so other parts of system can access:
        - Flight state machine checks before allowing "ARM" state
        - Telemetry system reports to ground station
        - Pre-flight checklist requires all three = 1
        */
        fjalar->pyro1_sense = gpio_pin_get_dt(&pyro1_sense_dt);
        fjalar->pyro2_sense = gpio_pin_get_dt(&pyro2_sense_dt);
        fjalar->pyro3_sense = gpio_pin_get_dt(&pyro3_sense_dt);
        
        /*
        Log continuity status at INFO level
        Example output: "pyros connected p1:1 p2:1 p3:0"
        This means:
        - Drogue e-match connected (1)
        - Main e-match connected (1)  
        - Aux e-match NOT connected (0) ← Problem! Need to install igniter
        
        Ground crew monitors this during pre-flight to verify all parachutes
        are properly connected before allowing launch.
        */
        LOG_INF("pyros connected p1:%d p2:%d p3:%d", 
                fjalar->pyro1_sense, fjalar->pyro2_sense, fjalar->pyro3_sense);
        
        /*
        Sleep for 500ms between checks
        500ms is frequent enough to catch continuity issues quickly, but not
        so frequent that it wastes CPU time or floods the log.
        
        While sleeping, other threads continue running (cooperative multitasking).
        */
        k_msleep(500);
    }
}

/*
================================================================================
FUNCTION: play_song
PURPOSE: Play a musical melody through the buzzer
PARAMETERS:
    buzzer_dt - PWM device specification for buzzer hardware
    melody - array of [frequency, duration] pairs
    size - size of melody array in bytes
    tempo - beats per minute (controls overall playback speed)
================================================================================
HOW IT WORKS:
The buzzer is controlled via PWM (Pulse Width Modulation). By varying the
PWM frequency, we can generate different musical notes.

MELODY FORMAT:
melody[] = {NOTE_C4, 4, NOTE_G4, 8, NOTE_A4, 8, ...}
           ^^^^^^^^  ^
           frequency | duration (as fraction of whole note)
           
Duration values:
- 4 = quarter note (1/4 of whole note)
- 8 = eighth note (1/8 of whole note)
- -4 = dotted quarter note (1.5x normal quarter note)
Negative durations add 50% length for "dotted" notes in musical notation.
================================================================================
*/
void play_song(const struct pwm_dt_spec *buzzer_dt, int melody[], int size, int tempo) {
    /*
    Calculate number of notes in the melody
    sizeof(melody[0]) = size of one int (typically 4 bytes)
    Each note uses 2 ints (frequency + duration), so divide by 2
    */
    int notes = size / sizeof(melody[0]) / 2;
    
    /*
    Calculate duration of a whole note in milliseconds
    Formula: (60 seconds/minute * 1000 ms/second * 4 quarter notes) / tempo
    Example: tempo=120 BPM → wholenote = (60000 * 4) / 120 = 2000ms
    */
    int wholenote = (60000 * 4) / tempo;

    int divider = 0;       // Duration divisor (4=quarter, 8=eighth, etc.)
    int noteDuration = 0;  // Calculated duration in milliseconds

    /*
    Iterate through melody array, stepping by 2 each time:
    - melody[thisNote] = frequency
    - melody[thisNote + 1] = duration
    */
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
        // Get duration divider from melody array
        divider = melody[thisNote + 1];
        
        if (divider > 0) {
            /*
            Normal note: duration = wholenote / divider
            Example: quarter note (divider=4) with tempo=120
            → noteDuration = 2000ms / 4 = 500ms
            */
            noteDuration = (wholenote) / divider;
            
        } else if (divider < 0) {
            /*
            Dotted note (negative divider): duration is 1.5x normal
            Example: dotted quarter note (divider=-4)
            → noteDuration = 2000ms / 4 = 500ms, then * 1.5 = 750ms
            
            In musical notation, a dot adds half the note's value.
            */
            noteDuration = (wholenote) / abs(divider);
            noteDuration *= 1.5;  // Add 50% for dot
        }

        /*
        Calculate PWM period from frequency
        PWM period (nanoseconds) = 1 second / frequency
        1e9 = 1,000,000,000 nanoseconds = 1 second
        
        Example: 440 Hz (musical note A4)
        → period = 1e9 / 440 = 2,272,727 nanoseconds ≈ 2.27ms
        */
        int period = 1e9 / melody[thisNote];
        
        /*
        Set PWM to generate the tone
        period = total cycle time
        period / 2 = duty cycle (50% on, 50% off → square wave → clean tone)
        
        50% duty cycle produces the loudest, clearest tone for most buzzers
        */
        pwm_set_dt(buzzer_dt, period, period / 2);
        
        /*
        Play note for 90% of its duration
        This creates a slight gap between notes, making the melody more distinct
        Without this gap, notes would blend together and sound muddy
        */
        k_msleep(noteDuration * 0.9);
        
        /*
        Turn off buzzer (duty cycle = 0, no sound)
        This creates 10% silence between notes
        */
        pwm_set_dt(buzzer_dt, period, 0);
        
        /*
        Wait for remaining 10% of note duration (silence between notes)
        Total time per note = (90% playing) + (10% silence) = 100% = noteDuration
        */
        k_msleep(noteDuration * 0.1);
    }
    
    // 1 second pause after song finishes (before looping or next song)
    k_msleep(1000);
}

/*
================================================================================
BUZZER THREAD (CONDITIONAL COMPILATION)
================================================================================
Only compiled if device tree indicates buzzer hardware exists
This allows same code to work on hardware variants with/without buzzers
*/
#if DT_ALIAS_EXISTS(buzzer)

/*
================================================================================
FUNCTION: buzzer_thread
PURPOSE: Play audio feedback based on rocket flight state
RUNS: Continuously in its own thread
PARAMETERS: 
    fjalar - main system structure (contains sudo flag)
    state - pointer to flight state structure
    p3 - unused
================================================================================
AUDIO FEEDBACK STRATEGY:
Different flight states play different melodies, allowing ground crew to
know rocket status just by listening:
- Greensleeves = IDLE (sitting on pad, waiting)
- Keyboard Cat = INITIATED (armed and ready to launch)
- Nokia ringtone = LANDED (come find the rocket!)
- Beeping = In-flight (boost/coast/descent)
- Doom theme = "Sudo mode" (special diagnostic/testing mode)

This is especially useful when:
- Rocket lands out of visual range (follow the sound)
- Multiple people working on rocket (everyone hears state changes)
- Telemetry link is down (still have audio confirmation)
================================================================================
*/
void buzzer_thread(fjalar_t *fjalar, state_t *state, void *p3) {
    /*
    Check if buzzer is disabled via configuration
    CONFIG_BUZZER_ENABLED is set at compile time (can be turned off for
    quiet operations, noise-restricted areas, or battery conservation)
    */
    #if !CONFIG_BUZZER_ENABLED
    LOG_INF("Buzzer disabled");
    return;  // Exit thread immediately if buzzer disabled in config
    #endif
    
    int err;  // Error code for PWM operations
    
    /*
    Get buzzer PWM specification from device tree
    PWM allows precise frequency control needed for generating musical tones
    */
    const struct pwm_dt_spec buzzer_dt = PWM_DT_SPEC_GET(DT_ALIAS(buzzer));
    
    // Verify buzzer hardware is initialized and ready
    if (!device_is_ready(buzzer_dt.dev)) {
        LOG_ERR("buzzer is not ready");
        return;  // Can't use buzzer, exit thread
    }

    /*
    Calculate PWM period for 2000 Hz (2 kHz tone)
    This is used for the simple beeping pattern during flight
    2 kHz is loud and attention-getting, good for in-flight status
    
    period (ns) = 1 second / frequency = 1e9 / 2000 = 500,000 ns = 0.5ms
    */
    uint32_t center_period = 1e9 / 2000;
    
    /*
    Initialize buzzer to OFF state (duty cycle = 0)
    This ensures buzzer is silent at startup, preventing unwanted noise
    */
    err = pwm_set_dt(&buzzer_dt, center_period, 0);
    if (err) {
        LOG_ERR("Could not set buzzer");
        // Continue anyway - error might be transient
    }

    /*
    ============================================================================
    MAIN BUZZER LOOP
    ============================================================================
    Continuously plays appropriate audio feedback based on system state
    Should check terminate_actuation flag for graceful shutdown
    */
    while (true) {
        /*
        Check for "sudo mode" first (highest priority)
        This is a special diagnostic/testing mode that overrides normal
        flight state feedback. Useful for:
        - Demonstrating the system
        - Testing buzzer functionality
        - Identifying specific rocket in a fleet ("the one playing Doom")
        */
        if (fjalar->sudo) {
            // Play Doom theme continuously while in sudo mode
            play_song(&buzzer_dt, doom_melody, sizeof(doom_melody), doom_tempo);
            k_msleep(1000);  // 1 second pause between repetitions
            continue;         // Skip normal state machine, loop back to check sudo again
        }
        
        /*
        ========================================================================
        STATE-BASED AUDIO FEEDBACK
        ========================================================================
        Play different melodies based on current flight state
        flight_state is updated by the flight state machine based on sensor data
        */
        switch (state->flight_state) {
            
            case STATE_LANDED:
                /*
                Play Nokia ringtone - the classic "come find me" tone
                Repeats continuously so you can locate the rocket in tall grass/trees
                This is critical for recovery - rocket might land km away
                */
                play_song(&buzzer_dt, nokia_melody, sizeof(nokia_melody), nokia_tempo);
                k_msleep(1000);  // 1 second pause between songs
                break;

            case STATE_IDLE:
                /*
                Play Greensleeves - calm, peaceful melody
                Indicates system is powered on and healthy, but not armed
                Safe to approach rocket and make final preparations
                */
                play_song(&buzzer_dt, greensleeves_melody, sizeof(greensleeves_melody), greensleeves_tempo);
                k_msleep(1000);
                break;

            case STATE_INITIATED:
                /*
                Play Keyboard Cat - upbeat, energetic melody
                Indicates system is ARMED and ready to detect liftoff
                STAY BACK - rocket is ready to fly!
                This is your audio warning to clear the launch area
                */
                play_song(&buzzer_dt, keyboardcat_melody, sizeof(keyboardcat_melody), keyboardcat_tempo);
                k_msleep(1000);
                break;

            default:
                /*
                For all other states (BOOST, COAST, APOGEE, DESCENT, etc.):
                Play simple double-beep pattern
                
                This covers:
                -
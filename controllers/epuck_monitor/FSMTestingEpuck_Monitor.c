#include <webots/robot.h>
#include <webots/display.h> 
#include <stdio.h> 

// --- NASRUDIN'S GLOBAL ARCHITECTURAL VARIABLES ---
// -------------------------------------------------

typedef enum {
    PATROLLING,
    OBSTACLE_AVOIDANCE,
    ALARM_BLOCK
} FSM_STATE;

FSM_STATE current_state = PATROLLING; 

// Evaluation Variables
double T_detection = 0.0;
double T_stop = 0.0;
bool alarm_triggered = false; // Flag to ensure T_detection is logged only once

// --- (PLACEHOLDERS FOR TEAMMATES  WORK) ---
// -------------------------------------------------
// These functions and variables are temporary until integration.

// Input Mock 
bool VPE_VIOLATION_DETECTED = false; // Will be set based on timer for testing
bool IR_OBSTACLE_PRESENT = false;

// Output Mock
void set_motor_velocity(double left, double right) { 
    // In final integration, this will call Tairui's real motor functions.
}
void set_alarm_led(bool state) {
    // In final integration, this will call Tairui's real LED functions.
}


// --- CORE FSM AND CONTROL LOOP ---
// ---------------------------------------------------

void nasrudin_fsm_control_loop() {
    double time = wb_robot_get_time();

    // 1. --- MOCK INPUT LOGIC (INDEPENDENT TESTING) ---
    // Simulating a violation between 10.0 and 12.0 seconds for testing H1
    if (time > 10.0 && time < 12.0) {
        VPE_VIOLATION_DETECTED = true;
    } else {
        VPE_VIOLATION_DETECTED = false;
    }
    // Simulating IR detection every 8 seconds for 1 second
    if (((int)time % 8) < 1.0) {
        IR_OBSTACLE_PRESENT = true;
    } else {
        IR_OBSTACLE_PRESENT = false;
    }

    // 2. --- STATE TRANSITION LOGIC (Highest Priority Check First) ---
    if (VPE_VIOLATION_DETECTED) {
        // H1 Suppression Check
        if (!alarm_triggered) {
            T_detection = time; // Log time the alarm flag first went high
            alarm_triggered = true;
            printf("LOG: T_detection set at %f\n", T_detection);
        }
        current_state = ALARM_BLOCK;

    } else {
        // If NO VIOLATION, manage lower priority states
        switch (current_state) {
            case ALARM_BLOCK:
                // Recovery: Violation is cleared. Reset evaluation variables.
                current_state = PATROLLING;
                T_detection = 0.0; 
                T_stop = 0.0;
                alarm_triggered = false;
                printf("LOG: Alarm cleared.\n");
                break;
            case PATROLLING:
                if (IR_OBSTACLE_PRESENT) current_state = OBSTACLE_AVOIDANCE;
                break;
            case OBSTACLE_AVOIDANCE:
                if (!IR_OBSTACLE_PRESENT) current_state = PATROLLING;
                break;
        }
    }

    // 3. --- STATE ACTION LOGIC (Execute Commands) ---
    switch (current_state) {
        case PATROLLING:
            set_motor_velocity(2.0, 2.0);
            set_alarm_led(false);
            break;
        case OBSTACLE_AVOIDANCE:
            // Tairui's steering logic goes here later
            break;
        case ALARM_BLOCK:
            // CRITICAL SUPPRESSION ACTION
            set_motor_velocity(0.0, 0.0); // STOP IMMEDIATELY
            set_alarm_led(true); 

            if (T_stop == 0.0 && T_detection != 0.0) {
                T_stop = time; // Log time the stop command was executed
                printf("LOG: T_stop set at %f\n", T_stop);
                printf("LOG: REACTION TIME (RT) = %f\n", T_stop - T_detection);
                // In final evaluation, this data would be written to reaction_time_log.csv
            }
            break;
    }
}

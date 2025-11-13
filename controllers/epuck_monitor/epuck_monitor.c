// Define the states for the Finite State Machine
typedef enum {
    PATROLLING,
    OBSTACLE_AVOIDANCE,
    ALARM_BLOCK
} FSM_STATE;

FSM_STATE current_state = PATROLLING; // Start states

void nasrudin_fsm_control_loop() {
    // 1. --- STATE TRANSITION LOGIC (Highest Priority Check First) ---
    if (VPE_VIOLATION_DETECTED) {
        // H1 check: If violation detected, immediately suppress all other states.
        current_state = ALARM_BLOCK;
    } else {
        // If NO VIOLATION, manage lower priority states
        switch (current_state) {
            case ALARM_BLOCK:
                // Recovery: Violation is cleared, return to patrol.
                current_state = PATROLLING; 
                break;
            case PATROLLING:
                if (IR_OBSTACLE_PRESENT) current_state = OBSTACLE_AVOIDANCE;
                break;
            case OBSTACLE_AVOIDANCE:
                if (!IR_OBSTACLE_PRESENT) current_state = PATROLLING;
                break;
        }
    }

    // 2. --- STATE ACTION LOGIC (Execute Commands) ---
    switch (current_state) {
        case PATROLLING:
            set_motor_velocity(2.0, 2.0); // Placeholder values
            set_alarm_led(FALSE);
            break;
        case OBSTACLE_AVOIDANCE:
            // Tairui's logic goes here later
            break;
        case ALARM_BLOCK:
            // CRITICAL SUPPRESSION ACTION
            set_motor_velocity(0.0, 0.0); // STOP!
            set_alarm_led(TRUE); // ALARM!
            break;
    }
}

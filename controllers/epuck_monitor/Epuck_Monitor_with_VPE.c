#include <webots/robot.h>
#include <webots/receiver.h> 
#include <stdio.h>
#include <stdbool.h> 


#define TIME_STEP 64 

// FSM Definition
typedef enum {
    PATROLLING,
    OBSTACLE_AVOIDANCE,
    ALARM_BLOCK
} FSM_STATE;

FSM_STATE current_state = PATROLLING;

// Evaluation Variables 
double T_detection = 0.0;
double T_stop = 0.0;
bool alarm_triggered = false; 

// --- INTEGRATION VARIABLES & RECEIVER ---
bool IR_OBSTACLE_PRESENT = false; 

// Webots Receiver for VPE signal
WbDeviceTag receiver; 
bool VPE_VIOLATION_DETECTED = false; 

// Output Mocks (Placeholder functions for Actuator Control)
void set_motor_velocity(double left, double right) { /* Placeholder */ }
void set_alarm_led(bool state) { /* Placeholder */ }

// --- FSM INITIALIZATION FUNCTION ---
void control_init() {
    // Get and enable the receiver device
    receiver = wb_robot_get_device("receiver"); 
    wb_receiver_enable(receiver, TIME_STEP);
    wb_receiver_set_channel(receiver, 1); 
}

// --- INPUT READING FUNCTION ---
void read_vpe_input() {
    // Read all pending messages from the receiver queue
    if (wb_receiver_get_queue_length(receiver) > 0) {
        const char *data = wb_receiver_get_data(receiver);
        
        // The data is a single byte (0 or 1). Cast it to an integer.
        int alarm_status = (int)data[0];
        
        // Update the global VPE flag
        VPE_VIOLATION_DETECTED = (alarm_status == 1);
        
        // Clear the message queue for the next step
        wb_receiver_next_packet(receiver);
    }
}

// --- CORE FSM AND CONTROL LOOP ---
void fsm_control_loop() {
    double time = wb_robot_get_time();

    // 1. --- READ ACTUAL INPUTS ---
    read_vpe_input(); // Reads VPE signal

    // 2. --- STATE TRANSITION LOGIC (Highest Priority Check First) ---
    if (VPE_VIOLATION_DETECTED) {
        // H1 Suppression Check & T_detection Logging
        if (!alarm_triggered) {
            T_detection = time; 
            alarm_triggered = true;
            printf("LOG: T_detection set at %f\n", T_detection);
        }
        current_state = ALARM_BLOCK;

    } else {
        // If NO VIOLATION
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
                // Mock IR check (Will use Tairui's real IR check later)
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
            set_motor_velocity(0.0, 0.0);
            set_alarm_led(true); 

            if (T_stop == 0.0 && T_detection != 0.0) {
                T_stop = time; 
                printf("LOG: T_stop set at %f\n", T_stop);
                printf("LOG: REACTION TIME (RT) = %f\n", T_stop - T_detection);
                // Data logging logic here for CSV
            }
            break;
    }
}

int main(int argc, char **argv) {
    
    wb_robot_init();
    
    control_init(); 
    
    while (wb_robot_step(TIME_STEP) != -1) {
        fsm_control_loop();
    }

    wb_robot_cleanup();
    return 0;
}

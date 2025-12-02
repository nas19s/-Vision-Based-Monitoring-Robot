/*
 * Stationary e-puck camera monitor.
 * This version focuses on detecting a very specific shade of bright green
 * used by the EV marker. The thresholds are intentionally strict to avoid
 * picking up greenish walls or reflections.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define TIME_STEP 32

/* 
 * "Pure" green thresholds — tuned around the EV marker's emissive color.
 * The marker is basically (0, 255, 0), so we only accept pixels that are
 * strongly dominated by the green channel.
 */
#define PURE_GREEN_R_MAX 80
#define PURE_GREEN_G_MIN 180
#define PURE_GREEN_B_MAX 80
#define GREEN_DOMINANCE_RATIO 2.0     // G must be at least twice R and B
#define EV_DETECTION_THRESHOLD 20     // Minimum pixels to count as a detection

typedef enum {
    STATE_MONITORING,
    STATE_TARGET_DETECTED,
    STATE_ALARM_ACTIVE
} MonitorState;

static const char* state_names[] = {
    "MONITORING",
    "TARGET_DETECTED",
    "ALARM_ACTIVE"
};

/* Webots device tags */
static WbDeviceTag left_motor, right_motor;
static WbDeviceTag receiver;
static WbDeviceTag leds[10];
static WbDeviceTag camera;

/* Camera info */
static int camera_width = 0;
static int camera_height = 0;

/* LED flashing */
static int flash_counter = 0;
static int target_lock_timer = 0;

/* Experiment data (H1 tests) */
static int alarm_count = 0;
static double alarm_start_times[100];
static double response_times[100];
static MonitorState state_at_alarm[100];
static int detection_method[100];          // 0 = VPE, 1 = camera
static int green_pixels_at_detection[100];

/* Camera detection results */
static bool camera_ev_detected = false;
static int camera_pure_green_pixels = 0;
static int detected_x = -1;
static int detected_y = -1;


static void all_leds_on(void) {
    for (int i = 0; i < 10; i++) {
        if (leds[i])
            wb_led_set(leds[i], 1);
    }
}

static void all_leds_off(void) {
    for (int i = 0; i < 10; i++) {
        if (leds[i])
            wb_led_set(leds[i], 0);
    }
}

static void flash_leds(void) {
    flash_counter++;
    if ((flash_counter / 8) % 2 == 0)
        all_leds_on();
    else
        all_leds_off();
}

static void init_devices(void) {
    /* Motors – robot stays stationary */
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");

    if (left_motor && right_motor) {
        wb_motor_set_position(left_motor, INFINITY);
        wb_motor_set_position(right_motor, INFINITY);
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        printf("[INIT] Motors locked.\n");
    }

    /* Receiver */
    receiver = wb_robot_get_device("receiver");
    if (receiver) {
        wb_receiver_enable(receiver, TIME_STEP);
        printf("[INIT] Receiver enabled.\n");
    }

    /* Camera */
    camera = wb_robot_get_device("camera");
    if (camera) {
        wb_camera_enable(camera, TIME_STEP);
        camera_width = wb_camera_get_width(camera);
        camera_height = wb_camera_get_height(camera_height);

        printf("[INIT] Camera: %dx%d\n", camera_width, camera_height);
        printf("       Pure green thresholds: R<%d, G>%d, B<%d\n",
               PURE_GREEN_R_MAX, PURE_GREEN_G_MIN, PURE_GREEN_B_MAX);
    }

    /* LEDs */
    char name[8];
    int found = 0;
    for (int i = 0; i < 10; i++) {
        sprintf(name, "led%d", i);
        leds[i] = wb_robot_get_device(name);
        if (leds[i])
            found++;
    }
    printf("[INIT] %d LEDs found.\n", found);

    /* Quick LED test */
    for (int i = 0; i < 2; i++) {
        all_leds_on();
        wb_robot_step(80);
        all_leds_off();
        wb_robot_step(80);
    }
}

/*
 * Camera processing:
 * Count how many pixels match the strict green definition.
 */
static bool process_camera_for_ev(void) {
    if (!camera)
        return false;

    const unsigned char* image = wb_camera_get_image(camera);
    if (!image)
        return false;

    int pure_count = 0;
    int sx = 0, sy = 0;

    for (int y = 0; y < camera_height; y++) {
        for (int x = 0; x < camera_width; x++) {
            int r = wb_camera_image_get_red(image, camera_width, x, y);
            int g = wb_camera_image_get_green(image, camera_width, x, y);
            int b = wb_camera_image_get_blue(image, camera_width, x, y);

            bool is_green =
                r < PURE_GREEN_R_MAX &&
                g > PURE_GREEN_G_MIN &&
                b < PURE_GREEN_B_MAX &&
                g > r * GREEN_DOMINANCE_RATIO &&
                g > b * GREEN_DOMINANCE_RATIO;

            if (is_green) {
                pure_count++;
                sx += x;
                sy += y;
            }
        }
    }

    camera_pure_green_pixels = pure_count;

    if (pure_count >= EV_DETECTION_THRESHOLD) {
        camera_ev_detected = true;
        detected_x = sx / pure_count;
        detected_y = sy / pure_count;
    } else {
        camera_ev_detected = false;
        detected_x = detected_y = -1;
    }

    return camera_ev_detected;
}

static void save_experiment_data(void) {
    FILE* f = fopen("h1_experiment_data.csv", "w");
    if (!f)
        return;

    fprintf(f, "alarm_num,time_sec,response_ms,prev_state,method,green_pixels\n");

    for (int i = 0; i < alarm_count; i++) {
        fprintf(f, "%d,%.3f,%.3f,%s,%s,%d\n",
                i + 1,
                alarm_start_times[i],
                response_times[i] * 1000.0,
                state_names[state_at_alarm[i]],
                (detection_method[i] == 0 ? "VPE" : "CAMERA"),
                green_pixels_at_detection[i]);
    }

    fclose(f);
    printf("[DATA] Saved %d entries.\n", alarm_count);
}

static void print_status(double t, MonitorState s, bool alarm) {
    static int counter = 0;
    counter++;

    if (counter % 31 == 0) {
        if (alarm)
            printf("[%.1fs] %-14s | %4d green px | ALARM\n",
                   t, state_names[s], camera_pure_green_pixels);
        else
            printf("[%.1fs] %-14s | %4d green px\n",
                   t, state_names[s], camera_pure_green_pixels);
    }
}


int main(int argc, char** argv) {
    wb_robot_init();
    init_devices();

    MonitorState current = STATE_MONITORING;
    MonitorState previous = STATE_MONITORING;

    bool vpe_active = false;
    bool cam_active = false;

    double alarm_start = 0;

    printf("Stationary camera monitor running (strict green detection)...\n");

    while (wb_robot_step(TIME_STEP) != -1) {
        double now = wb_robot_get_time();
        previous = current;

        /* --- VPE input --- */
        while (wb_receiver_get_queue_length(receiver) > 0) {
            const unsigned char* data = wb_receiver_get_data(receiver);
            int val = data[0];

            if (val == 1 && !vpe_active) {
                vpe_active = true;
                alarm_start = now;

                if (alarm_count < 100) {
                    alarm_start_times[alarm_count] = now;
                    state_at_alarm[alarm_count] = previous;
                    response_times[alarm_count] = TIME_STEP / 1000.0;
                    detection_method[alarm_count] = 0;
                    green_pixels_at_detection[alarm_count] = camera_pure_green_pixels;
                    alarm_count++;
                }

                printf("\n[VPE] Alarm triggered at %.2fs\n", now);
            } else if (val == 0 && vpe_active) {
                printf("[VPE] Cleared (%.2fs)\n\n", now - alarm_start);
                vpe_active = false;
            }

            wb_receiver_next_packet(receiver);
        }

        /* --- Camera detection --- */
        bool cam_seen = process_camera_for_ev();

        if (cam_seen && !cam_active) {
            cam_active = true;
            alarm_start = now;

            if (alarm_count < 100) {
                alarm_start_times[alarm_count] = now;
                state_at_alarm[alarm_count] = previous;
                response_times[alarm_count] = TIME_STEP / 1000.0;
                detection_method[alarm_count] = 1;
                green_pixels_at_detection[alarm_count] = camera_pure_green_pixels;
                alarm_count++;
            }

            printf("\n[CAMERA] EV detected (%d px) at %.2fs\n",
                   camera_pure_green_pixels, now);
        } else if (!cam_seen && cam_active) {
            printf("[CAMERA] Lost EV (%.2fs)\n\n", now - alarm_start);
            cam_active = false;
        }

        /* Combined alarm state */
        bool alarm = vpe_active || cam_active;

        /* --- State machine --- */
        if (alarm) {
            if (current == STATE_MONITORING) {
                current = STATE_TARGET_DETECTED;
                target_lock_timer = 0;
            } else if (current == STATE_TARGET_DETECTED) {
                if (++target_lock_timer > 2)
                    current = STATE_ALARM_ACTIVE;
            }
        } else {
            current = STATE_MONITORING;
            target_lock_timer = 0;
        }

        /* --- LEDs --- */
        switch (current) {
        case STATE_MONITORING:
            all_leds_off();
            if (leds[0])
                wb_led_set(leds[0], 1);
            break;

        case STATE_TARGET_DETECTED:
            for (int i = 0; i < 5; i++)
                if (leds[i]) wb_led_set(leds[i], 1);
            for (int i = 5; i < 10; i++)
                if (leds[i]) wb_led_set(leds[i], 0);
            break;

        case STATE_ALARM_ACTIVE:
            flash_leds();
            break;
        }

        print_status(now, current, alarm);
    }

    save_experiment_data();
    wb_robot_cleanup();
    return 0;
}

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28
#define CRUISING_SPEED 4.0
#define PURSUIT_SPEED 3.5

/* Vision Thresholds */
#define GREEN_R_MAX 100
#define GREEN_G_MIN 150
#define GREEN_B_MAX 100
#define GREEN_DOMINANCE 1.5

#define BLACK_R_MAX 50
#define BLACK_G_MAX 50
#define BLACK_B_MAX 50
#define BLACK_BRIGHTNESS_MAX 100

#define BLACK_R_MAX_CLOSE 70
#define BLACK_G_MAX_CLOSE 70
#define BLACK_B_MAX_CLOSE 70
#define BLACK_BRIGHTNESS_MAX_CLOSE 150

#define MIN_GREEN_PIXELS 15
#define MIN_GREEN_FOR_TRACKING 5

#define MIN_WHEEL_PIXELS 5
#define MIN_TOTAL_WHEELS 8
#define MIN_WHEEL_PIXELS_STRICT 10
#define MIN_WHEELS_EACH_SIDE 4
#define WHEEL_TO_GREEN_RATIO_MIN 0.03

#define MIN_BLACK_PIXELS_CLOSE 15
#define MIN_BLACK_BELOW_CLOSE 8

#define WHEEL_BELOW_MARGIN 3
#define WHEEL_SIDE_MARGIN 10
#define ASPECT_RATIO_MIN 0.5
#define ASPECT_RATIO_MAX 3.0

#define PILLAR_ASPECT_RATIO_MIN 1.5
#define PILLAR_HEIGHT_MIN 0.6

#define CONFIDENCE_LOW 0.3
#define CONFIDENCE_MEDIUM 0.5
#define CONFIDENCE_HIGH 0.8

#define CLOSE_THRESHOLD 150
#define VERY_CLOSE_THRESHOLD 300
#define TURN_SENSITIVITY 3.0

#define OBSTACLE_THRESHOLD_HIGH 150.0
#define OBSTACLE_THRESHOLD_MED 100.0
#define OBSTACLE_THRESHOLD_LOW 70.0

static const double BRAITENBERG_LEFT[8] = {
    -1.5, -1.0, -0.5, 0.0,
     0.0,  0.5,  1.0, 1.5
};
static const double BRAITENBERG_RIGHT[8] = {
     1.5,  1.0,  0.5, 0.0,
     0.0, -0.5, -1.0, -1.5
};

#define AVOID_TURN_SPEED 3.0
#define AVOID_BACKUP_SPEED -2.0
#define OBSTACLE_MEMORY_FRAMES 20

#define TARGET_STOP_DIST 300.0

#define SCAN_INTERVAL 500
#define SCAN_DURATION 80
#define LOST_TARGET_TIMEOUT 50
#define STATUS_LOG_INTERVAL 100

#define LOCK_LOST_TIMEOUT 30

/* Re-validation thresholds - CRITICAL FOR PILLAR REJECTION */
#define REVALIDATE_INTERVAL 8
#define NO_WHEELS_INTERRUPT_COUNT 5
#define NO_WHEELS_FORCE_RELEASE_COUNT 10
#define WHEELS_CONFIRM_COUNT 3
#define ZERO_BLACK_FORCE_RELEASE 8

/* Robot State Machine */
typedef enum {
    STATE_PATROL,
    STATE_SCANNING,
    STATE_VALIDATING,
    STATE_PURSUIT,
    STATE_APPROACHING,
    STATE_ALARM_BLOCK,
    STATE_AVOIDING
} RobotState;

const char* state_names[] = {
    "PATROL", "SCANNING", "VALIDATING", "PURSUIT", "APPROACHING", "ALARM_BLOCK", "AVOIDING"
};

typedef struct {
    int green_pixels;
    double green_center_x;
    double green_center_y;
    double green_min_x, green_max_x;
    double green_min_y, green_max_y;
    double green_width;
    double green_height;

    int black_pixels_total;
    int black_pixels_left;
    int black_pixels_right;
    int black_pixels_below;
    int black_pixels_bottom_third;
    double black_center_x;
    double black_center_y;
    
    double wheel_to_green_ratio;
    bool has_green;
    bool has_black_below;
    bool has_wheels;
    bool wheels_below_chassis;
    bool wheels_on_sides;
    bool valid_aspect_ratio;
    
    bool is_likely_pillar;
    double height_ratio;
    double aspect_ratio;

    bool is_close_range;
    bool is_very_close;

    bool is_vehicle;
    double confidence;
    char rejection_reason[128]; 
} VehicleDetection;

typedef struct {
    bool is_locked;
    double lock_confidence;
    double lock_time;
    double last_seen_x;
    double last_seen_y;
    int frames_since_seen;
    int total_pursuit_frames;
    
    int no_wheels_count;
    int zero_black_count;         // Consecutive frames with 0 black pixels
    int wheels_confirm_count;     
    int revalidate_counter;       
    bool wheels_confirmed;        
} TargetLock;

typedef struct {
    double sensor_values[8];
    double front_left;
    double front_right;
    double side_left;
    double side_right;
    
    bool obstacle_front;
    bool obstacle_left;
    bool obstacle_right;
    bool any_obstacle;

    double obstacle_intensity;
    int avoid_direction;
    int avoid_timer;
    RobotState return_state; 
} ObstacleData;

static WbDeviceTag left_motor, right_motor;
static WbDeviceTag ps[8];
static WbDeviceTag camera;
static WbDeviceTag leds[10];
static WbDeviceTag receiver; 

static int cam_width, cam_height;

static RobotState current_state = STATE_PATROL;
static RobotState previous_state = STATE_PATROL; 

static int scan_timer = 0;
static int scan_counter = 0;
static int lost_target_counter = 0;
static int validation_counter = 0;
static double left_speed = 0.0;
static double right_speed = 0.0;

static VehicleDetection detection;
static TargetLock target_lock = {0};
static ObstacleData obstacles = {0};
static int step_counter = 0;

static FILE *fp = NULL;
static FILE *detection_log = NULL;
static int alarm_logged = 0;

static int total_green_detections = 0;
static int validated_vehicle_detections = 0;
static int false_positive_rejections = 0;
static int obstacle_avoidances = 0;
static int pillar_rejections = 0;
static int wheel_revalidations = 0;
static int no_wheel_interrupts = 0;
static int forced_releases = 0;

/* ============================================
   HELPER FUNCTIONS
   ============================================ */

void lights_out(void) {
    for (int i = 0; i < 10; i++) {
        if (leds[i]) wb_led_set(leds[i], 0);
    }
}

double clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void init_devices(void) {
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    char ps_name[4];
    for (int i = 0; i < 8; i++) {
        sprintf(ps_name, "ps%d", i);
        ps[i] = wb_robot_get_device(ps_name);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    camera = wb_robot_get_device("camera");
    if (camera) {
        wb_camera_enable(camera, TIME_STEP);
        cam_width = wb_camera_get_width(camera);
        cam_height = wb_camera_get_height(camera);
        printf("[INIT] Camera activated: %dx%d\n", cam_width, cam_height);
    }

    char led_name[8];
    for (int i = 0; i < 10; i++) {
        sprintf(led_name, "led%d", i);
        leds[i] = wb_robot_get_device(led_name);
    }

    receiver = wb_robot_get_device("receiver");
    if (receiver) {
        wb_receiver_enable(receiver, TIME_STEP);
    }

    printf("\n--- E-Puck EV Tracker v3.1 ---\n");
    printf("EV = GREEN chassis + BLACK wheels\n");
    printf("Pillar rejection logic is ON.\n");
    printf("------------------------------\n\n");
}

/* --- LED Patterns --- */

void set_patrol_lights(void) {
    lights_out();
    if (leds[0]) wb_led_set(leds[0], 1);
}

void set_pursuit_lights(void) {
    lights_out();
    if (leds[0]) wb_led_set(leds[0], 1);
    if (leds[7]) wb_led_set(leds[7], 1);
}

void set_locked_lights(void) {
    lights_out();
    if (leds[0]) wb_led_set(leds[0], 1);
    if (leds[1]) wb_led_set(leds[1], 1);
    if (leds[6]) wb_led_set(leds[6], 1);
    if (leds[7]) wb_led_set(leds[7], 1);
}

void flash_confirmed_target(void) {
    lights_out();
    for (int i = 0; i < 8; i++) {
        if (leds[i]) wb_led_set(leds[i], 1);
    }
}

void flash_validating_lights(void) {
    int time_ms = (int)(wb_robot_get_time() * 1000);
    int phase = (time_ms / 150) % 2;
    lights_out();
    for (int i = phase; i < 8; i += 2) {
        if (leds[i]) wb_led_set(leds[i], 1);
    }
}

void pulse_approaching_lights(void) {
    int time_ms = (int)(wb_robot_get_time() * 1000);
    int phase = (time_ms / 100) % 4;
    lights_out();
    if (leds[phase]) wb_led_set(leds[phase], 1);
    if (leds[(phase + 4) % 8]) wb_led_set(leds[(phase + 4) % 8], 1);
}

void show_avoiding_lights(void) {
    lights_out();
    if (obstacles.obstacle_left) {
        if (leds[5]) wb_led_set(leds[5], 1);
        if (leds[6]) wb_led_set(leds[6], 1);
    }
    if (obstacles.obstacle_right) {
        if (leds[1]) wb_led_set(leds[1], 1);
        if (leds[2]) wb_led_set(leds[2], 1);
    }
    if (obstacles.obstacle_front) {
        if (leds[0]) wb_led_set(leds[0], 1);
        if (leds[7]) wb_led_set(leds[7], 1);
    }
}

/* ============================================
   OBSTACLE SENSING / BRAITENBERG
   ============================================ */
ObstacleData read_obstacle_sensors(void) {
    ObstacleData obs = {0};

    for (int i = 0; i < 8; i++) {
        obs.sensor_values[i] = wb_distance_sensor_get_value(ps[i]);
    }

    obs.front_right = obs.sensor_values[0];
    obs.front_left = obs.sensor_values[7];
    obs.side_right = (obs.sensor_values[1] + obs.sensor_values[2]) / 2.0;
    obs.side_left = (obs.sensor_values[5] + obs.sensor_values[6]) / 2.0;
    
    obs.obstacle_front = (obs.front_left > OBSTACLE_THRESHOLD_MED ||
                          obs.front_right > OBSTACLE_THRESHOLD_MED);
    obs.obstacle_left = (obs.side_left > OBSTACLE_THRESHOLD_MED ||
                         obs.sensor_values[6] > OBSTACLE_THRESHOLD_MED);
    obs.obstacle_right = (obs.side_right > OBSTACLE_THRESHOLD_MED ||
                          obs.sensor_values[1] > OBSTACLE_THRESHOLD_MED);

    obs.any_obstacle = obs.obstacle_front || obs.obstacle_left || obs.obstacle_right;

    double max_reading = 0;
    for (int i = 0; i < 8; i++) {
        if (obs.sensor_values[i] > max_reading) max_reading = obs.sensor_values[i];
    }
    obs.obstacle_intensity = max_reading;

    if (obs.obstacle_front) {
        double left_clear = OBSTACLE_THRESHOLD_HIGH - obs.side_left;
        double right_clear = OBSTACLE_THRESHOLD_HIGH - obs.side_right;

        if (obs.front_left > obs.front_right + 20) {
            obs.avoid_direction = 1; 
        } else if (obs.front_right > obs.front_left + 20) {
            obs.avoid_direction = -1; 
        } else if (right_clear > left_clear) {
            obs.avoid_direction = 1;
        } else {
            obs.avoid_direction = -1;
        }
    } else {
        obs.avoid_direction = 0;
    }

    return obs;
}

void calculate_braitenberg_speeds(double *left_out, double *right_out) {
    double left_speed_mod = 0.0;
    double right_speed_mod = 0.0;
    
    for (int i = 0; i < 8; i++) {
        double normalized = obstacles.sensor_values[i] / OBSTACLE_THRESHOLD_HIGH;
        if (normalized > 1.0) normalized = 1.0;

        left_speed_mod += normalized * BRAITENBERG_LEFT[i];
        right_speed_mod += normalized * BRAITENBERG_RIGHT[i];
    }

    double base_speed = CRUISING_SPEED;

    if (obstacles.obstacle_intensity > OBSTACLE_THRESHOLD_MED) base_speed *= 0.7;
    if (obstacles.obstacle_intensity > OBSTACLE_THRESHOLD_HIGH) base_speed *= 0.5;

    *left_out = base_speed + left_speed_mod * 1.5;
    *right_out = base_speed + right_speed_mod * 1.5;

    *left_out = clamp(*left_out, -MAX_SPEED, MAX_SPEED);
    *right_out = clamp(*right_out, -MAX_SPEED, MAX_SPEED);
}

bool should_avoid_obstacle(void) {
    
    // If locked on a confirmed target EV, assume proximity is the EV itself.
    if (target_lock.is_locked && target_lock.wheels_confirmed && 
        detection.black_pixels_total > 0) {
        return false;
    }
    
    // If locked but ZERO black, let pillar check handle it.
    if (target_lock.is_locked && detection.black_pixels_total == 0) {
        return false; 
    }
    
    // If locked but not confirmed, don't avoid if we see the green body.
    if (target_lock.is_locked && detection.green_pixels >= MIN_GREEN_FOR_TRACKING) {
        return false;
    }

    bool immediate_danger = (obstacles.front_left > OBSTACLE_THRESHOLD_HIGH ||
                             obstacles.front_right > OBSTACLE_THRESHOLD_HIGH);

    return immediate_danger ||
           (obstacles.any_obstacle && obstacles.obstacle_intensity > OBSTACLE_THRESHOLD_MED);
}

/* ============================================
   TARGET LOCKING LOGIC
   ============================================ */
void acquire_target_lock(double confidence, double center_x, double center_y, bool wheels_seen) {
    target_lock.is_locked = true;
    target_lock.lock_confidence = confidence;
    target_lock.lock_time = wb_robot_get_time();
    target_lock.last_seen_x = center_x;
    target_lock.last_seen_y = center_y;
    target_lock.frames_since_seen = 0;
    target_lock.total_pursuit_frames = 0;
    target_lock.no_wheels_count = 0;
    target_lock.zero_black_count = 0;
    target_lock.wheels_confirm_count = wheels_seen ? 1 : 0;
    target_lock.revalidate_counter = 0;
    target_lock.wheels_confirmed = wheels_seen;

    printf("[LOCK] *** TARGET ACQUIRED *** (%.0f%% confidence)\n", confidence * 100);
    if (wheels_seen) {
        printf("[LOCK] High confidence lock: Wheels visible.\n");
    }
}

void update_target_lock(bool can_see_target, double center_x, double center_y) {
    if (!target_lock.is_locked) return;

    target_lock.total_pursuit_frames++;
    target_lock.revalidate_counter++;

    if (can_see_target) {
        target_lock.last_seen_x = center_x;
        target_lock.last_seen_y = center_y;
        target_lock.frames_since_seen = 0;
    } else {
        target_lock.frames_since_seen++;
    }
}

void release_target_lock(const char* reason) {
    if (target_lock.is_locked) {
        double duration = wb_robot_get_time() - target_lock.lock_time;
        printf("[LOCK] *** LOCK RELEASED *** after %.1fs. Reason: %s\n", duration, reason);
    }

    target_lock.is_locked = false;
    target_lock.lock_confidence = 0;
    target_lock.frames_since_seen = 0;
    target_lock.total_pursuit_frames = 0;
    target_lock.no_wheels_count = 0;
    target_lock.zero_black_count = 0;
    target_lock.wheels_confirm_count = 0;
    target_lock.wheels_confirmed = false;
}

bool is_target_lock_valid(void) {
    if (!target_lock.is_locked) return false;

    int timeout = target_lock.wheels_confirmed ? 
                  LOCK_LOST_TIMEOUT * 5 : LOCK_LOST_TIMEOUT * 2;

    if (target_lock.frames_since_seen > timeout) {
        release_target_lock("target lost for too long");
        return false;
    }

    return true;
}

/* === Critical Pillar Rejection Logic === */
bool check_wheel_revalidation(void) {
    if (!target_lock.is_locked) return false;
    
    if (target_lock.revalidate_counter < REVALIDATE_INTERVAL) return false;
    target_lock.revalidate_counter = 0;
    
    /* CHECK 1: ZERO BLACK PIXELS? */
    if (detection.black_pixels_total == 0) {
        target_lock.zero_black_count++;
        
        printf("[REVALIDATE] ZERO black pixels detected (%d/%d)\n",
               target_lock.zero_black_count, ZERO_BLACK_FORCE_RELEASE);
        
        // Force release on consistent zero black
        if (target_lock.zero_black_count >= ZERO_BLACK_FORCE_RELEASE) {
            forced_releases++;
            printf("[REVALIDATE] *** FORCE RELEASE *** - Definitely a PILLAR (zero black frames)\n");
            release_target_lock("PILLAR - zero black pixels detected consistently");
            return true;
        }
    } else {
        target_lock.zero_black_count = 0;
    }
    
    /* CHECK 2: ARE WHEELS BELOW THE CHASSIS? */
    bool has_wheels_now = detection.has_black_below || 
                          (detection.black_pixels_below >= MIN_BLACK_BELOW_CLOSE);
    
    if (detection.is_very_close && detection.black_pixels_total >= MIN_BLACK_PIXELS_CLOSE) {
        has_wheels_now = true;
    }
    
    if (has_wheels_now) {
        target_lock.wheels_confirm_count++;
        target_lock.no_wheels_count = 0;
        
        if (target_lock.wheels_confirm_count >= WHEELS_CONFIRM_COUNT && 
            !target_lock.wheels_confirmed) {
            target_lock.wheels_confirmed = true;
            wheel_revalidations++;
            printf("[REVALIDATE] WHEELS CONFIRMED - Trusting this target now.\n");
        }
    } else {
        target_lock.no_wheels_count++;
        target_lock.wheels_confirm_count = 0;
        
        printf("[REVALIDATE] No wheels detected (%d/%d) - is it a pillar?\n",
               target_lock.no_wheels_count, NO_WHEELS_INTERRUPT_COUNT);
        
        if (target_lock.no_wheels_count >= NO_WHEELS_INTERRUPT_COUNT) {
            if (detection.is_likely_pillar || detection.black_pixels_total < MIN_BLACK_PIXELS_CLOSE) {
                
                if (!target_lock.wheels_confirmed) {
                    no_wheel_interrupts++;
                    release_target_lock("no wheels detected - likely pillar");
                    return true;
                } 
                else if (target_lock.no_wheels_count >= NO_WHEELS_FORCE_RELEASE_COUNT) {
                    forced_releases++;
                    printf("[REVALIDATE] *** FORCE RELEASE *** - Lost wheels after confirmation.\n");
                    release_target_lock("PILLAR - lost wheels for too long");
                    return true;
                }
            }
        }
    }
    
    return false;
}

/* ============================================
   VISION / FEATURE DETECTION
   ============================================ */
VehicleDetection process_camera_advanced(void) {
    VehicleDetection det = {0};
    strcpy(det.rejection_reason, "");

    if (!camera) {
        strcpy(det.rejection_reason, "No camera");
        return det;
    }

    const unsigned char *image = wb_camera_get_image(camera);
    if (!image) {
        strcpy(det.rejection_reason, "No image");
        return det;
    }

    det.green_min_x = cam_width;
    det.green_max_x = 0;
    det.green_min_y = cam_height;
    det.green_max_y = 0;

    long green_sum_x = 0, green_sum_y = 0;
    long black_sum_x = 0, black_sum_y = 0;
    int black_left = 0, black_right = 0, black_below = 0;
    int black_bottom_third = 0;
    
    int bottom_third_start = (cam_height * 2) / 3;

    /* --- PASS 1: Pixel Classification --- */
    for (int y = 0; y < cam_height; y++) {
        for (int x = 0; x < cam_width; x++) {
            int r = wb_camera_image_get_red(image, cam_width, x, y);
            int g = wb_camera_image_get_green(image, cam_width, x, y);
            int b = wb_camera_image_get_blue(image, cam_width, x, y);
            int brightness = r + g + b;

            /* Green check */
            bool is_green = (r < GREEN_R_MAX) &&
                            (g > GREEN_G_MIN) &&
                            (b < GREEN_B_MAX) &&
                            (g > r * GREEN_DOMINANCE) &&
                            (g > b * GREEN_DOMINANCE);

            if (is_green) {
                det.green_pixels++;
                green_sum_x += x;
                green_sum_y += y;

                if (x < det.green_min_x) det.green_min_x = x;
                if (x > det.green_max_x) det.green_max_x = x;
                if (y < det.green_min_y) det.green_min_y = y;
                if (y > det.green_max_y) det.green_max_y = y;
            }

            /* Black check */
            bool is_black = (r < BLACK_R_MAX) && (g < BLACK_G_MAX) && 
                            (b < BLACK_B_MAX) && (brightness < BLACK_BRIGHTNESS_MAX);

            bool is_black_lenient = (r < BLACK_R_MAX_CLOSE) && (g < BLACK_G_MAX_CLOSE) && 
                                    (b < BLACK_B_MAX_CLOSE) && (brightness < BLACK_BRIGHTNESS_MAX_CLOSE);

            if (is_black || is_black_lenient) {
                det.black_pixels_total++;
                black_sum_x += x;
                black_sum_y += y;
                
                if (y >= bottom_third_start) {
                    black_bottom_third++;
                }
            }
        }
    }

    /* Post-Pass 1 Calculations */
    if (det.green_pixels >= MIN_GREEN_PIXELS) {
        det.has_green = true;
        det.green_center_x = (double)green_sum_x / det.green_pixels;
        det.green_center_y = (double)green_sum_y / det.green_pixels;
        det.green_width = det.green_max_x - det.green_min_x;
        det.green_height = det.green_max_y - det.green_min_y;
    } else if (det.green_pixels >= MIN_GREEN_FOR_TRACKING) {
        det.has_green = true; 
        det.green_center_x = (double)green_sum_x / det.green_pixels;
        det.green_center_y = (double)green_sum_y / det.green_pixels;
    }

    if (det.black_pixels_total > 0) {
        det.black_center_x = (double)black_sum_x / det.black_pixels_total;
        det.black_center_y = (double)black_sum_y / det.black_pixels_total;
    }
    det.black_pixels_bottom_third = black_bottom_third;

    det.is_close_range = (det.green_pixels > CLOSE_THRESHOLD);
    det.is_very_close = (det.green_pixels > VERY_CLOSE_THRESHOLD);

    if (det.green_width > 0 && det.green_height > 0) {
        det.aspect_ratio = det.green_height / det.green_width;
        det.height_ratio = det.green_height / (double)cam_height;
    }

    /* --- PASS 2: Relational Analysis (Black pixels relative to Green) --- */
    if (det.has_green && det.black_pixels_total > 0) {
        int center_x = (int)det.green_center_x;
        
        for (int y = 0; y < cam_height; y++) {
            for (int x = 0; x < cam_width; x++) {
                int r = wb_camera_image_get_red(image, cam_width, x, y);
                int g = wb_camera_image_get_green(image, cam_width, x, y);
                int b = wb_camera_image_get_blue(image, cam_width, x, y);
                int brightness = r + g + b;

                bool is_black = ((r < BLACK_R_MAX && g < BLACK_G_MAX && 
                                  b < BLACK_B_MAX && brightness < BLACK_BRIGHTNESS_MAX) ||
                                 (r < BLACK_R_MAX_CLOSE && g < BLACK_G_MAX_CLOSE && 
                                  b < BLACK_B_MAX_CLOSE && brightness < BLACK_BRIGHTNESS_MAX_CLOSE));

                if (is_black) {
                    // Black pixels near the bottom edge of the green chassis?
                    if (y > det.green_max_y - WHEEL_BELOW_MARGIN) {
                        black_below++;
                    }
                    
                    // Black pixels to the left/right of the green center?
                    if (x < center_x - WHEEL_SIDE_MARGIN) {
                        black_left++;
                    } else if (x > center_x + WHEEL_SIDE_MARGIN) {
                        black_right++;
                    }
                }
            }
        }
        
        det.black_pixels_below = black_below;
        det.black_pixels_left = black_left;
        det.black_pixels_right = black_right;
        
        if (det.green_pixels > 0) {
            det.wheel_to_green_ratio = (double)det.black_pixels_total / det.green_pixels;
        }
    }

    /* --- Feature Synthesis --- */
    
    int min_black_below = det.is_close_range ? MIN_BLACK_BELOW_CLOSE : MIN_WHEEL_PIXELS_STRICT;
    int min_black_total = det.is_close_range ? MIN_BLACK_PIXELS_CLOSE : MIN_TOTAL_WHEELS;
    
    det.has_black_below = (det.black_pixels_below >= min_black_below) ||
                          (det.black_pixels_bottom_third >= min_black_total);
    
    det.has_wheels = (det.black_pixels_total >= min_black_total);
    
    det.wheels_below_chassis = det.has_black_below && 
                               (det.black_center_y > det.green_center_y);
    
    det.wheels_on_sides = (det.black_pixels_left >= MIN_WHEELS_EACH_SIDE) &&
                          (det.black_pixels_right >= MIN_WHEELS_EACH_SIDE);

    if (det.green_height > 0 && det.green_width > 0) {
        double width_height_ratio = det.green_width / det.green_height;
        det.valid_aspect_ratio = (width_height_ratio >= ASPECT_RATIO_MIN) &&
                                 (width_height_ratio <= ASPECT_RATIO_MAX);
    }

    /* --- Pillar Hypothesis --- */
    det.is_likely_pillar = false;
    
    // Green but almost no black
    if (det.has_green && det.black_pixels_total < min_black_total) {
        det.is_likely_pillar = true;
    }
    
    if (det.has_green && !det.has_black_below) {
        if (det.aspect_ratio > PILLAR_ASPECT_RATIO_MIN) {
            det.is_likely_pillar = true;
        }
        
        if (det.height_ratio > PILLAR_HEIGHT_MIN) {
            det.is_likely_pillar = true;
        }
    }
    
    // Solid wheel data means NOT a pillar
    if (det.has_black_below && det.wheels_below_chassis) {
        det.is_likely_pillar = false;
    }

    /* --- Confidence Score --- */
    double confidence = 0.0;

    if (det.has_green) confidence += 0.15;
    
    if (det.has_wheels) confidence += 0.15;
    if (det.has_black_below) confidence += 0.25;
    if (det.wheels_below_chassis) confidence += 0.20;
    if (det.wheels_on_sides) confidence += 0.10;
    
    if (det.black_pixels_bottom_third >= MIN_BLACK_PIXELS_CLOSE) {
        confidence += 0.10;
    }
    
    if (det.valid_aspect_ratio) confidence += 0.05;
    
    // Penalties for looking like a pillar
    if (det.is_likely_pillar) {
        confidence -= 0.50;
    }
    
    if (!det.has_black_below && det.has_green) {
        confidence -= 0.20;
    }
    
    if (det.black_pixels_total == 0 && det.has_green) {
        confidence -= 0.30;
    }

    det.confidence = clamp(confidence, 0.0, 1.0);

    /* --- Final Vehicle Decision --- */
    if (det.is_likely_pillar) {
        det.is_vehicle = false;
    } else if (det.has_green && det.has_black_below) {
        det.is_vehicle = (det.confidence >= CONFIDENCE_MEDIUM);
    } else if (det.is_very_close && det.has_green && det.black_pixels_total >= MIN_BLACK_PIXELS_CLOSE) {
        det.is_vehicle = true;
    } else {
        det.is_vehicle = false;
    }

    /* Set the rejection string */
    if (!det.is_vehicle) {
        if (det.black_pixels_total == 0 && det.has_green) {
            strcpy(det.rejection_reason, "PILLAR - Zero black pixels");
            pillar_rejections++;
        } else if (det.is_likely_pillar) {
            sprintf(det.rejection_reason, "PILLAR - Insufficient black (total: %d)", 
                    det.black_pixels_total);
            pillar_rejections++;
        } else if (!det.has_green) {
            strcpy(det.rejection_reason, "No green chassis");
        } else if (!det.has_black_below) {
            sprintf(det.rejection_reason, "No black below green (below: %d)", 
                    det.black_pixels_below);
        } else {
            sprintf(det.rejection_reason, "Low confidence: %.0f%%", det.confidence * 100);
        }
    }

    return det;
}

/* ============================================
   STATE BEHAVIORS
   ============================================ */

void do_patrol(void) {
    scan_timer++;

    if (scan_timer >= SCAN_INTERVAL) {
        current_state = STATE_SCANNING;
        scan_counter = 0;
        scan_timer = 0;
        return;
    }

    if (should_avoid_obstacle()) {
        obstacles.return_state = STATE_PATROL;
        obstacles.avoid_timer = OBSTACLE_MEMORY_FRAMES;
        current_state = STATE_AVOIDING;
        obstacle_avoidances++;
        return;
    }

    if (obstacles.any_obstacle) {
        calculate_braitenberg_speeds(&left_speed, &right_speed);
    } else {
        left_speed = CRUISING_SPEED;
        right_speed = CRUISING_SPEED;
    }

    set_patrol_lights();
}

void do_scanning(void) {
    scan_counter++;

    if (!target_lock.is_locked && obstacles.obstacle_front) {
        obstacles.return_state = STATE_SCANNING;
        obstacles.avoid_timer = OBSTACLE_MEMORY_FRAMES / 2;
        current_state = STATE_AVOIDING;
        return;
    }

    left_speed = -MAX_SPEED * 0.4;
    right_speed = MAX_SPEED * 0.4;

    int led_index = (scan_counter / 5) % 8;
    lights_out();
    if (leds[led_index]) wb_led_set(leds[led_index], 1);

    if (scan_counter >= SCAN_DURATION) {
        current_state = STATE_PATROL;
    }
}

void do_validating(void) {
    validation_counter++;

    if (!target_lock.is_locked && obstacles.obstacle_front) {
        obstacles.return_state = STATE_VALIDATING;
        obstacles.avoid_timer = OBSTACLE_MEMORY_FRAMES / 2;
        current_state = STATE_AVOIDING;
        return;
    }

    left_speed = CRUISING_SPEED * 0.3;
    right_speed = CRUISING_SPEED * 0.3;

    flash_validating_lights();

    if (validation_counter >= 10) { 
        if (detection.is_vehicle && detection.confidence >= CONFIDENCE_MEDIUM) {
            bool wheels_seen = detection.has_black_below || 
                              (detection.black_pixels_below >= MIN_BLACK_BELOW_CLOSE);
            
            acquire_target_lock(detection.confidence,
                                detection.green_center_x,
                                detection.green_center_y,
                                wheels_seen);
            current_state = STATE_PURSUIT;
            validated_vehicle_detections++;
            
            printf("[VALIDATE] EV confirmed! Wheels: %s (black: %d)\n",
                   wheels_seen ? "YES" : "NO", detection.black_pixels_total);
        } else {
            current_state = STATE_PATROL;
            false_positive_rejections++;
            
            printf("[VALIDATE] REJECTED: %s\n", detection.rejection_reason);
        }
        validation_counter = 0;
    }
}

void do_pursuit(void) {
    bool can_see_green = (detection.green_pixels >= MIN_GREEN_FOR_TRACKING);

    update_target_lock(can_see_green, detection.green_center_x, detection.green_center_y);

    if (check_wheel_revalidation()) {
        current_state = STATE_PATROL;
        return;
    }

    if (!is_target_lock_valid()) {
        current_state = STATE_PATROL;
        return;
    }

    double track_x = can_see_green ? detection.green_center_x : target_lock.last_seen_x;
    double error_x = (track_x - (cam_width / 2.0)) / (cam_width / 2.0); 

    double front_reading = (obstacles.front_left + obstacles.front_right) / 2.0;
    
    if (front_reading > TARGET_STOP_DIST) {
        if (detection.black_pixels_total > 0 || target_lock.wheels_confirmed) {
            printf("[PURSUIT] *** CONTACT / STOPPING RANGE ***\n"); 
            current_state = STATE_ALARM_BLOCK;
            return;
        } else {
            printf("[PURSUIT] Contact but ZERO black - this is a PILLAR!\n");
            release_target_lock("contacted suspected pillar (no black pixels)");
            current_state = STATE_PATROL;
            return;
        }
    }

    if (detection.green_pixels > CLOSE_THRESHOLD) {
        current_state = STATE_APPROACHING;
        return;
    }

    double turn_adjustment = error_x * TURN_SENSITIVITY;
    double base_speed = PURSUIT_SPEED * (1.0 - fabs(error_x) * 0.3); 

    left_speed = base_speed + turn_adjustment;
    right_speed = base_speed - turn_adjustment;

    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED);
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED);

    if (target_lock.wheels_confirmed) {
        flash_confirmed_target();
    } else {
        set_locked_lights();
    }
}

void do_approaching(void) {
    bool can_see_green = (detection.green_pixels >= MIN_GREEN_FOR_TRACKING);

    update_target_lock(can_see_green, detection.green_center_x, detection.green_center_y);

    if (check_wheel_revalidation()) {
        current_state = STATE_PATROL;
        return;
    }

    if (!is_target_lock_valid()) {
        current_state = STATE_PATROL;
        return;
    }

    double track_x = can_see_green ? detection.green_center_x : target_lock.last_seen_x;
    double error_x = (track_x - (cam_width / 2.0)) / (cam_width / 2.0);

    double front_reading = (obstacles.front_left + obstacles.front_right) / 2.0;
    if (front_reading > TARGET_STOP_DIST) {
        if (detection.black_pixels_total > 0 || target_lock.wheels_confirmed) {
            printf("[APPROACHING] *** EV BLOCKED (close range) ***\n");
            current_state = STATE_ALARM_BLOCK;
            return;
        } else {
            printf("[APPROACHING] Contact but ZERO black - PILLAR!\n");
            release_target_lock("approached pillar (no black pixels)");
            current_state = STATE_PATROL;
            return;
        }
    }

    double approach_speed = PURSUIT_SPEED * 0.5; 
    double turn_adjustment = error_x * TURN_SENSITIVITY * 0.7;

    left_speed = approach_speed + turn_adjustment;
    right_speed = approach_speed - turn_adjustment;

    left_speed = clamp(left_speed, -MAX_SPEED * 0.5, MAX_SPEED * 0.5);
    right_speed = clamp(right_speed, -MAX_SPEED * 0.5, MAX_SPEED * 0.5);

    pulse_approaching_lights();
}

void do_alarm_block(void) {
    left_speed = 0.0;
    right_speed = 0.0;

    // Fast alternating flash for ALARM
    int time_ms = (int)(wb_robot_get_time() * 1000);
    int state = (time_ms / 200) % 2;
    for (int i = 0; i < 10; i++) {
        if (leds[i]) wb_led_set(leds[i], state);
    }

    // Log the event once
    if (fp && !alarm_logged) {
        double pursuit_duration = wb_robot_get_time() - target_lock.lock_time;
        fprintf(fp, "%.2f,ALARM,%d,%.0f%%,%.1fs,wheels:%s,black:%d\n",
                wb_robot_get_time(),
                detection.green_pixels,
                target_lock.lock_confidence * 100,
                pursuit_duration,
                target_lock.wheels_confirmed ? "YES" : "NO",
                detection.black_pixels_total);
        fflush(fp);
        alarm_logged = 1;

        printf("[ALARM] *** EV BLOCKED/FOUND *** (pursuit duration: %.1fs)\n", pursuit_duration);
    }

    // Check if the target has moved away
    double front_reading = (obstacles.front_left + obstacles.front_right) / 2.0;
    if (front_reading < TARGET_STOP_DIST * 0.3) {
        lost_target_counter++;
        if (lost_target_counter > LOST_TARGET_TIMEOUT * 2) {
            release_target_lock("EV moved away from contact range");
            current_state = STATE_PATROL;
            alarm_logged = 0;
        }
    } else {
        lost_target_counter = 0;
    }
}

void do_avoiding(void) {
    if (target_lock.is_locked) {
        current_state = STATE_PURSUIT;
        return;
    }

    obstacles.avoid_timer--;

    if (obstacles.avoid_timer <= 0 && !obstacles.obstacle_front) {
        current_state = obstacles.return_state;
        return;
    }

    // Hard turn/backup for immediate front danger
    if (obstacles.front_left > OBSTACLE_THRESHOLD_HIGH ||
        obstacles.front_right > OBSTACLE_THRESHOLD_HIGH) {

        if (obstacles.avoid_direction >= 0) { // Turn right
            left_speed = AVOID_TURN_SPEED;
            right_speed = -AVOID_TURN_SPEED * 0.5;
        } else { // Turn left
            left_speed = -AVOID_TURN_SPEED * 0.5;
            right_speed = AVOID_TURN_SPEED;
        }

        obstacles.avoid_timer = OBSTACLE_MEMORY_FRAMES;
    }
    // Backup for high intensity if not directly front
    else if (obstacles.obstacle_intensity > OBSTACLE_THRESHOLD_HIGH * 1.5) {
        if (obstacles.avoid_direction >= 0) { // Backup and turn right
            left_speed = AVOID_BACKUP_SPEED * 0.5;
            right_speed = AVOID_BACKUP_SPEED;
        } else { // Backup and turn left
            left_speed = AVOID_BACKUP_SPEED;
            right_speed = AVOID_BACKUP_SPEED * 0.5;
        }
    }
    else {
        calculate_braitenberg_speeds(&left_speed, &right_speed);
    }

    show_avoiding_lights();
}

/* ============================================
   LOGGING / MAIN LOOP
   ============================================ */
void log_detection_details(double time) {
    if (!detection_log) return;

    fprintf(detection_log,
            "%.2f,%d,%d,%d,%d,%d,%d,%.0f,%.2f,%s,%s,%s\n",
            time,
            detection.green_pixels,
            detection.black_pixels_total,
            detection.black_pixels_below,
            detection.black_pixels_left,
            detection.black_pixels_right,
            target_lock.zero_black_count,
            detection.confidence * 100,
            detection.aspect_ratio,
            detection.is_likely_pillar ? "PILLAR" : "NOT_PILLAR",
            detection.is_vehicle ? "VEHICLE" : "REJECTED",
            target_lock.is_locked ? 
                (target_lock.wheels_confirmed ? "LOCK+WHL" : "LOCKED") : "UNLOCKED");
    fflush(detection_log);
}

void print_detection_summary(void) {
    printf("\n\n========== SESSION SUMMARY ==========\n");
    printf("Total green detections seen: %d\n", total_green_detections);
    printf("Confirmed EV detections:     %d\n", validated_vehicle_detections);
    printf("-------------------------------------\n");
    printf("False positives rejected:    %d\n", false_positive_rejections);
    printf("Pillar rejections (total):   %d\n", pillar_rejections);
    printf("  - Wheel re-confirmations:  %d\n", wheel_revalidations);
    printf("  - No-wheel lock interrupts:%d\n", no_wheel_interrupts);
    printf("  - Zero-black force releases:%d\n", forced_releases);
    printf("-------------------------------------\n");
    printf("Obstacle avoidances:         %d\n", obstacle_avoidances);
    printf("=====================================\n");
}

int main(int argc, char **argv) {
    wb_robot_init();
    init_devices();

    fp = fopen("patrol_data.csv", "w");
    if (fp) {
        fprintf(fp, "Time,Event,GreenPx,Confidence,Duration,Wheels,BlackPx\n");
    }

    detection_log = fopen("detection_analysis.csv", "w");
    if (detection_log) {
        fprintf(detection_log,
                "Time,GreenPx,BlackPx,BlackBelow,BlackL,BlackR,ZeroCount,Confidence,AspectRatio,PillarCheck,Result,LockStatus\n");
    }

    printf("Pillar rejection settings:\n");
    printf("  Zero-black force release: %d checks\n", ZERO_BLACK_FORCE_RELEASE);
    printf("  No-wheels force release:  %d checks\n\n", NO_WHEELS_FORCE_RELEASE_COUNT);

    while (wb_robot_step(TIME_STEP) != -1) {
        step_counter++;
        double current_time = wb_robot_get_time();

        obstacles = read_obstacle_sensors();
        detection = process_camera_advanced();

        if (detection.has_green) total_green_detections++;

        previous_state = current_state;

        /* --- State Transition Logic --- */
        if (!target_lock.is_locked && current_state != STATE_AVOIDING) {
            bool promising = detection.has_green && 
                            detection.has_black_below &&
                            !detection.is_likely_pillar;
            
            if (promising && current_state == STATE_PATROL) {
                current_state = STATE_VALIDATING;
                validation_counter = 0;
            }

            if (detection.is_vehicle && current_state == STATE_SCANNING) {
                bool wheels_seen = detection.has_black_below;
                acquire_target_lock(detection.confidence,
                                   detection.green_center_x,
                                   detection.green_center_y,
                                   wheels_seen);
                current_state = STATE_PURSUIT;
            }
        }

        /* --- Execute Current State Behavior --- */
        switch (current_state) {
            case STATE_PATROL:
                do_patrol();
                break;
            case STATE_SCANNING:
                do_scanning();
                break;
            case STATE_VALIDATING:
                do_validating();
                if (step_counter % 5 == 0) {
                    log_detection_details(current_time);
                }
                break;
            case STATE_PURSUIT:
                do_pursuit();
                break;
            case STATE_APPROACHING:
                do_approaching();
                break;
            case STATE_ALARM_BLOCK:
                do_alarm_block();
                break;
            case STATE_AVOIDING:
                if (target_lock.is_locked) {
                    current_state = STATE_PURSUIT; 
                } else {
                    do_avoiding();
                }
                break;
            default:
                current_state = STATE_PATROL;
                break;
        }

        /* --- State Change Logging --- */
        if (current_state != previous_state) {
            printf("[%.1fs] State change: %s -> %s",
                   current_time, state_names[previous_state], state_names[current_state]);

            switch (current_state) {
                case STATE_VALIDATING:
                    printf(" (G:%d B:%d)", 
                           detection.green_pixels, detection.black_pixels_total);
                    break;
                case STATE_PURSUIT:
                    printf(" (LOCKED %.0f%%)", target_lock.lock_confidence * 100);
                    break;
                case STATE_ALARM_BLOCK:
                    printf(" (*** TARGET FOUND/BLOCKED ***)");
                    break;
                case STATE_PATROL:
                    if (previous_state == STATE_VALIDATING) {
                        printf(" (rejected: %s)", detection.rejection_reason);
                    } else if (previous_state == STATE_PURSUIT || previous_state == STATE_APPROACHING) {
                        printf(" (lock released: target lost/rejected)");
                    }
                    break;
                default:
                    break;
            }
            printf("\n");
        }

        if (current_state != STATE_ALARM_BLOCK) alarm_logged = 0;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);

        /* Periodic status update */
        if (step_counter % STATUS_LOG_INTERVAL == 0) {
            const char* lock_str = "";
            if (target_lock.is_locked) {
                lock_str = target_lock.wheels_confirmed ? "LOCK+WHL" : "LOCKED";
            }
            
            const char* det_str = "";
            if (detection.is_likely_pillar) {
                det_str = "PILLAR!";
            } else if (detection.has_black_below) {
                det_str = "WHEELS";
            } else if (detection.black_pixels_total == 0 && detection.has_green) {
                det_str = "NO-BLACK";
            }
            
            printf("[%.0fs] %-10s | Green:%5d Black:%3d (below:%2d) ZeroCnt:%d | %s %s\n",
                   current_time,
                   state_names[current_state],
                   detection.green_pixels,
                   detection.black_pixels_total,
                   detection.black_pixels_below,
                   target_lock.zero_black_count,
                   lock_str,
                   det_str);
        }
    }

    print_detection_summary();

    if (fp) fclose(fp);
    if (detection_log) fclose(detection_log);
    wb_robot_cleanup();
    return 0;
}

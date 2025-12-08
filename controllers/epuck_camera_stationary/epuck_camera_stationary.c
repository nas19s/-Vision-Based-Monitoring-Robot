#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/led.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#define TIME_STEP 32

/* Color detection thresholds for the EV marker */
#define PURE_GREEN_R_MAX 100
#define PURE_GREEN_G_MIN 150
#define PURE_GREEN_B_MAX 100
#define GREEN_DOMINANCE_RATIO 1.5
#define EV_DETECTION_THRESHOLD 10

/* Communication */
#define CAMERA_CHANNEL 2

/* Device Tags */
static WbDeviceTag receiver = NULL;
static WbDeviceTag emitter = NULL;
static WbDeviceTag leds[10] = {0};
static WbDeviceTag camera = NULL;

/* Camera properties */
static int camera_width = 0, camera_height = 0;

/* Detection state */
static bool camera_ev_visible = false;
static bool previous_ev_visible = false;
static int camera_pure_green_pixels = 0;
static int detected_x = -1, detected_y = -1;

/* Zone alarm from supervisor (for Blue LED) */
static bool vpe_zone_alarm = false;

/* LED flash state */
static int flash_counter = 0;

/* Data Logging File Pointer */
FILE *h1_log_file = NULL;

/* --- LED Control --- */

static void all_lights_out(void) {
  for (int i = 0; i < 10; i++) {
    if (leds[i]) wb_led_set(leds[i], 0);
  }
}

static void all_lights_on(void) {
  for (int i = 0; i < 10; i++) {
    if (leds[i]) wb_led_set(leds[i], 1);
  }
}

static void update_leds(void) {
  flash_counter++;
  all_lights_out();
  
  if (camera_ev_visible && vpe_zone_alarm) {
    /* BOTH: Very fast strobe */
    if ((flash_counter / 2) % 2 == 0) {
      all_lights_on();
    }
  } else if (camera_ev_visible) {
    /* Camera detection: Fast blink */
    if ((flash_counter / 4) % 2 == 0) {
      all_lights_on();
    }
  } else if (vpe_zone_alarm) {
    /* Zone alarm only: Slow blink */
    if ((flash_counter / 16) % 2 == 0) {
      for (int i = 0; i < 5; i++) {
        if (leds[i]) wb_led_set(leds[i], 1);
      }
    }
  } else {
    /* Normal: Just power LED */
    if (leds[0]) wb_led_set(leds[0], 1);
  }
}

/* --- Initialization --- */

static void init_devices(void) {
  
  receiver = wb_robot_get_device("receiver");
  if (receiver) {
    wb_receiver_enable(receiver, TIME_STEP);
    printf("[CAM2] Receiver: OK\n");
  } else {
    printf("[CAM2] Receiver: NOT FOUND\n");
  }

  
  emitter = wb_robot_get_device("cam2_emitter");  
  if (emitter) {
    wb_emitter_set_channel(emitter, CAMERA_CHANNEL);
    printf("[CAM2] Emitter: OK (channel %d)\n", CAMERA_CHANNEL);
  } else {
    printf("[CAM2] Emitter: NOT FOUND - Blue light won't work!\n");
  }

  camera = wb_robot_get_device("camera");
  if (camera) {
    wb_camera_enable(camera, TIME_STEP);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    printf("[CAM2] Camera: OK (%dx%d)\n", camera_width, camera_height);
  } else {
    printf("[CAM2] Camera: NOT FOUND!\n");
  }

  char led_name[8];
  int leds_found = 0;
  for (int i = 0; i < 10; i++) {
    sprintf(led_name, "led%d", i);
    leds[i] = wb_robot_get_device(led_name);
    if (leds[i]) leds_found++;
  }
  printf("[CAM2] LEDs: %d/10 found\n", leds_found);

  printf("[CAM2] Testing LEDs...\n");
  all_lights_on();
  wb_robot_step(300);
  all_lights_out();
  wb_robot_step(200);

  /* Initialize H1 Data Logging */
  h1_log_file = fopen("h1_stationary_data.csv", "w");
  if (h1_log_file) {
      // Header: Timestamp, Pixels Seen, Alarm Signal (1/0)
      fprintf(h1_log_file, "Time,Detected_Pixels,Signal_Sent\n");
      printf("[CAM2] Data Logging: ENABLED (h1_stationary_data.csv)\n");
  } else {
      printf("[CAM2] Data Logging: FAILED (Could not open file)\n");
  }

  printf("[CAM2] Initialization complete!\n\n");
}

/* --- Camera Processing --- */

static bool process_camera_for_ev(void) {
  if (!camera) return false;

  const unsigned char* image = wb_camera_get_image(camera);
  if (!image) return false;

  int pure_count = 0;
  int sum_x = 0, sum_y = 0;
  int max_green = 0;

  for (int y = 0; y < camera_height; y++) {
    for (int x = 0; x < camera_width; x++) {
      int r = wb_camera_image_get_red(image, camera_width, x, y);
      int g = wb_camera_image_get_green(image, camera_width, x, y);
      int b = wb_camera_image_get_blue(image, camera_width, x, y);

      if (g > max_green) max_green = g;

      bool is_pure_green =
        r < PURE_GREEN_R_MAX &&
        g > PURE_GREEN_G_MIN &&
        b < PURE_GREEN_B_MAX &&
        g > r * GREEN_DOMINANCE_RATIO &&
        g > b * GREEN_DOMINANCE_RATIO;

      if (is_pure_green) {
        pure_count++;
        sum_x += x;
        sum_y += y;
      }
    }
  }

  camera_pure_green_pixels = pure_count;

  if (pure_count >= EV_DETECTION_THRESHOLD) {
    camera_ev_visible = true;
    detected_x = sum_x / pure_count;
    detected_y = sum_y / pure_count;
  } else {
    camera_ev_visible = false;
    detected_x = detected_y = -1;
  }

  /* Debug output */
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter % 62 == 0) {
    printf("[CAM2] Pixels:%4d | MaxG:%3d | Visible:%s\n",
           pure_count, max_green, camera_ev_visible ? "YES" : "NO");
  }

  return camera_ev_visible;
}

/* --- Send Detection Status to Supervisor --- */

static void send_detection_status(void) {
  if (!emitter) return;

  unsigned char msg = camera_ev_visible ? 1 : 0;
  wb_emitter_send(emitter, &msg, sizeof(msg));
}

/* --- Main Loop --- */

int main(int argc, char** argv) {
  wb_robot_init();
  init_devices();

  printf("========================================\n");
  printf("E-puck CAM2: Stationary Camera Monitor\n");
  printf("========================================\n");
  printf("MODE: Stationary (no movement)\n");
  printf("FUNCTION: Detect green EV, signal supervisor\n");
  printf("BLUE LIGHT: Controlled by supervisor based on\n");
  printf("            detection signals from this camera\n");
  printf("LOGGING: Recording H1 data to CSV\n");
  printf("========================================\n\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    double current_time = wb_robot_get_time();

    /* Receive zone alarm from supervisor */
    while (receiver && wb_receiver_get_queue_length(receiver) > 0) {
      const unsigned char* data = wb_receiver_get_data(receiver);
      bool new_zone_alarm = (data[0] == 1);

      if (new_zone_alarm != vpe_zone_alarm) {
        printf("[CAM2] Zone alarm: %s\n", new_zone_alarm ? "ON" : "OFF");
      }

      vpe_zone_alarm = new_zone_alarm;
      wb_receiver_next_packet(receiver);
    }

    /* Process camera */
    previous_ev_visible = camera_ev_visible;
    process_camera_for_ev();

    /* Report detection changes */
    if (camera_ev_visible && !previous_ev_visible) {
      printf("\n[CAM2] *** GREEN EV DETECTED! ***\n");
      printf("       Pixels: %d at (%d, %d)\n", 
             camera_pure_green_pixels, detected_x, detected_y);
      printf("       -> Sending signal to supervisor for BLUE light\n\n");
    } else if (!camera_ev_visible && previous_ev_visible) {
      printf("[CAM2] Green EV lost.\n");
      printf("       -> Sending clear signal to supervisor\n\n");
    }

    /* Send detection status to supervisor EVERY step */
    send_detection_status();

    /* Update LEDs */
    update_leds();

    /* Periodic status */
    static int print_counter = 0;
    print_counter++;
    if (print_counter % 93 == 0) {
      printf("[CAM2] t=%.1f | Px:%4d | EV:%s | Zone:%s\n",
             current_time, camera_pure_green_pixels,
             camera_ev_visible ? "YES" : "NO",
             vpe_zone_alarm ? "YES" : "NO");
    }

    /* LOG DATA FOR HYPOTHESIS H1 */
    if (h1_log_file) {
        fprintf(h1_log_file, "%f,%d,%d\n", 
                current_time, 
                camera_pure_green_pixels, 
                camera_ev_visible ? 1 : 0);
    }
  }

  /* Cleanup */
  if (h1_log_file) fclose(h1_log_file);
  wb_robot_cleanup();
  return 0;
}

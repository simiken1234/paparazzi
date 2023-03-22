/*
 * Copyright (C) Kirk Scheper <kirkscheper@gmail.com>
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider_guided.c"
 * @author Kirk Scheper
 * This module is an example module for the course AE4317 Autonomous Flight of Micro Air Vehicles at the TU Delft.
 * This module is used in combination with a color filter (cv_detect_color_object) and the guided mode of the autopilot.
 * The avoidance strategy is to simply count the total number of orange pixels. When above a certain percentage threshold,
 * (given by color_count_frac) we assume that there is an obstacle and we turn.
 *
 * The color filter settings are set using the cv_detect_color_object. This module can run multiple filters simultaneously
 * so you have to define which filter to use with the ORANGE_AVOIDER_VISUAL_DETECTION_ID setting.
 * This module differs from the simpler orange_avoider.xml in that this is flown in guided mode. This flight mode is
 * less dependent on a global positioning estimate as witht the navigation mode. This module can be used with a simple
 * speed estimate rather than a global position.
 *
 * Here we also need to use our onboard sensors to stay inside of the cyberzoo and not collide with the nets. For this
 * we employ a simple color detector, similar to the orange poles but for green to detect the floor. When the total amount
 * of green drops below a given threshold (given by floor_count_frac) we assume we are near the edge of the zoo and turn
 * around. The color detection is done by the cv_detect_color_object module, use the FLOOR_VISUAL_DETECTION_ID setting to
 * define which filter to use.
 */

#include "modules/orange_avoider/orange_avoider_guided.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "generated/airframe.h"
#include "state.h"
#include "modules/core/abi.h"
#include <stdio.h>
#include <time.h>

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider_guided->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t chooseRandomIncrementAvoidance(void);
uint8_t flyToOrigin(void);
uint8_t flyFromOrigin(void);
uint8_t flyCircle(void);
uint8_t flyBetterCircle(void);
uint8_t flyWorseCircle(void);
float perpToCenter(void);
uint8_t tunePerpSpeedConst(void);
float getGain(void);
float linearInterpolation(float x, float y, float z_xlo_ylo, float z_xhi_ylo, float z_xlo_yhi, float z_xhi_yhi);

enum navigation_state_t {
  SAFE,
  OBSTACLE_FOUND,
  SEARCH_FOR_SAFE_HEADING,
  OUT_OF_BOUNDS,
  REENTER_ARENA,
  CIRCLE
};

// define settings
float oag_color_count_frac = 0.18f;       // obstacle detection threshold as a fraction of total of image
float oag_floor_count_frac = 0.05f;       // floor detection threshold as a fraction of total of image
float oag_max_speed = 2.5f;               // max flight speed [m/s]
float oag_heading_rate = RadOfDeg(20.f);  // heading change setpoint for avoidance [rad/s]
float circle_radius = 2.5f;               // radius of circle of flight [m]
float perp_speed_const = 2.2f;            // constant for perpendicular-to-body speed in circular motion

// define and initialise global variables
enum navigation_state_t navigation_state = SEARCH_FOR_SAFE_HEADING;   // current state in state machine
int32_t color_count = 0;                // orange color count from color filter for obstacle detection
int32_t floor_count = 0;                // green color count from color filter for floor detection
int32_t floor_centroid = 0;             // floor detector centroid in y direction (along the horizon)
float avoidance_heading_direction = 0;  // heading change direction for avoidance [rad/s]
int16_t obstacle_free_confidence = 0;   // a measure of how certain we are that the way ahead if safe.
float dist_to_center = 0;               // distance to center of arena
bool in_circle = false;                 // whether or not the drone is in the circle
float error_const_threshold = 0.05f;    // threshold for error consistency in circle flight
float error_min_threshold = 0.03f;      // threshold for minimum error which warrant tuneing in circle flight
float error_memory[7] = {0};           // array to store the last errors for tuning of perp. speed in circular flight
float perp_speed_increment = 0.05f;      // increment for perp. speed in circular flight
float gain_table[7][8] = {
  {2.1, 1.65, 1.5, 1.3, 1.15, 1.1, 1.05, 1.0},
  {4.0, 2.1, 1.75, 1.65, 1.55, 1.45, 1.35, 1.30},
  {4.00, 4.00, 2.15, 1.85, 1.70, 1.65, 1.60, 1.55},
  {4.00, 4.00, 3.45, 2.25, 1.95, 1.75, 1.70, 1.65},
  {4.00, 4.00, 4.00, 3.10, 2.20, 1.95, 1.80, 1.75},
  {4.00, 4.00, 4.00, 4.00, 2.75, 2.15, 1.95, 1.85},
  {4.00, 4.00, 4.00, 4.00, 4.00, 2.70, 2.15, 2.05},
};                                    // table of gains for perp. speed in circular flight x_coord = radius, y_coord = speed, 0.5 increments

const int16_t max_trajectory_confidence = 5;  // number of consecutive negative object detections to be sure we are obstacle free

// This call back will be used to receive the color count from the orange detector
#ifndef ORANGE_AVOIDER_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define ORANGE_AVOIDER_VISUAL_DETECTION_ID to the orange filter
#error Please define ORANGE_AVOIDER_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event color_detection_ev;
static void color_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t __attribute__((unused)) pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  color_count = quality;
}

#ifndef FLOOR_VISUAL_DETECTION_ID
#error This module requires two color filters, as such you have to define FLOOR_VISUAL_DETECTION_ID to the orange filter
#error Please define FLOOR_VISUAL_DETECTION_ID to be COLOR_OBJECT_DETECTION1_ID or COLOR_OBJECT_DETECTION2_ID in your airframe
#endif
static abi_event floor_detection_ev;
static void floor_detection_cb(uint8_t __attribute__((unused)) sender_id,
                               int16_t __attribute__((unused)) pixel_x, int16_t pixel_y,
                               int16_t __attribute__((unused)) pixel_width, int16_t __attribute__((unused)) pixel_height,
                               int32_t quality, int16_t __attribute__((unused)) extra)
{
  floor_count = quality;
  floor_centroid = pixel_y;
}

/*
 * Initialisation function
 */
void orange_avoider_guided_init(void)
{ 
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();

  // bind our colorfilter callbacks to receive the color filter outputs
  AbiBindMsgVISUAL_DETECTION(ORANGE_AVOIDER_VISUAL_DETECTION_ID, &color_detection_ev, color_detection_cb);
  AbiBindMsgVISUAL_DETECTION(FLOOR_VISUAL_DETECTION_ID, &floor_detection_ev, floor_detection_cb);
}

/*
 * Function that checks it is safe to move forwards, and then sets a forward velocity setpoint or changes the heading
 */
void orange_avoider_guided_periodic(void)
{
  // Only run the mudule if we are in the correct flight mode
  if (guidance_h.mode != GUIDANCE_H_MODE_GUIDED) {
    navigation_state = SEARCH_FOR_SAFE_HEADING;
    obstacle_free_confidence = 0;
    return;
  }

  // compute current color thresholds
  int32_t color_count_threshold = oag_color_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  int32_t floor_count_threshold = oag_floor_count_frac * front_camera.output_size.w * front_camera.output_size.h;
  float floor_centroid_frac = floor_centroid / (float)front_camera.output_size.h / 2.f;

  VERBOSE_PRINT("Color_count: %d  threshold: %d state: %d \n", color_count, color_count_threshold, navigation_state);
  VERBOSE_PRINT("Floor count: %d, threshold: %d\n", floor_count, floor_count_threshold);
  VERBOSE_PRINT("Floor centroid: %f\n", floor_centroid_frac);
  VERBOSE_PRINT("Current circle radius: %f\n", circle_radius);

  // update our safe confidence using color threshold
  if(color_count < color_count_threshold){
    obstacle_free_confidence++;
  } else {
    obstacle_free_confidence -= 2;  // be more cautious with positive obstacle detections
  }

  // bound obstacle_free_confidence
  Bound(obstacle_free_confidence, 0, max_trajectory_confidence);

  float speed_sp = fminf(oag_max_speed, 0.2f * obstacle_free_confidence);

  switch (navigation_state){
    case SAFE:
      VERBOSE_PRINT("STATE: SAFE\n");
      if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
        navigation_state = OUT_OF_BOUNDS;
      } else if (obstacle_free_confidence == 0){
        navigation_state = OBSTACLE_FOUND;
      } else {
        guidance_h_set_body_vel(speed_sp, 0);
      }

      break;
    case OBSTACLE_FOUND:
      VERBOSE_PRINT("STATE: OBSTACLE_FOUND\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // randomly select new search direction
      chooseRandomIncrementAvoidance();

      navigation_state = SEARCH_FOR_SAFE_HEADING;

      break;
    case SEARCH_FOR_SAFE_HEADING:
      VERBOSE_PRINT("STATE: SEARCH_FOR_SAFE_HEADING\n");
      guidance_h_set_heading_rate(avoidance_heading_direction * oag_heading_rate);

      // make sure we have a couple of good readings before declaring the way safe
      if (obstacle_free_confidence >= 2){
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);
        navigation_state = CIRCLE;
      }
      break;
    case OUT_OF_BOUNDS:
    VERBOSE_PRINT("STATE: OUT_OF_BOUNDS\n");
      // stop
      guidance_h_set_body_vel(0, 0);

      // start turn back into arena
      guidance_h_set_heading_rate(avoidance_heading_direction * RadOfDeg(15));

      navigation_state = REENTER_ARENA;

      break;
    case REENTER_ARENA:
    VERBOSE_PRINT("STATE: REENTER_ARENA\n");
      // force floor center to opposite side of turn to head back into arena
      if (floor_count >= floor_count_threshold && avoidance_heading_direction * floor_centroid_frac >= 0.f){
        // return to heading mode
        guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi);

        // reset safe counter
        obstacle_free_confidence = 0;

        // ensure direction is safe before continuing
        navigation_state = CIRCLE;
      }
      break;
    case CIRCLE:
      // fly in a circle around the arena
      //if (floor_count < floor_count_threshold || fabsf(floor_centroid_frac) > 0.12){
      //  navigation_state = OUT_OF_BOUNDS;
      //} else if (obstacle_free_confidence == 0){
      //  navigation_state = OBSTACLE_FOUND;
      //} else {
        dist_to_center = sqrtf((stateGetPositionNed_f()->x * stateGetPositionNed_f()->x) + (stateGetPositionNed_f()->y * stateGetPositionNed_f()->y));
        VERBOSE_PRINT("Distance to center: %f\n", dist_to_center);
        //flyBetterCircle();
        flyWorseCircle();
        //VERBOSE_PRINT("X Ned_f: %f,  Y: %f\n", stateGetPositionNed_f()->x, stateGetPositionNed_f()->y);
        /*if (circle_radius - 0.5f > dist_to_center){
          in_circle = false;
          flyFromOrigin();
        }
        else if (circle_radius + 0.5f < dist_to_center){
          in_circle = false;
          flyToOrigin();
        }
        else{
          if (!in_circle){
            in_circle = true;
            VERBOSE_PRINT("Turning into circle\n");
            guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + 0.157);
          }
          flyCircle();
        }*/
      //}
      break;
    default:
      break;
  }
  return;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance(void)
{
  // Randomly choose CW or CCW avoiding direction
  if (rand() % 2 == 0) {
    avoidance_heading_direction = 1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  } else {
    avoidance_heading_direction = -1.f;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", avoidance_heading_direction * oag_heading_rate);
  }
  return false;
}

uint8_t flyToOrigin(void)
{
  VERBOSE_PRINT("Fly to origin");
  float angle = atan2f(stateGetPositionNed_f()->y, stateGetPositionNed_f()->x);
  guidance_h_set_heading(angle + 3.1415926);
  guidance_h_set_body_vel(oag_max_speed, 0);
  return false;
}

uint8_t flyFromOrigin(void)
{
  VERBOSE_PRINT("Fly from origin");
  float angle = atan2f(stateGetPositionNed_f()->y, stateGetPositionNed_f()->x);
  guidance_h_set_heading(angle);
  guidance_h_set_body_vel(oag_max_speed, 0);
  return false;
}

uint8_t flyCircle(void)
{
  VERBOSE_PRINT("Fly circle");
  guidance_h_set_heading_rate(6.2832 / ((6.2832 * circle_radius) / oag_max_speed));  // 6.2832 = 2pi
  guidance_h_set_body_vel(oag_max_speed, 0);
  return false;
}

uint8_t flyBetterCircle(void)
{
  VERBOSE_PRINT("Fly better circle. PerpSpeedConst: %f\n", perp_speed_const);
  float angle = perpToCenter();
  // Add offset to angle dependent on distance to center
  angle -= 0.157 * (1 - (dist_to_center / circle_radius));
  // Set exact heading to eliminate drift of center of circle
  guidance_h_set_heading(angle);
  // Body velocity X is the actual speed, Y needs to account for centrifugal force, which is proportional to radius and speed squared
  // Direct control of Y acceleration is impossible, so this has to be a tunable parameter
  // perp_speed_const = getGain();
  float vx = oag_max_speed;
  float vy = perp_speed_const * ((oag_max_speed * oag_max_speed) / (circle_radius * (circle_radius / dist_to_center)));
  float v_mult = oag_max_speed / sqrtf((vx * vx) + (vy * vy));
  guidance_h_set_body_vel(v_mult * vx, v_mult * vy);
  tunePerpSpeedConst();
  return false;
}

uint8_t flyWorseCircle(void)
{
  VERBOSE_PRINT("Fly better circle. PerpSpeedConst: %f\n", perp_speed_const);
  float angle = perpToCenter();
  
  // Add offset to angle dependent on distance to center
  angle -= 0.157 * (1 - (dist_to_center / circle_radius));

  // Increase heading
  guidance_h_set_heading(stateGetNedToBodyEulers_f()->psi + 5.0);

  // Body velocity X is the actual speed, Y needs to account for centrifugal force, which is proportional to radius and speed squared
  // Direct control of Y acceleration is impossible, so this has to be a tunable parameter
  float vx = oag_max_speed;
  float vy = perp_speed_const * ((oag_max_speed * oag_max_speed) / (circle_radius * (circle_radius / dist_to_center)));
  float v_mult = oag_max_speed / sqrtf((vx * vx) + (vy * vy));
  vx = v_mult * vx;
  vy = v_mult * vy;
  // Translate velocities from vy pointing towards center and vx perpendicular to NED coordinates
  float vx_ned = vx * cosf(angle) - vy * sinf(angle);
  float vy_ned = vx * sinf(angle) + vy * cosf(angle);

  guidance_h_set_vel(vx_ned, vy_ned);
  tunePerpSpeedConst();
  return false;
}

float perpToCenter(void)
{
  //Get angle perpendicular to line to center
  float angle = atan2f(stateGetPositionNed_f()->y, stateGetPositionNed_f()->x);
  angle += 1.5708;
  return angle;
}

uint8_t tunePerpSpeedConst(void)
{
  double error = dist_to_center - circle_radius;
  int error_memory_size = sizeof(error_memory) / sizeof(error_memory[0]);

    // Shift error memory to the right and add new error to the left
    for (int i = error_memory_size - 1; i > 0; i--) {
        error_memory[i] = error_memory[i-1];
    }
    error_memory[0] = error;

    // Check if errors are roughly constant
    bool is_constant = true;
    for (int i = 1; i < error_memory_size; i++) {
        if (fabs(error_memory[i] - error_memory[0]) > error_const_threshold) {
            is_constant = false;
            break;
        }
    }

    if (is_constant) {
      // Check if error is big enough to warrant a change
      if (fabs(error_memory[0]) > error_min_threshold) {

        // Increase/decrease perp_speed_const depending on whether the error is positive or negative
        if (dist_to_center > circle_radius) {
            perp_speed_const += perp_speed_increment;
        } else {
            perp_speed_const -= perp_speed_increment;
        }
        // Reset memory
        memset(error_memory, 0, sizeof(error_memory));
        error_memory[0] = 5.0;
      }
    }
}

float getGain(void)
{
  float vx = stateGetSpeedNed_f()->x;
  float vy = stateGetSpeedNed_f()->y;
  float v = sqrtf((vx * vx) + (vy * vy));

  float speed_step = 0.5f;
  int i_speed_lo = (int)(v / speed_step);
  float d_speed = v - (i_speed_lo * speed_step);
  if (i_speed_lo > 0) {
    i_speed_lo -= 1;
  }
  VERBOSE_PRINT("Speed: %f, speed_lo: %f\n", v, i_speed_lo * speed_step);

  float radius_step = 0.5f;
  int i_radius_lo = (int)(dist_to_center / radius_step);
  float d_rad = dist_to_center - (i_radius_lo * radius_step);
  if (i_radius_lo > 0) {
    i_radius_lo -= 1;
  }
  VERBOSE_PRINT("Radius: %f, radius_lo: %f\n", dist_to_center, i_radius_lo * radius_step);  

  float gain = linearInterpolation(d_rad, d_speed, 
    gain_table[i_radius_lo][i_speed_lo], 
    gain_table[i_radius_lo + 1][i_speed_lo], 
    gain_table[i_radius_lo][i_speed_lo + 1], 
    gain_table[i_radius_lo + 1][i_speed_lo + 1]);
  
  return gain;
}

float linearInterpolation(float x, float y, float z_xlo_ylo, float z_xhi_ylo, float z_xlo_yhi, float z_xhi_yhi)
{
  // Print all z values
  VERBOSE_PRINT("z_xlo_ylo: %f, z_xhi_ylo: %f, z_xlo_yhi: %f, z_xhi_yhi: %f\n", z_xlo_ylo, z_xhi_ylo, z_xlo_yhi, z_xhi_yhi);
  float x_lo = 0.0f;
  float x_hi = 0.5f;
  float y_lo = 0.0f;
  float y_hi = 0.5f;

  float z_xlo = z_xlo_ylo + (z_xlo_yhi - z_xlo_ylo) * (y - y_lo) / (y_hi - y_lo);
  float z_xhi = z_xhi_ylo + (z_xhi_yhi - z_xhi_ylo) * (y - y_lo) / (y_hi - y_lo);
  float z = z_xlo + (z_xhi - z_xlo) * (x - x_lo) / (x_hi - x_lo);
  return z;
}
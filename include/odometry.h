#pragma once

#include <math.h>
#include <Arduino.h>
#include "HAL.h"

#define SQR(x) ((x) * (x))
#define SGN(x) ( ( (x)<0 ) ? -1 : 1)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

extern float current_x;
extern float current_y;
extern float current_phi; // angle between x axis and the robot's center axis, in radians
extern float absolute_phi; // phi without modulo 2PI
extern int32_t previous_encoder_count_R;
extern int32_t previous_encoder_count_L;

void update_speed(float dt);
void update_position(float dt);
void reset_position();
void set_position(float current__x, float current__y, float current__phi, float absolute__phi);
float get_distance_to(float x, float y);
float get_arc_distance_to(float target_point_x, float target_point_y, float center_point_x, float center_point_y, float radius);
float get_angle_to(float x, float y);
float ticks_to_distance(int32_t ticks, float wheel_diameter);
float distance_to_ticks(int32_t distance, float wheel_diameter);
float rad_to_deg(float radian);
float deg_to_rad(float degrees);
float normalize_angle(float angle);
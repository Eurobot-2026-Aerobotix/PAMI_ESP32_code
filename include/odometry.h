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
extern int32_t previous_encoder_PCNT_count_R;
extern int32_t previous_encoder_PCNT_count_L;

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
float get_target_phi(float t,float x0, float y0,float x1, float y1,float x2, float y2,float x3, float y3);
float get_bezier_distance(float t ,float x0, float y0,float x1, float y1,float x2 , float y2 ,float x3, float y3);
void bezier(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float *x, float *y);
void bezier_derivative(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float *x, float *y);
float get_bezier_radius(float t,float x0, float y0,float x1, float y1, float x2, float y2,float x3, float y3);
double signed_distance_to_bezier(double px, double py, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double t);
float project_bezier(float px, float py, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3);
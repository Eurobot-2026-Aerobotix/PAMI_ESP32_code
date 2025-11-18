#pragma once

#include <math.h>
#include <Arduino.h>
#include "odometry.h"

enum Wheel_state
{
	DEAD_STOP,
	ACCELERATING,
	CONTROLLED_BREAK,
	EMERGENCY_BREAK,
	CRUISING // holding the same speed
};

enum Trajectory
{
	STRAIGHT,
	ROTATING,
	CIRCLE_CURVE,
	POLYNOMIAL_CURVE
};

extern enum Wheel_state wheel_state_R;
extern enum Wheel_state wheel_state_L;
extern enum Trajectory trajectory;
extern float ramping_acceleration; // !!! these accelerations mark the rate of change in velocity with respect to the distance traveled, not time!!!
extern float breaking_acceleration;
extern float emergency_breaking_acceleration;
extern float rotating_speed;
extern float cruise_speed_R; // the speed which the right wheel is accelerating towards
extern float cruise_speed_L;
extern float starting_phi;
extern float turning_radius;
extern int32_t previous_speed_control_time;

int move_forward(float distance, float speed);	// will return 0 if target has been reached successfully or 1 if it could not be reached
int move_backward(float distance, float speed); // will return 0 if target has been reached successfully or 1 if it could not be reached
float calculate_instant_target_speed_R_full(float x, float distance, float ramping__acceleration, float breaking__acceleration);
float calculate_instant_target_speed_L_full(float x, float distance, float ramping__acceleration, float breaking__acceleration);
float calculate_instant_target_speed_R(float x, float distance);
float calculate_instant_target_speed_L(float x, float distance);
float calculate_instant_target_phi(float x, float distance);
int rotate(float angle, int8_t is_radian);									   // is_radian should be set to 1 if the angle is expressed in radians and to 0 if in degrees
int orientate(float orientation, int8_t is_radian);							   // will return 0 if target has been reached successfully or 1 if it could not be reached
int move_forward_with_deviation(float distance, float deviation, float speed); // follows an arc to the target, that deviates at its max from the straight line "deviation" millimeters will deviate left if deviation is positive, will deviate right otherwise
int circle_curve(float target_x, float target_y, float target_phi, float speed);
int go_to(float x, float y, float speed); // will return 0 if target has been reached successfully or 1 if some error happened in rotation, 2 if some error happened in moving forward
void emergency_break();
void reset_speed_control_variables();
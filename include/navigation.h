#pragma once

#include <math.h>
#include <Arduino.h>
#include "odometry.h"

#define STOP_PWM_THRESHOLD   1000   // close to PWM_max (4499)
#define STOP_SPEED_THRESHOLD 50    // mm/s (robot almost stopped)

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
	POLYNOMIAL_CURVE,
	BEZIER_CURVE
};

enum Control_loop {
	DISACTIVATED, // blocks the main PID until the Raspberry activates the STM
	ACTIVATED,
	SPEED,
	POSITION
};

enum command {
	DELAY,
	GOTO,
	ORIENTATE,
	SET_POSITION,
	BEZIER,
	WAIT_FOR,
	BACKWARDS,
	PICKUP,
	PUTDOWN,
	WALKING_POSITION,
	PREPARE,
	LETGO,
	MOVE_FORWARD,
	ROTATE
};

extern enum Control_loop control_loop_state;
extern enum Wheel_state wheel_state_R;
extern enum Wheel_state wheel_state_L;
extern enum Trajectory trajectory;
extern float ramping_acceleration; // !!! these accelerations mark the rate of change in velocity with respect to the distance traveled, not time!!!
extern float breaking_acceleration;
extern float emergency_breaking_acceleration;
extern float acc_linearization_coef;
extern float rotating_speed;
extern float cruise_speed_R; // the speed which the right wheel is accelerating towards
extern float cruise_speed_L;
extern float starting_phi;
extern float turning_radius;
extern int32_t previous_speed_control_time;
extern int min_distance;


int move_forward(float distance, float speed, float target_x, float target_y, float direction);	// will return 0 if target has been reached successfully or 1 if it could not be reached
float calculate_instant_target_speed_R_full(float x, float distance, float ramping__acceleration, float breaking__acceleration);
float calculate_instant_target_speed_L_full(float x, float distance, float ramping__acceleration, float breaking__acceleration);
float calculate_instant_target_speed_R(float x, float distance);
float calculate_instant_target_speed_L(float x, float distance);
float calculate_instant_target_phi(float x, float distance);
int rotate(float angle, int8_t is_radian);									   // is_radian should be set to 1 if the angle is expressed in radians and to 0 if in degrees
int orientate(float orientation, int8_t is_radian);
int go_to(float x, float y, float speed ,float direction); // will return 0 if target has been reached successfully or 1 if some error happened in rotation, 2 if some error happened in moving forward
// int move_linear(float distance, float speed, float target_point_x, float target_point_y, int direction);
void emergency_break();
void reset_speed_control_variables();
bool is_idle();
void wait_for_idle();
void set_target_encoder_counts(int32_t encoder__count_R, int32_t encoder__count_L);
void wait_for_obstacle_clearance();
void wait_for_idle();
void prepare_emergency_break();
int bezier_curve(float x1, float y1, float x2, float y2, float x3, float y3, float speed, int direction);
bool check_for_collision(int* counter);
void recover_from_emergency_break();
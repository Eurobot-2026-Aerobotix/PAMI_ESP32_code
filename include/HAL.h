#pragma once
/**
 * @file HLHA.h
 * @brief Motor and encoder for PAMI robot using Arduino.
 *
 * This file contains function declarations and global variables for controlling the motors and
 * reading encoder.
 *
 * - Motor control is achieved through PWM signals.
 * - Encoder counts are updated using interrupt service routines (ISRs).
 * - Functions are provided to set motor speeds and initialize hardware.
 *
 * @note The macro FAST_READ(pin) and FAST_READ1(pin) are used for fast digital reads by directly accessing
 *       the GPIO register. The expression `& 0x1` is used to mask all bits except the
 *       least significant bit, effectively extracting the digital state (HIGH or LOW)
 *       of the specified pin.
 */

#include <Arduino.h>
#include "driver/pcnt.h"
#include <ESP32Servo.h>
#include <ESP32Servo.h>

#define FAST_READ(pin) (((uint32_t)((pin) < 32 ? GPIO.in : GPIO.in1.val) >> ((pin) & 31)) & 0x1)

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CH   PCNT_CHANNEL_0
#define PCNT_UNIT PCNT_UNIT_1
#define PCNT_CH   PCNT_CHANNEL_1

// motor pins
extern int motorLa;
extern int motorLb;
extern int motorRa;
extern int motorRb;
extern int motorREN;
extern int motorLEN;
extern int motorREN_channel;
extern int motorLEN_channel;

// encoder pins
extern int encoderRA;
extern int encoderRB;
extern int encoderLA;
extern int encoderLB;

// distance sensor pins
extern int trigger;
extern int echo;

//system pins
extern int electrovalve_R_pin;
extern int electrovalve_L_pin;
extern int pump_pin;
extern int servo_pin;
extern Servo servo;

// global constants
extern int PWM_R_forward_min;
extern int PWM_R_backward_min;
extern int PWM_L_forward_min;
extern int PWM_L_backward_min;
extern int PWM_max;
extern int CPR;

// global variables
extern volatile int32_t encoder_count_R;
extern volatile int32_t encoder_count_L;
extern volatile int32_t speedCalculationPreviousTime;
extern float direction_R;
extern float direction_L;
extern float directionR;
extern float directionL;
extern volatile float current_speed_R;
extern volatile float current_speed_L;
extern float current_speed_L_filtered;
extern float current_speed_R_filtered;
extern volatile float target_speed_R;
extern volatile float target_speed_L;
extern int PWM_offset_R;
extern int PWM_offset_L;
extern volatile int PWM_R;
extern volatile int PWM_L;

// physical parameters
extern float wheel_diameter_R;
extern float wheel_diameter_L;
extern float wheels_spacing;

// PID control parameters
extern float kpS_R;
extern float kdS_R;
extern float kiS_R;
extern float speed_integral_R;
extern float speed_derivative_R;
extern float previous_speed_error_R;

extern float kpS_L;
extern float kdS_L;
extern float kiS_L;
extern float speed_integral_L;
extern float speed_derivative_L;
extern float previous_speed_error_L;

extern float previous_position_error_R;
extern float previous_position_error_L;
extern int32_t previous_position_control_time;
extern float position_max_response;
extern float position_integral_R;
extern float position_derivative_R;
extern float position_integral_L;
extern float position_derivative_L;
extern int32_t target_encoder_count_R;
extern int32_t target_encoder_count_L;

// position control parameters
extern float kpP_R;
extern float kdP_R;
extern float kiP_R;

extern float kpP_L;
extern float kdP_L;
extern float kiP_L;
extern volatile float debugger;
extern int strategy_start_time;
const int max_running_time = 100000; // 10 seconds

extern int servo_positions[6];
extern bool ultrasonic_blocked;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward, int right_enable, int left_enable);
void set_ultrasonic_pins(int trigger_pin, int echo_pin);
void set_system_pins(int right_electrovalve, int left_electrovalve, int pump, int servo);
void set_encoder_channels(int _encoderRA, int _encoderRB, int _encoderLA, int _encoderLB, int cpr);
void set_PWM_min_max(int PWM__max, int PWM__R_forward_min, int PWM__L_forward_min, int PWM__R_backward_min, int PWM__L_backward_min);
void set_wheel_diameter_and_spacing(float wheel__diameter_R, float wheel__diameter_L, float wheel__spacing);

void run_right_motor();
void run_left_motor();

void stop_motors();

void IRAM_ATTR encoderRISRA();
void IRAM_ATTR encoderLISRA();

void set_target_speeds(float target__speed_R, float target__speed_L);
void send_values_to_usart(float value1, float value2, float value3, float value4);

float get_ultrasonic_distance();
void reset_position_control_variables();

void encoder_init();
int16_t encoder_get_R();
int16_t encoder_get_L();

void encoder_init();
int16_t encoder_get_R();
int16_t encoder_get_L();

void encoder_init();
int16_t encoder_get_R();
int16_t encoder_get_L();

void encoder_init();
int16_t encoder_get_R();
int16_t encoder_get_L();

void move_servo(int position);//0 means down position and so on till 5 means up position
void set_EV_states(int left_electrovalve_state, int right_electrovalve_state);//0 means off means fantzing, 1 means no tfantiz
void set_pump_state(int pump_state);
void pickup(int pick_right, int pick_left, int up_or_down);
void putdown();
void walking_position();
void prepare_servo_up();
void letgo();
void HAL_setup();
void custom_delay(int ms);
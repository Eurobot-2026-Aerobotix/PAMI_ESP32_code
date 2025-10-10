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

#define FAST_READ(pin) (((uint32_t)GPIO.in >> pin) & 0x1)
#define FAST_READ1(pin) (((uint32_t)GPIO.in1.val >> ((pin) - 32)) & 0x1)
#define PI 3.14159265358979323846

// motor pins
extern int motorLa;
extern int motorLb;
extern int motorRa;
extern int motorRb;
extern int motorLa_channel;
extern int motorLb_channel;
extern int motorRa_channel;
extern int motorRb_channel;

// encoder pins
extern int encoderRA;
extern int encoderRB;
extern int encoderLA;
extern int encoderLB;

// global constants
extern int PWM_R_forward_min;
extern int PWM_R_backward_min;
extern int PWM_L_forward_min;
extern int PWM_L_backward_min;
extern int PWM_max;
extern int CPR;

// global variables
extern volatile int encoder_count_R;
extern volatile int encoder_count_L;
extern volatile int32_t speedCalculationPreviousTimeR;
extern volatile int32_t speedCalculationCurrentTimeR;
extern volatile int32_t speedCalculationPreviousTimeL;
extern volatile int32_t speedCalculationCurrentTimeL;
extern float direction_R;
extern float direction_L;
extern volatile float current_speed_R;
extern volatile float current_speed_L;
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

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward);
void set_encoder_channels(int encoderRA, int encoderRB, int encoderLA, int encoderLB, int cpr);
void set_PWM_min_max(int PWM_max, int PWM_R_forward_min, int PWM_L_forward_min, int PWM_R_backward_min, int PWM_L_backward_min);
void set_wheel_diameter_and_spacing(float wheel_diameter_R, float wheel_diameter_L, float wheel_spacing);

void run_right_motor();
void run_left_motor();

void stop_motors();

void IRAM_ATTR encoderRISRA();
void IRAM_ATTR encoderLISRA();

void set_target_speeds(float target__speed_R, float target__speed_L);
void send_values_to_usart(float value1, float value2, float value3, float value4);

void HAL_setup();
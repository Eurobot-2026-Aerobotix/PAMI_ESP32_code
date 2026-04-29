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
#include <Preferences.h>

#define FAST_READ(pin) (((uint32_t)((pin) < 32 ? GPIO.in : GPIO.in1.val) >> ((pin) & 31)) & 0x1)

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CH   PCNT_CHANNEL_0
#define PCNT_UNIT PCNT_UNIT_1
#define PCNT_CH   PCNT_CHANNEL_1
extern Preferences prefs;

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
extern int electrovalve_pin;
extern int pump_left_pin;
extern int pump_right_pin;
extern int servo_pin;
extern Servo servo;

// tirette
extern int tirette_pin;
extern int color_switch_pin;
extern int agent_led_pin;

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
extern volatile int32_t encoder_raw_count_R;
extern volatile int32_t encoder_raw_count_L;
extern volatile int32_t encoder_offset_R;
extern volatile int32_t encoder_offset_L;
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
extern double wheel_diameter_R;
extern double wheel_diameter_L;
extern double wheels_spacing;

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
extern volatile bool encoder_guard_fault;
extern int32_t encoder_guard_threshold_ticks;

// position control parameters
extern float kpP_R;
extern float kdP_R;
extern float kiP_R;

extern float kpP_L;
extern float kdP_L;
extern float kiP_L;
extern volatile float debugger;
extern int strategy_start_time;
const int max_running_time = 100000;
extern bool strategy_started;
extern bool strategy_waiting;
extern int servo_positions[9];
extern bool ultrasonic_blocked;
extern bool heza;
extern int strategy_commands[400];
extern int strategy_length;
extern bool is_yellow;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward, int right_enable, int left_enable);
void set_tirettes_pins(int tirette_pin, int color_switch_pin);
void set_ultrasonic_pins(int trigger_pin, int echo_pin);
void set_system_pins(int electrovalve, int pump_left, int pump_right, int servo);
void set_agent_led_pin(int led_pin);
void set_agent_led_state(bool state);
void set_agent_led_on();
void set_agent_led_off();
void set_encoder_channels(int _encoderRA, int _encoderRB, int _encoderLA, int _encoderLB, int cpr);
void set_PWM_min_max(int PWM__max, int PWM__R_forward_min, int PWM__L_forward_min, int PWM__R_backward_min, int PWM__L_backward_min);
void set_wheel_diameter_and_spacing(double wheel__diameter_R, double wheel__diameter_L, double wheel__spacing);

void run_right_motor();
void run_left_motor();

void stop_motors();

void IRAM_ATTR encoderRISRA();
void IRAM_ATTR encoderLISRA();

void set_target_speeds(float target__speed_R, float target__speed_L);
void send_values_to_usart(float value1, float value2, float value3, float value4);

float get_ultrasonic_distance();
void reset_position_control_variables();
bool get_tirette_state();
bool get_color_switch_state();

void save_strategy(int* strategy, int strategy_length, bool yellow);
void load_strategy(int* strategy, int* strategy_length, bool yellow);

void encoder_init();
int16_t encoder_get_R();
int16_t encoder_get_L();

void move_servo(int position);//0 means down position and so on till 5 means up position
void set_electrovalve_state(int state);
void set_pump_states(int left_state, int right_state);
extern volatile int pump_left_active;
extern volatile int pump_right_active;
void pickup(int pick_right, int pick_left, int up_or_down);
void putdown(int bechwaya, int nid_lot);
void walking_position();
void prepare_servo_up();
void letgo();
void HAL_setup();
void custom_delay(int ms);
void party_time();
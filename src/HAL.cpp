#pragma once

#include "HAL.h"

// motor pins
int motorLa;
int motorLb;
int motorRa;
int motorRb;

// encoder pins
int encoderRA;
int encoderRB;
int encoderLA;
int encoderLB;
int motorLa_channel;
int motorLb_channel;
int motorRa_channel;
int motorRb_channel;

// global constants
int PWM_R_forward_min;
int PWM_R_backward_min;
int PWM_L_forward_min;
int PWM_L_backward_min;
int PWM_max;
int CPR;

// global variables
volatile int encoder_count_R;
volatile int encoder_count_L;
volatile int32_t speedCalculationPreviousTimeR;
volatile int32_t speedCalculationCurrentTimeR;
volatile int32_t speedCalculationPreviousTimeL;
volatile int32_t speedCalculationCurrentTimeL;
volatile int8_t direction_R;
volatile int8_t direction_L;
volatile float current_speed_R;
volatile float current_speed_L;
volatile float target_speed_R;
volatile float target_speed_L;
int PWM_offset_R;
int PWM_offset_L;
volatile int PWM_R;
volatile int PWM_L;

// physical parameters
float wheel_diameter_R;
float wheel_diameter_L;
float wheels_spacing;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward)
{
    motorRa = right_forward;
    motorRb = right_backward;
    motorLa = left_forward;
    motorLb = left_backward;
}
void set_encoder_channels(int encoderRA, int encoderRB, int encoderLA, int encoderLB, int cpr)
{
    encoderRA = encoderRA;
    encoderRB = encoderRB;
    encoderLA = encoderLA;
    encoderLB = encoderLB;
    CPR = cpr;
}
void set_PWM_min_max(int PWM_max, int PWM_R_forward_min, int PWM_L_forward_min, int PWM_R_backward_min, int PWM_L_backward_min)
{
    PWM_max = PWM_max;
    PWM_R_forward_min = PWM_R_forward_min;
    PWM_R_backward_min = PWM_R_backward_min;
    PWM_L_forward_min = PWM_L_forward_min;
    PWM_L_backward_min = PWM_L_backward_min;
}
void set_wheel_diameter_and_spacing(float wheel_diameter_R, float wheel_diameter_L, float wheels_spacing)
{
    wheel_diameter_R = wheel_diameter_R;
    wheel_diameter_L = wheel_diameter_L;
    wheels_spacing = wheels_spacing;
}

void run_right_motor()
{
    int pwm = (PWM_R > 0) ? ((PWM_R < PWM_R_forward_min) ? 0 : PWM_R) : ((PWM_R > PWM_R_backward_min) ? 0 : PWM_R);
    pwm = constrain(pwm, -PWM_max, PWM_max);
    if (pwm > 0)
    {
        ledcWrite(motorRa_channel, pwm);
        ledcWrite(motorRb_channel, 0);
    }
    else
    {
        ledcWrite(motorRa_channel, 0);
        ledcWrite(motorRb_channel, -pwm);
    }
}
void run_left_motor()
{
    int pwm = (PWM_R > 0) ? ((PWM_R < PWM_L_forward_min) ? 0 : PWM_R) : ((PWM_R > PWM_L_backward_min) ? 0 : PWM_R);
    pwm = constrain(pwm, -PWM_max, PWM_max);
    if (pwm > 0)
    {
        ledcWrite(motorLa_channel, pwm);
        ledcWrite(motorLb_channel, 0);
    }
    else
    {
        ledcWrite(motorLa_channel, 0);
        ledcWrite(motorLb_channel, -pwm);
    }
}

void set_target_speeds(float target__speed_R, float target__speed_L)
{
    PWM_offset_R = (target__speed_R > 0) ? PWM_R_forward_min - 30 : PWM_R_backward_min + 30;
    target_speed_R = target__speed_R;
    PWM_offset_L = (target__speed_L > 0) ? PWM_L_forward_min - 30 : PWM_L_backward_min + 30;
    target_speed_L = target__speed_L;
}

void IRAM_ATTR encoderRISRA()
{
    if (FAST_READ(encoderRB) == FAST_READ1(encoderRA))
    {
        encoder_count_R--;
        direction_R = -1;
    }
    else
    {
        encoder_count_R++;
        direction_R = 1;
    }
    speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
    speedCalculationCurrentTimeR = micros();

    // optimise by replacing (wheel_diameter_R*PI/CPR * direction_R) by a constant factor
    current_speed_R = (wheel_diameter_R * PI / CPR * direction_R) / (speedCalculationCurrentTimeR - speedCalculationPreviousTimeR);
}
void IRAM_ATTR encoderLISRA()
{
    if (FAST_READ(encoderLB) == FAST_READ(encoderLA))
    {
        encoder_count_L++;
        direction_L = 1;
    }
    else
    {
        encoder_count_L--;
        direction_L = -1;
    }
    speedCalculationPreviousTimeL = speedCalculationCurrentTimeL;
    speedCalculationCurrentTimeL = micros();
    current_speed_L = (wheel_diameter_L * PI / CPR * direction_L) / (speedCalculationCurrentTimeL - speedCalculationPreviousTimeL);
}

void send_values_to_usart(float value1, float value2, float value3, float value4)
{
    // temporary
    Serial.print(value1);
    Serial.print(",");
    Serial.print(value2);
    Serial.print(",");
    Serial.print(value3);
    Serial.print(",");
    Serial.println(value4);
}

void stop_motors()
{
    PWM_R = 0;
    PWM_L = 0;
    target_speed_R = 0;
    target_speed_L = 0;
    run_right_motor();
    run_left_motor();
}

void HAL_setup()
{
    motorRa_channel = 0;
    motorRb_channel = 1;
    motorLa_channel = 2;
    motorLb_channel = 3;
    ledcSetup(motorRa_channel, 20000, 10);
    ledcSetup(motorRb_channel, 20000, 10);
    ledcSetup(motorLa_channel, 20000, 10);
    ledcSetup(motorLb_channel, 20000, 10);
    ledcAttachPin(motorRa, motorRa_channel);
    ledcAttachPin(motorRb, motorRb_channel);
    ledcAttachPin(motorLa, motorLa_channel);
    ledcAttachPin(motorLb, motorLb_channel);

    // Global variables
    speedCalculationCurrentTimeR = micros();
    speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
    speedCalculationCurrentTimeL = speedCalculationCurrentTimeR;
    speedCalculationPreviousTimeL = speedCalculationCurrentTimeR;
    current_speed_L = 0;
    current_speed_R = 0;
    direction_R = 1;
    direction_L = 1;

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
}
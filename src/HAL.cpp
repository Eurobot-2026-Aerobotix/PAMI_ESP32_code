#include "HAL.h"

// motor pins
int motorLa;
int motorLb;
int motorRa;
int motorRb;
int motorREN;
int motorLEN;

// encoder pins
int encoderRA;
int encoderRB;
int encoderLA;
int encoderLB;
int motorREN_channel;
int motorLEN_channel;

// global constants
int PWM_R_forward_min;
int PWM_R_backward_min;
int PWM_L_forward_min;
int PWM_L_backward_min;
int PWM_max;
int CPR;

// global variables
volatile int32_t encoder_count_R;
volatile int32_t encoder_count_L;
volatile int32_t speedCalculationPreviousTimeR;
volatile int32_t speedCalculationCurrentTimeR;
volatile int32_t speedCalculationPreviousTimeL;
volatile int32_t speedCalculationCurrentTimeL;
float direction_R;
float direction_L;
volatile float current_speed_R;
volatile float current_speed_L;
float current_speed_L_filtered = 0;
float current_speed_R_filtered = 0;
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

// PID control parameters
float kpS_R = 5;
float kdS_R = 0.0000;
float kiS_R = 10;
float speed_integral_R = 0;
float speed_derivative_R = 0;
float previous_speed_error_R = 0;

float kpS_L = 5;//0
float kdS_L = 0;
float kiS_L = 10;//3
float speed_integral_L = 0;
float speed_derivative_L = 0;
float previous_speed_error_L = 0;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward, int right_enable, int left_enable)
{
    motorRa = right_forward;
    motorRb = right_backward;
    motorLa = left_forward;
    motorLb = left_backward;
    motorREN = right_enable;
    motorLEN = left_enable;
}
void set_encoder_channels(int _encoderRA, int _encoderRB, int _encoderLA, int _encoderLB, int cpr)
{
    encoderRA = _encoderRA;
    encoderRB = _encoderRB;
    encoderLA = _encoderLA;
    encoderLB = _encoderLB;
    CPR = cpr;
}
void set_PWM_min_max(int PWM__max, int PWM__R_forward_min, int PWM__L_forward_min, int PWM__R_backward_min, int PWM__L_backward_min)
{
    PWM_max = PWM__max;
    PWM_R_forward_min = PWM__R_forward_min;
    PWM_R_backward_min = PWM__R_backward_min;
    PWM_L_forward_min = PWM__L_forward_min;
    PWM_L_backward_min = PWM__L_backward_min;
}
void set_wheel_diameter_and_spacing(float wheel__diameter_R, float wheel__diameter_L, float wheels__spacing)
{
    wheel_diameter_R = wheel__diameter_R;
    wheel_diameter_L = wheel__diameter_L;
    wheels_spacing = wheels__spacing;
}

void run_right_motor()
{
    int pwm = (PWM_R > 0) ? ((PWM_R < PWM_R_forward_min) ? 0 : PWM_R) : ((PWM_R > PWM_R_backward_min) ? 0 : PWM_R);
    pwm = constrain(pwm, -PWM_max, PWM_max);
    
    ledcWrite(motorREN_channel, abs(pwm));
    if (pwm > 0){
        digitalWrite(motorRa, 1);
        digitalWrite(motorRb, 0);
    }
    else if (pwm < 0){
        digitalWrite(motorRa, 0);
        digitalWrite(motorRb, 1);
    }else{
        digitalWrite(motorRa, 1);
        digitalWrite(motorRb, 1);
    }
}
void run_left_motor()
{
    int pwm = (PWM_L > 0) ? ((PWM_L < PWM_L_forward_min) ? 0 : PWM_L) : ((PWM_L > PWM_L_backward_min) ? 0 : PWM_L);
    pwm = constrain(pwm, -PWM_max, PWM_max);
    ledcWrite(motorLEN_channel, abs(pwm));
    if (pwm > 0){
        digitalWrite(motorLa, 1);
        digitalWrite(motorLb, 0);
    }
    else if (pwm < 0){
        digitalWrite(motorLa, 0);
        digitalWrite(motorLb, 1);
    }else{
        digitalWrite(motorLa, 1);
        digitalWrite(motorLb, 1);
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
    if (FAST_READ(encoderRB) == FAST_READ(encoderRA))
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

    // optimise by replacing (wheel_diameter_R*PI/CPR * direction_R) by a constant factors
    current_speed_R = (wheel_diameter_R * PI / CPR * direction_R)*1000000 / (speedCalculationCurrentTimeR - speedCalculationPreviousTimeR);
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
    current_speed_L = (wheel_diameter_L * PI / CPR * direction_L)*1000000 / (speedCalculationCurrentTimeL - speedCalculationPreviousTimeL);
}

void send_values_to_usart(float value1, float value2, float value3, float value4)
{
    // temporary
    Serial.write((uint8_t)0xAA);
    Serial.write((uint8_t*)&value1, sizeof(value1));
    Serial.write((uint8_t)0xAB);
    Serial.write((uint8_t*)&value2, sizeof(value2));
    Serial.write((uint8_t)0xAC);
    Serial.write((uint8_t*)&value3, sizeof(value3));
    Serial.write((uint8_t)0xAD);
    Serial.write((uint8_t*)&value4, sizeof(value4));
}

void stop_motors()
{
    PWM_R = 0;
    PWM_L = 0;
    target_speed_R = 0;
    target_speed_L = 0;
    run_right_motor();
    run_left_motor();
    set_target_speeds(0, 0);
}

void HAL_setup()
{
    pinMode(motorRa, OUTPUT);
    pinMode(motorRb, OUTPUT);
    pinMode(motorLa, OUTPUT);
    pinMode(motorLb, OUTPUT);

    pinMode(encoderRA, INPUT_PULLUP);
    pinMode(encoderRB, INPUT_PULLUP);
    pinMode(encoderLA, INPUT_PULLUP);
    pinMode(encoderLB, INPUT_PULLUP);

    motorREN_channel = 8;
    motorLEN_channel = 9;
    ledcSetup(motorREN_channel, 20000, 10);
    ledcSetup(motorLEN_channel, 20000, 10);
    ledcAttachPin(motorREN, motorREN_channel);
    ledcAttachPin(motorLEN, motorLEN_channel);

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
void custom_delay(int ms)
{
    unsigned long start_time = millis();
    while (millis() - start_time < ms)
    {
        vTaskDelay(1);
    }
}
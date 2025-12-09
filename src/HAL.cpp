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

// distance sensor pins
int trigger;
int echo;

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
float directionR;
float directionL;
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
float kpS_R = 6;
float kdS_R = 0.0000;
float kiS_R = 10;
float speed_integral_R = 0;
float speed_derivative_R = 0;
float previous_speed_error_R = 0;

float kpS_L = 6;//0
float kdS_L = 0;
float kiS_L = 10;//3
float speed_integral_L = 0;
float speed_derivative_L = 0;
float previous_speed_error_L = 0;

float previous_position_error_R = 0;
float previous_position_error_L = 0;
int32_t previous_position_control_time = 0;
float position_max_response = 100;
float position_integral_R = 0;
float position_derivative_R = 0;
float position_integral_L = 0;
float position_derivative_L = 0;
int32_t target_encoder_count_R = 0;
int32_t target_encoder_count_L = 0;

// position control parameters
float kpP_R = 10;
float kdP_R = 5;
float kiP_R = 60;

float kpP_L = 10;
float kdP_L = 5;
float kiP_L = 60;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward, int right_enable, int left_enable)
{
    motorRa = right_forward;
    motorRb = right_backward;
    motorLa = left_forward;
    motorLb = left_backward;
    motorREN = right_enable;
    motorLEN = left_enable;
}

void set_ultrasonic_pins(int trigger_pin, int echo_pin){
    trigger = trigger_pin;
    echo = echo_pin;
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
        digitalWrite(motorRa, 0);
        digitalWrite(motorRb, 0);
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
        digitalWrite(motorLa, 0);
        digitalWrite(motorLb, 0);
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
        directionR = -1;
    }
    else
    {
        encoder_count_R++;
        directionR = 1;
    }
    speedCalculationPreviousTimeR = speedCalculationCurrentTimeR;
    speedCalculationCurrentTimeR = micros();

    // optimise by replacing (wheel_diameter_R*PI/CPR * direction_R) by a constant factors
    current_speed_R = (wheel_diameter_R * PI / CPR * directionR)*1000000 / (speedCalculationCurrentTimeR - speedCalculationPreviousTimeR);
}
void IRAM_ATTR encoderLISRA()
{
    if (FAST_READ(encoderLB) == FAST_READ(encoderLA))
    {
        encoder_count_L++;
        directionL = 1;
    }
    else
    {
        encoder_count_L--;
        directionL = -1;
    }
    speedCalculationPreviousTimeL = speedCalculationCurrentTimeL;
    speedCalculationCurrentTimeL = micros();
    current_speed_L = (wheel_diameter_L * PI / CPR * directionL)*1000000 / (speedCalculationCurrentTimeL - speedCalculationPreviousTimeL);
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

float get_ultrasonic_distance(){
    long duration;
    float distance;

    // Clear the trigger pin
    digitalWrite(trigger, LOW);
    delayMicroseconds(2);

    // Set the trigger pin HIGH for 10 microseconds
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);

    // Read the echo pin
    duration = pulseIn(echo, HIGH, 2000); // Timeout after 2000us to avoid blocking

    // Calculate distance in mm
    distance = (duration / 2.0) * 0.343; // Speed of sound is ~343 m/s or 0.0343 mm/µs

    // If no echo received, return -1
    if (duration == 0) {
        return 500.0;
    }

    return distance;
}

void reset_position_control_variables(){
	previous_position_control_time = micros()-999;
	position_integral_L = 0;
	position_integral_R = 0;
	position_derivative_L = 0;
	position_derivative_R = 0;
	PWM_L = 0;
	PWM_R = 0;
	previous_position_error_L = 0;
	previous_position_error_R = 0;
}

void stop_motors()
{
    PWM_R = 0;
    PWM_L = 0;
    target_speed_R = 0;
    target_speed_L = 0;
    for(int i=0; i<10; i++){
        run_right_motor();
        run_left_motor();
        custom_delay(10);
    }
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

    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);

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
    directionR = 1;
    directionL = 1;

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
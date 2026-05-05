#include "HAL.h"

Preferences prefs;

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
// tirette
int tirette_pin;
int color_switch_pin;
int agent_led_pin = -1;

//servo
int servo_pin;

Servo servo;

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
volatile int32_t speedCalculationPreviousTime;
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
float kpS_R = 3;
float kdS_R = 0.0000;
float kiS_R = 40;
float speed_integral_R = 0;
float speed_derivative_R = 0;
float previous_speed_error_R = 0;

float kpS_L = 3;
float kdS_L = 0;
float kiS_L = 40;
float speed_integral_L = 0;
float speed_derivative_L = 0;
float previous_speed_error_L = 0;

float previous_position_error_R = 0;
float previous_position_error_L = 0;
int32_t previous_position_control_time = 0;
float position_max_response = 150;
float position_integral_R = 0;
float position_derivative_R = 0;
float position_integral_L = 0;
float position_derivative_L = 0;
int32_t target_encoder_count_R = 0;
int32_t target_encoder_count_L = 0;

// position control parameters
float kpP_R = 200;
float kdP_R = 5;
float kiP_R = 1000;

float kpP_L = 200;
float kdP_L = 5;
float kiP_L = 1000;
volatile float debugger = 0;
int strategy_start_time = 1000000;
bool strategy_waiting = 0;

int strategy_commands[300];
int strategy_length;
bool is_yellow = true;

void set_motor_pins(int right_forward, int right_backward, int left_forward, int left_backward, int right_enable, int left_enable)
{
    motorRa = right_forward;
    motorRb = right_backward;
    motorLa = left_forward;
    motorLb = left_backward;
    motorREN = right_enable;
    motorLEN = left_enable;
}
void set_tirettes_pins(int tirette__pin, int color__switch_pin){
    tirette_pin = tirette__pin;
    color_switch_pin = color__switch_pin;
}
void set_ultrasonic_pins(int trigger_pin, int echo_pin){
    trigger = trigger_pin;
    echo = echo_pin;
}
void set_agent_led_pin(int led_pin)
{
    agent_led_pin = led_pin;
}
void set_servo_pin(int servo__pin){
    servo_pin = servo__pin;
}
void set_agent_led_state(bool state)
{
    if (agent_led_pin >= 0)
    {
        digitalWrite(agent_led_pin, state ? HIGH : LOW);
    }
}
void set_agent_led_on()
{
    set_agent_led_state(true);
}
void set_agent_led_off()
{
    set_agent_led_state(false);
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
    PWM_offset_R = (target__speed_R > 0) ? PWM_R_forward_min - 20 : PWM_R_backward_min + 20;
    target_speed_R = target__speed_R;
    PWM_offset_L = (target__speed_L > 0) ? PWM_L_forward_min - 20 : PWM_L_backward_min + 20;
    target_speed_L = target__speed_L;
}

void IRAM_ATTR encoderRISRA()
{
    if (FAST_READ(encoderRB) == FAST_READ(encoderRA))
    {
        encoder_count_R--;
    }
    else
    {
        encoder_count_R++;
    }
}
void IRAM_ATTR encoderLISRA()
{
    if (FAST_READ(encoderLB) == FAST_READ(encoderLA))
    {
        encoder_count_L++;
    }
    else
    {
        encoder_count_L--;
    }
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

bool get_tirette_state(){
    return digitalRead(tirette_pin);
}
bool get_color_switch_state(){
    return digitalRead(color_switch_pin) == 0;
}

void save_strategy(int* strategy, int strategy_length, bool yellow){
    if (strategy == NULL) {
        return;
    }

    const int max_strategy = 300;
    if (strategy_length < 0) {
        strategy_length = 0;
    }
    if (strategy_length > max_strategy) {
        strategy_length = max_strategy;
    }

    int buffer[max_strategy + 1];
    buffer[0] = strategy_length;
    for (int i = 0; i < strategy_length; i++){
        buffer[i+1] = strategy[i];
    }

    const size_t bytes_to_write = (size_t)(strategy_length + 1) * sizeof(int);
    prefs.begin("pami", false);
    prefs.putBytes(yellow? "strategy_yellow" : "strategy_blue", buffer, bytes_to_write);
    prefs.end();
}
void load_strategy(int* strategy, int* strategy_length, bool yellow){
    if (strategy == NULL || strategy_length == NULL) {
        return;
    }

    prefs.begin("pami", true);
    int buffer[301];
    size_t bytesRead = prefs.getBytes(yellow? "strategy_yellow" : "strategy_blue", buffer, sizeof(buffer));
    prefs.end();
    if (bytesRead > 0 && bytesRead <= sizeof(buffer)){
        *strategy_length = buffer[0];
        if (*strategy_length < 0) {
            *strategy_length = 0;
        }
        if (*strategy_length > 300) {
            *strategy_length = 300;
        }
        for (int i = 0; i < *strategy_length; i++){
            strategy[i] = buffer[i+1];
        }
    }else{
        *strategy_length = 0; // no strategy found
    }
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

void encoder_init() {
  pcnt_config_t pcnt_config_1 = {
    .pulse_gpio_num = encoderRB,
    .ctrl_gpio_num  = encoderRA,

    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,

    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,

    .counter_h_lim = 32767,
    .counter_l_lim = -32768,

    .unit    = PCNT_UNIT_0,
    .channel = PCNT_CHANNEL_0
  };

  pcnt_unit_config(&pcnt_config_1);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  pcnt_config_t pcnt_config_2 = {
    .pulse_gpio_num = encoderLA,
    .ctrl_gpio_num  = encoderLB,

    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,

    .pos_mode = PCNT_COUNT_INC,
    .neg_mode = PCNT_COUNT_DIS,

    .counter_h_lim = 32767,
    .counter_l_lim = -32768,

    .unit    = PCNT_UNIT_1,
    .channel = PCNT_CHANNEL_1
  };

  pcnt_unit_config(&pcnt_config_2);
  pcnt_set_filter_value(PCNT_UNIT_1, 100);
  pcnt_filter_enable(PCNT_UNIT_1);

  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);
}

int16_t encoder_get_R() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
  return count;
}

int16_t encoder_get_L() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_1, &count);
  return count;
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

    pinMode(tirette_pin, INPUT_PULLUP);
    pinMode(color_switch_pin, INPUT_PULLUP);

    if (agent_led_pin >= 0)
    {
        pinMode(agent_led_pin, OUTPUT);
        set_agent_led_off();
    }

    motorREN_channel = 8;
    motorLEN_channel = 9;
    ledcSetup(motorREN_channel, 20000, 10);
    ledcSetup(motorLEN_channel, 20000, 10);
    ledcAttachPin(motorREN, motorREN_channel);
    ledcAttachPin(motorLEN, motorLEN_channel);

    // Global variables
    speedCalculationPreviousTime = micros() - 999;
    current_speed_L = 0;
    current_speed_R = 0;
    direction_R = 1;
    direction_L = 1;
    directionR = 1;
    directionL = 1;

    servo.attach(servo_pin);
    servo.write(180);
    delay(250);
    servo.write(0);

    // Interrupts
    //attachInterrupt(digitalPinToInterrupt(encoderRA), encoderRISRA, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(encoderLA), encoderLISRA, CHANGE);
    encoder_init();
}
void custom_delay(int ms)
{
    unsigned long start_time = millis();
    while (millis() - start_time < ms)
    {
        vTaskDelay(1);
    }
}

void party_time()
{
    while(1){
        servo.write(180);
        set_agent_led_on();
        delay(500);
        set_agent_led_off();
        servo.write(0);
        delay(500);
    }
}
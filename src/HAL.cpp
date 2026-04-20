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

//system pins
int electrovalve_pin;
int pump_left_pin;
int pump_right_pin;
int servo_pin;
Servo servo;
// tirette
int tirette_pin;
int color_switch_pin;
int agent_led_pin = -1;

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
volatile int32_t encoder_raw_count_R;
volatile int32_t encoder_raw_count_L;
volatile int32_t encoder_offset_R;
volatile int32_t encoder_offset_L;
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
double wheel_diameter_R;
double wheel_diameter_L;
double wheels_spacing;

// PID control parameters
float kpS_R = 10;
float kdS_R = 0.0000;
float kiS_R = 100;
float speed_integral_R = 0;
float speed_derivative_R = 0;
float previous_speed_error_R = 0;

float kpS_L = 10;
float kdS_L = 0;
float kiS_L = 100;
float speed_integral_L = 0;
float speed_derivative_L = 0;
float previous_speed_error_L = 0;

float previous_position_error_R = 0;
float previous_position_error_L = 0;
int32_t previous_position_control_time = 0;
float position_max_response = 250;
float position_integral_R = 0;
float position_derivative_R = 0;
float position_integral_L = 0;
float position_derivative_L = 0;
int32_t target_encoder_count_R = 0;
int32_t target_encoder_count_L = 0;

// position control parameters
float kpP_R = 100;
float kdP_R = 5;
float kiP_R = 400;

float kpP_L = 100;
float kdP_L = 5;
float kiP_L = 400;
volatile float debugger = 0;
int strategy_start_time = 100000;
//   {intial pos , pick pos down , walk pos without crates , 110, pick pos up , servo up , putdown pos nid , 70 , put down pos lot };  
int servo_positions[9] =  {0, 150, 75, 110, 106, 30, 120 , 70 , 150};   //0, 130, 48, 85, 50, 15, 110, 50, 130
bool ultrasonic_blocked = true;
bool heza = false;

int strategy_commands[400];
int strategy_length;
bool is_yellow = true;
bool strategy_waiting = false;

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
void set_system_pins(int electrovalve, int pump_left, int pump_right, int servo){
    electrovalve_pin = electrovalve;
    pump_left_pin = pump_left;
    pump_right_pin = pump_right;
    servo_pin = servo;
}
void set_agent_led_pin(int led_pin)
{
    agent_led_pin = led_pin;
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
void set_wheel_diameter_and_spacing(double wheel__diameter_R, double wheel__diameter_L, double wheels__spacing)
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
        encoder_raw_count_R--;
    }
    else
    {
        encoder_raw_count_R++;
    }
    encoder_count_R = encoder_raw_count_R + encoder_offset_R;
}
void IRAM_ATTR encoderLISRA()
{
    if (FAST_READ(encoderLB) == FAST_READ(encoderLA))
    {
        encoder_raw_count_L++;
    }
    else
    {
        encoder_raw_count_L--;
    }
    encoder_count_L = encoder_raw_count_L + encoder_offset_L;
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
    if (strategy_length < 0) {
        strategy_length = 0;
    }
    if (strategy_length > 400) {
        strategy_length = 400;
    }

    int buffer[401] = {0};
    buffer[0] = strategy_length;
    for (int i = 0; i < strategy_length; i++){
        buffer[i+1] = strategy[i];
    }
    prefs.begin("pami", false);
    prefs.putBytes(yellow? "strategy_yellow" : "strategy_blue", buffer, sizeof(buffer));
    prefs.end();
}
void load_strategy(int* strategy, int* strategy_length, bool yellow){
    prefs.begin("pami", true);
    int buffer[401] = {0};
    size_t bytesRead = prefs.getBytes(yellow? "strategy_yellow" : "strategy_blue", buffer, sizeof(buffer));
    prefs.end();
    if (bytesRead > 0 && bytesRead <= sizeof(buffer)){
        *strategy_length = buffer[0];
        if (*strategy_length < 0) {
            *strategy_length = 0;
        }
        if (*strategy_length > 400) {
            *strategy_length = 400;
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

void encoder_init()
{
    // =========================
    // RIGHT ENCODER (UNIT 0)
    // =========================

    pcnt_config_t pcnt_config_R = {0};

    pcnt_config_R.pulse_gpio_num = encoderRA;   // Count edges on A
    pcnt_config_R.ctrl_gpio_num  = encoderRB;   // B decides direction

    pcnt_config_R.unit    = PCNT_UNIT_0;
    pcnt_config_R.channel = PCNT_CHANNEL_0;

    // X2: count rising + falling edges
    pcnt_config_R.pos_mode = PCNT_COUNT_INC;
    pcnt_config_R.neg_mode = PCNT_COUNT_DEC;

    // Direction control
    pcnt_config_R.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_R.hctrl_mode = PCNT_MODE_REVERSE;

    pcnt_config_R.counter_h_lim = 30000;
    pcnt_config_R.counter_l_lim = -30000;

    pcnt_unit_config(&pcnt_config_R);

    pcnt_set_filter_value(PCNT_UNIT_0, 200);   // Start small (50–200 typical)
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);


    // =========================
    // LEFT ENCODER (UNIT 1)
    // =========================

    pcnt_config_t pcnt_config_L = {0};

    pcnt_config_L.pulse_gpio_num = encoderLB;   // Count edges on A
    pcnt_config_L.ctrl_gpio_num  = encoderLA;   // B decides direction

    pcnt_config_L.unit    = PCNT_UNIT_1;
    pcnt_config_L.channel = PCNT_CHANNEL_0;

    pcnt_config_L.pos_mode = PCNT_COUNT_INC;
    pcnt_config_L.neg_mode = PCNT_COUNT_DEC;

    pcnt_config_L.lctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_L.hctrl_mode = PCNT_MODE_REVERSE;

    pcnt_config_L.counter_h_lim = 30000;
    pcnt_config_L.counter_l_lim = -30000;

    pcnt_unit_config(&pcnt_config_L);

    pcnt_set_filter_value(PCNT_UNIT_1, 200);
    pcnt_filter_enable(PCNT_UNIT_1);

    pcnt_counter_pause(PCNT_UNIT_1);
    pcnt_counter_clear(PCNT_UNIT_1);
    pcnt_counter_resume(PCNT_UNIT_1);
}

int16_t encoder_get_R() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_0, &count);
    encoder_raw_count_R += count;
    encoder_count_R = encoder_raw_count_R + encoder_offset_R;

  pcnt_counter_clear(PCNT_UNIT_0);
  return count;
}

int16_t encoder_get_L() {
  int16_t count;
  pcnt_get_counter_value(PCNT_UNIT_1, &count);
    encoder_raw_count_L += count;
    encoder_count_L = encoder_raw_count_L + encoder_offset_L;

  pcnt_counter_clear(PCNT_UNIT_1);
  return count;
}
void move_servo(int position){
    servo.write(servo_positions[position]);
}
void set_electrovalve_state(int state){
    digitalWrite(electrovalve_pin, state);
}
void set_pump_states(int left_state, int right_state){
    digitalWrite(pump_left_pin, left_state);
    digitalWrite(pump_right_pin, right_state);
}
void pickup(int pick_right, int pick_left, int up_or_down){
    set_pump_states(pick_left, pick_right);
    set_electrovalve_state(0);
    ultrasonic_blocked = up_or_down;
    move_servo(up_or_down? 4 : 1);
    if (up_or_down==1){
        custom_delay(1000);
    }else{
        custom_delay(1300);
    }
    move_servo(2);
    custom_delay(500);
    heza = true;
}

//   {0, 135, 70, 110, 85, 15, 120,70,140 };
void putdown(int bechwaya, int nid_lot){
    move_servo(nid_lot ? 6 : 8);
    custom_delay(bechwaya? 400 : 1);
    set_pump_states(0, 0);
    set_electrovalve_state(1);
    move_servo(0);
    custom_delay(1000);
    set_electrovalve_state(0);
    heza = false;
}
void letgo(){
    set_pump_states(0, 0);
    set_electrovalve_state(0);
    move_servo(5);
    custom_delay(500);
}
void walking_position(int position){
    move_servo(position? 0 : 3);
    custom_delay(500);
    ultrasonic_blocked = false;
}
void prepare_servo_up(){
    move_servo(5);
    ultrasonic_blocked = true;
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

    pinMode(electrovalve_pin, OUTPUT);
    pinMode(pump_left_pin, OUTPUT);
    pinMode(pump_right_pin, OUTPUT);

    servo.setPeriodHertz(50);
    servo.attach(servo_pin);
    move_servo(5);

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
void party_time(){
    while(1){
      move_servo(1);
        custom_delay(300);
        move_servo(3);
        custom_delay(300);
    }
}
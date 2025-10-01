#include <Arduino.h>
#include "HAL.h"
#include "navigation.h"

float kpS_R = 5;
float kdS_R = 0.0000;
float kiS_R = 15;
float speed_integral_R = 0;
float speed_derivative_R = 0;
float previous_speed_error_R = 0;

float kpS_L = 5;
float kdS_L = 0;
float kiS_L = 15;
float speed_integral_L = 0;
float speed_derivative_L = 0;

float previous_speed_error_L = 0;

int32_t previous_speed_control_time = 0;

void setup() {
    set_motor_pins(5, 6, 9, 10);
    set_encoder_channels(2, 3, 18, 19, 1440);
    set_PWM_min_max(255, 30, 30, 30, 30);
    set_wheel_diameter_and_spacing(0.1, 0.1, 0.2);

    HAL_setup();
}

void loop() {
}

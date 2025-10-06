#include <Arduino.h>
#include "HAL.h"
#include "odometry.h"
#include "navigation.h"
#include "command_controller.h"

/// tasks
TaskHandle_t communicationTaskHandle;
TaskHandle_t PIDTaskHandle;
TaskHandle_t commandExecutionTaskHandle;

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

void communicationTask(void *pvParameters){
    // tfannen bens3id
}

void PIDTask(void *pvParameters){
    const TickType_t xDelay = 2;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    update_position();
    float tmp = (micros() - previous_speed_control_time)/1000000.0; // in seconds
    // right motor speed control
    float error = target_speed_R - current_speed_R;

    speed_derivative_R = speed_derivative_R*0.999 + 0.001*(error - previous_speed_error_R)/tmp; // unit = mm/s²
    speed_integral_R += error*tmp;

    previous_speed_error_R = error;

    PWM_R = PWM_R*0.2 + 0.8 * (PWM_offset_R + kpS_R*error + kdS_R*speed_derivative_R + kiS_R*speed_integral_R); // the first part is a simple filter to reduce jitter
    PWM_R = (PWM_R > PWM_max)? PWM_max : (PWM_R < -PWM_max)? -PWM_max : PWM_R;
    PWM_R = (target_speed_R == 0)?0: PWM_R;

    speed_integral_R = ( (PWM_R == PWM_max) || (PWM_R == -PWM_max) )? (speed_integral_R - 2*error*tmp) : speed_integral_R;

    // left motor speed control
    error = target_speed_L - current_speed_L;
    speed_derivative_L = speed_derivative_L*0.999 + 0.001*(error - previous_speed_error_L)/tmp; // unit = mm/s²
    speed_integral_L += error*tmp;

    previous_speed_error_L = error;
    previous_speed_control_time = micros();

    PWM_L = PWM_L*0.2 + 0.8 * (PWM_offset_L + kpS_L*error + kdS_L*speed_derivative_L + kiS_L*speed_integral_L);
    PWM_L = (PWM_L > PWM_max)? PWM_max : (PWM_L < -PWM_max)? -PWM_max : PWM_L;
    PWM_L = (target_speed_L == 0)?0: PWM_L;

    speed_integral_L = ( (PWM_L == PWM_max) || (PWM_L == -PWM_max) )? (speed_integral_L - 2*error*tmp) : speed_integral_L;
    vTaskDelayUntil(&xLastWakeTime, xDelay);
}

void commandExecutionTask(void *pvParameters){
    execute_command();
}

void setup() {
    set_motor_pins(5, 6, 9, 10);
    set_encoder_channels(2, 3, 18, 19, 1440);
    set_PWM_min_max(255, 30, 30, 30, 30);
    set_wheel_diameter_and_spacing(0.1, 0.1, 0.2);

    HAL_setup();

    xTaskCreatePinnedToCore(
    communicationTask, 
    "communicationTask", 
    2048, 
    NULL, 
    2, 
    &communicationTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    PIDTask, 
    "PIDTask", 
    2048, 
    NULL, 
    3, 
    &PIDTaskHandle,
    1
  );

  xTaskCreatePinnedToCore(
    commandExecutionTask, 
    "commandExecutionTask", 
    2048, 
    NULL, 
    1, 
    &commandExecutionTaskHandle,
    1
  );
}

void loop() {
}

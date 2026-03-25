#include <Arduino.h>
#include "navigation.h"

TaskHandle_t PIDTaskHandle;
TaskHandle_t emergencyBreakTaskHandle;
TaskHandle_t communicationTaskHandle = NULL;

static bool test_started = false;
static bool test_finished = false;

void PIDTask(void *pvParameters)
{
  (void)pvParameters;
  const TickType_t xDelay = pdMS_TO_TICKS(1);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  previous_speed_control_time = micros();
  const float dt = 0.001f;

  for (;;)
  {
    update_position(dt);

    if (control_loop_state == SPEED)
    {
      float error = target_speed_R - current_speed_R;

      speed_derivative_R = speed_derivative_R * 0.999f + 0.001f * (error - previous_speed_error_R) / dt;
      speed_integral_R += error * dt;
      previous_speed_error_R = error;

      PWM_R = PWM_offset_R + kpS_R * error + kdS_R * speed_derivative_R + kiS_R * speed_integral_R;
      PWM_R = (PWM_R > PWM_max) ? PWM_max : (PWM_R < -PWM_max) ? -PWM_max : PWM_R;
      PWM_R = (target_speed_R == 0) ? 0 : PWM_R;
      speed_integral_R = ((PWM_R == PWM_max) || (PWM_R == -PWM_max)) ? (speed_integral_R - 2 * error * dt) : speed_integral_R;

      error = target_speed_L - current_speed_L;
      speed_derivative_L = speed_derivative_L * 0.999f + 0.001f * (error - previous_speed_error_L) / dt;
      speed_integral_L += error * dt;
      previous_speed_error_L = error;

      PWM_L = PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L;
      PWM_L = (PWM_L > PWM_max) ? PWM_max : (PWM_L < -PWM_max) ? -PWM_max : PWM_L;
      PWM_L = (target_speed_L == 0) ? 0 : PWM_L;
      speed_integral_L = ((PWM_L == PWM_max) || (PWM_L == -PWM_max)) ? (speed_integral_L - 2 * error * dt) : speed_integral_L;
    }
    else if (control_loop_state == POSITION)
    {
      float error = (target_encoder_count_R - encoder_count_R);
      position_derivative_R = (error - previous_position_error_R) / dt;
      position_integral_R = (SGN(error) == SGN(previous_position_error_R)) ? position_integral_R + error * dt : 0;
      previous_position_error_R = error;

      PWM_R = kpP_R * error + kdP_R * position_derivative_R + kiP_R * position_integral_R;
      PWM_R = (PWM_R > PWM_R_forward_min + position_max_response) ? PWM_R_forward_min + position_max_response
                                                                    : (PWM_R < PWM_R_backward_min - position_max_response) ? PWM_R_backward_min - position_max_response
                                                                                                                            : PWM_R;

      error = (target_encoder_count_L - encoder_count_L);
      position_derivative_L = (error - previous_position_error_L) / dt;
      position_integral_L = (SGN(error) == SGN(previous_position_error_L)) ? position_integral_L + error * dt : 0;
      previous_position_error_L = error;

      PWM_L = kpP_L * error + kdP_L * position_derivative_L + kiP_L * position_integral_L;
      PWM_L = (PWM_L > PWM_L_forward_min + position_max_response) ? PWM_L_forward_min + position_max_response
                                                                    : (PWM_L < PWM_L_backward_min - position_max_response) ? PWM_L_backward_min - position_max_response
                                                                                                                            : PWM_L;
    }

    run_right_motor();
    run_left_motor();
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void emergencyBreakTask(void *pvParameters)
{
  (void)pvParameters;
  const TickType_t xDelay = pdMS_TO_TICKS(30);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    if (((( !ultrasonic_blocked) && get_ultrasonic_distance() < min_distance && trajectory == STRAIGHT) ||
         (millis() - strategy_start_time >= max_running_time)) &&
        wheel_state_R != EMERGENCY_BREAK && wheel_state_L != EMERGENCY_BREAK)
    {
      prepare_emergency_break();
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  Serial.println("=== Emergency Break + Ultrasonic Test ===");

  set_motor_pins(19, 18, 21, 22, 17, 23);
  set_system_pins(32, 26, 33, 27);
  set_encoder_channels(2, 15, 4, 16, 1400);
  set_ultrasonic_pins(14, 34);
  set_PWM_min_max(1023, 100, 100, -100, -100);
  set_wheel_diameter_and_spacing(47.7464829275686, 47.66987063547587, 120.4418940144483);
  HAL_setup();

  ultrasonic_blocked = false;
  min_distance = 70;

  xTaskCreatePinnedToCore(PIDTask, "PIDTask", 4096, NULL, 2, &PIDTaskHandle, 1);
  xTaskCreatePinnedToCore(emergencyBreakTask, "emergencyBreakTask", 3072, NULL, 2, &emergencyBreakTaskHandle, 1);

  set_position(66, 2320, deg_to_rad(-90), deg_to_rad(-90));

  Serial.println("Robot initialized.");
  Serial.println("Will run one command: go_to(66, 1895, 550, 1)");
  Serial.println("Place obstacle in front during motion to trigger emergency break.");
}

void loop()
{
  if (!test_started)
  {
    test_started = true;
    strategy_start_time = millis();

    Serial.println("[TEST] Starting go_to...");
    int result = go_to(66, 1895, 550, 1);

    Serial.printf("[TEST] go_to result = %d\n", result);
    Serial.printf("[TEST] wheel_state_R=%d, wheel_state_L=%d\n", wheel_state_R, wheel_state_L);
    Serial.printf("[TEST] Final pose: x=%.1f, y=%.1f, phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));

    if (wheel_state_R == EMERGENCY_BREAK || wheel_state_L == EMERGENCY_BREAK)
    {
      Serial.println("[TEST] Emergency break TRIGGERED.");
    }
    else
    {
      Serial.println("[TEST] Emergency break NOT triggered.");
    }

    stop_motors();
    test_finished = true;
  }

  if (test_finished)
  {
    custom_delay(200);
  }
}
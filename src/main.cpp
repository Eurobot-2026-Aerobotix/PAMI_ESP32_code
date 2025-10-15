#include <Arduino.h>
#include "HAL.h"
#include "odometry.h"
#include "navigation.h"
#include "command_controller.h"
#include "communication.h"

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

// Custom strategy implementation
void strategy()
{
  Serial.println("PAMI Robot Strategy Started!");

  // Example strategy implementation:
  // 1. Check crate colors
  Serial.println("Current crate colors:");
  for (int i = 0; i < 8; i++)
  {
    Serial.print("Crate ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(crate_colors[i] ? "YELLOW" : "BLUE");
  }

  // 2. Execute some movement or action
  // Example: move forward, rotate, etc.
  // move_forward(100, 50); // Move 100mm at 50mm/s speed

  // 3. Send completion signal
  send_action_done(1);

  Serial.println("Strategy execution completed!");
}

void communicationTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay for responsive communication
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("Communication task started");

  // Setup micro-ROS
  if (setup_micro_ros("eurobot", "eurobot2026", "10.42.0.1", 8888))
  {
    Serial.println("micro-ROS setup successful");
  }
  else
  {
    Serial.println("micro-ROS setup failed");
  }

  for (;;) // Infinite loop - CRITICAL!
  {
    // Run the communication loop
    communication_loop();

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void PIDTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(10); // Increased to 10ms delay for stability
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("PID task started");

  // Initialize previous_speed_control_time
  previous_speed_control_time = micros();

  // Wait a bit before starting PID control
  delay(2000);

  for (;;) // Infinite loop - CRITICAL!
  {
    // Add error checking
    if (previous_speed_control_time == 0)
    {
      previous_speed_control_time = micros();
      vTaskDelayUntil(&xLastWakeTime, xDelay);
      continue;
    }

    update_position();

    float tmp = (micros() - previous_speed_control_time) / 1000000.0; // in seconds

    // Prevent division by zero or very small numbers
    if (tmp < 0.001)
    {
      vTaskDelayUntil(&xLastWakeTime, xDelay);
      continue;
    }

    // right motor speed control
    float error = target_speed_R - current_speed_R;

    speed_derivative_R = speed_derivative_R * 0.999 + 0.001 * (error - previous_speed_error_R) / tmp; // unit = mm/s²
    speed_integral_R += error * tmp;

    previous_speed_error_R = error;

    PWM_R = PWM_R * 0.2 + 0.8 * (PWM_offset_R + kpS_R * error + kdS_R * speed_derivative_R + kiS_R * speed_integral_R); // the first part is a simple filter to reduce jitter
    PWM_R = (PWM_R > PWM_max) ? PWM_max : (PWM_R < -PWM_max) ? -PWM_max
                                                             : PWM_R;
    PWM_R = (target_speed_R == 0) ? 0 : PWM_R;

    speed_integral_R = ((PWM_R == PWM_max) || (PWM_R == -PWM_max)) ? (speed_integral_R - 2 * error * tmp) : speed_integral_R;

    // left motor speed control
    error = target_speed_L - current_speed_L;
    speed_derivative_L = speed_derivative_L * 0.999 + 0.001 * (error - previous_speed_error_L) / tmp; // unit = mm/s²
    speed_integral_L += error * tmp;

    previous_speed_error_L = error;
    previous_speed_control_time = micros();

    PWM_L = PWM_L * 0.2 + 0.8 * (PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L);
    PWM_L = (PWM_L > PWM_max) ? PWM_max : (PWM_L < -PWM_max) ? -PWM_max
                                                             : PWM_L;
    PWM_L = (target_speed_L == 0) ? 0 : PWM_L;

    speed_integral_L = ((PWM_L == PWM_max) || (PWM_L == -PWM_max)) ? (speed_integral_L - 2 * error * tmp) : speed_integral_L;

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void commandExecutionTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("Command execution task started");

  for (;;) // Infinite loop - CRITICAL!
  {
    execute_command();
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void setup()
{
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000); // Give time for serial to initialize
  Serial.println("=== PAMI ESP32 Setup ===");

  Serial.println("Setting motor pins...");
  set_motor_pins(22, 23, 19, 18);

  Serial.println("Setting encoder channels...");
  set_encoder_channels(2, 3, 18, 19, 1440);

  Serial.println("Setting PWM parameters...");
  set_PWM_min_max(255, 30, 30, 30, 30);

  Serial.println("Setting wheel parameters...");
  set_wheel_diameter_and_spacing(0.1, 0.1, 0.2);

  HAL_setup();

  delay(1000);

  Serial.println("Creating tasks...");

  // Create tasks with lower stack sizes first
  xTaskCreatePinnedToCore(
      communicationTask,
      "communicationTask",
      8192,
      NULL,
      1, // Lower priority
      &communicationTaskHandle,
      0); // Pin to core 0

  delay(500); // Delay between task creation
  Serial.println("Communication task created");

  xTaskCreatePinnedToCore(
      commandExecutionTask,
      "commandExecutionTask",
      4096, // Back to smaller stack
      NULL,
      1, // Lower priority
      &commandExecutionTaskHandle,
      0); // Pin to core 0

  delay(500); // Delay between task creation
  Serial.println("Command execution task created");

  // Create PID task last (it might be causing issues)
  xTaskCreatePinnedToCore(
      PIDTask,
      "PIDTask",
      4096, // Back to smaller stack
      NULL,
      2, // Medium priority
      &PIDTaskHandle,
      1); // Pin to core 1

  Serial.println("All tasks created successfully!");
  delay(1000);
}

void loop()
{
  // Keep loop empty - tasks handle everything
  delay(1000);
}
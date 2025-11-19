
/*
 *
 *            ___ _   _ ___  ___  ___  ___ _____   ___ __ ___ ___
 *           | __| | | | _ \/ _ \| _ )/ _ \_   _| |_  )  \_  ) __/
 *           | _|| |_| |   / (_) | _ \ (_) || |    / / () / /| _ \
 *           |___|\___/|_|_\\___/|___/\___/ |_|   /___\__/___\___/
 *
 *
 *                 EUROBOT 2026 - AEROBOTIX | INSAT Tunisia
 *                        OFFICIAL SIMA CODE
 *
 */

#include <Arduino.h>
#include "navigation.h"
#include "command_controller.h"
#include "communication.h"
#include "websocket.h"

/// tasks
TaskHandle_t communicationTaskHandle;
TaskHandle_t PIDTaskHandle;
TaskHandle_t commandExecutionTaskHandle;
TaskHandle_t websocketTaskHandle;
int websocket=0;
int8_t strategy_waiting = 0;
// Note: websocket_enabled variable is defined in websocket.cpp

// Custom strategy implementation
void strategy()
{
  /*
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
  */

  // 2. Execute some movement or action
  // Example: move forward, rotate, etc.
  // move_forward(100, 50); // Move 100mm at 50mm/s speed
  custom_delay(500);
  go_to(300, 300, 300); // Go to (500, 0) at 300mm/s speed
  custom_delay(800);
  go_to(300, 0, 300);
  custom_delay(800);
  orientate(0, 0); // Orientate to 0 degrees
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
  const TickType_t xDelay = pdMS_TO_TICKS(1); // Increased to 10ms delay for stability
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("PID task started");

  // Initialize previous_speed_control_time
  previous_speed_control_time = micros();

  for (;;)
  {
    // Add error checking
    if (previous_speed_control_time == 0)
    {
      previous_speed_control_time = micros();
      vTaskDelayUntil(&xLastWakeTime, xDelay);
      continue;
    }


    // this is the compromise for calculating the speed in the tick count update ISR
    if ((micros() - speedCalculationPreviousTimeR) > 18000){
      current_speed_R = 0;
    }
    if ((micros() - speedCalculationPreviousTimeL) > 18000){
      current_speed_L = 0;
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
    float error = target_speed_R - current_speed_R_filtered;

    speed_derivative_R = speed_derivative_R * 0.999 + 0.001 * (error - previous_speed_error_R) / tmp; // unit = mm/s²
    speed_integral_R += error * tmp;

    previous_speed_error_R = error;

    PWM_R = PWM_R * 0.9 + 0.1 * (PWM_offset_R + kpS_R * error + kdS_R * speed_derivative_R + kiS_R * speed_integral_R); // the first part is a simple filter to reduce jitter
    PWM_R = (PWM_R > PWM_max) ? PWM_max : (PWM_R < -PWM_max) ? -PWM_max : PWM_R;
    PWM_R = (target_speed_R == 0) ? 0 : PWM_R;

    speed_integral_R = ((PWM_R == PWM_max) || (PWM_R == -PWM_max)) ? (speed_integral_R - 2 * error * tmp) : speed_integral_R;

    // left motor speed control
    error = target_speed_L - current_speed_L_filtered;
    speed_derivative_L = speed_derivative_L * 0.999 + 0.001 * (error - previous_speed_error_L) / tmp; // unit = mm/s²
    speed_integral_L += error * tmp;

    previous_speed_error_L = error;
    previous_speed_control_time = micros();

    PWM_L = PWM_L * 0.95 + 0.05 * (PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L);
    PWM_L = (PWM_L > PWM_max) ? PWM_max : (PWM_L < -PWM_max) ? -PWM_max : PWM_L;
    PWM_L = (target_speed_L == 0) ? 0 : PWM_L;

    speed_integral_L = ((PWM_L == PWM_max) || (PWM_L == -PWM_max)) ? (speed_integral_L - 2 * error * tmp) : speed_integral_L;
    run_right_motor();
    run_left_motor();

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
    if (strategy_waiting) {
      strategy_waiting = 0;
      strategy();
    }
    execute_command();
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void websocketTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay for WebSocket operations
  TickType_t xLastWakeTime = xTaskGetTickCount();
  Serial.println("WebSocket task started");
  
  // Wait a bit for system initialization
  vTaskDelay(pdMS_TO_TICKS(3));
  
  for (;;) // Infinite loop - CRITICAL!
  {
    if (websocket == 1) {
      // Handle reconnections and polling
      websocket_reconnect_if_needed();
      websocket_poll();
    
      // Send robot data via WebSocket at 10Hz (every 100ms)
      static TickType_t last_data_send = 0;
      if ((xTaskGetTickCount() - last_data_send) >= pdMS_TO_TICKS(100)) {
        last_data_send = xTaskGetTickCount();
        if (websocket_has_new_string()) {
        String receivedMsg = websocket_get_last_string();
        Serial.println("Processing: " + receivedMsg);
    // String is automatically processed by processReceivedString()
}
        if (websocket_is_connected()) {
          // Send current robot data
          /*
          websocket_send_data(0xAA, current_speed_R_filtered);
          websocket_send_data(0xAB, current_speed_L_filtered);
          websocket_send_data(0xAC, target_speed_R);
          websocket_send_data(0xAD, target_speed_L);
          */
          websocket_send_data(0xAA, current_x);
          websocket_send_data(0xAB, current_y);
          websocket_send_data(0xAC, rad_to_deg(current_phi));
          websocket_send_data(0xAD, rad_to_deg(absolute_phi));
        }
      }
    }
    
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void setup()
{
  // Initialize Serial for debugging
  Serial.begin(115200);

  Serial.println("=== PAMI ESP32 Setup ===");

  Serial.println("Setting motor pins...");
  set_motor_pins(19, 18, 21, 22, 17, 23);

  Serial.println("Setting encoder channels...");
  set_encoder_channels(15, 2, 16, 4, 400);

  Serial.println("Setting PWM parameters...");
  set_PWM_min_max(1023, 320, 320, -320, -320);

  Serial.println("Setting wheel parameters...");
  set_wheel_diameter_and_spacing(25.7393438962633966203, 44.72738447547409904512, 88.83238033718186); // howa la3jal 9add 9add ama wa7da lencodeur te3ha fih 700tck donc bech mano93odch na3mel fi CPR lkoll 3ajla hayka keka te5dem chihemmek fi waldeha

  HAL_setup();
  
  // Initialize WebSocket if enabled
  extern int websocket_enabled;
  websocket_enabled = websocket;  // Set from the main websocket variable
  if (websocket_enabled == 1) {
    Serial.println("Initializing WebSocket...");
    websocket_init();
    delay(1000);
  }


  Serial.println("Creating tasks...");

  // Create tasks with lower stack sizes first
  // xTaskCreatePinnedToCore(
  //     communicationTask,
  //     "communicationTask",
  //     8192,
  //     NULL,
  //     1, // Lower priority
  //     &communicationTaskHandle,
  //     0); // Pin to core 0

  // delay(500); // Delay between task creation
  // Serial.println("Communication task created");

  xTaskCreatePinnedToCore(
      commandExecutionTask,
      "commandExecutionTask",
      4096, // Back to smaller stack
      NULL,
      1, // Lower priority
      &commandExecutionTaskHandle,
      0); // Pin to core 0
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
  Serial.println("PID task created");

  // Create WebSocket task if enabled
  if (websocket_enabled == 1) {
    xTaskCreatePinnedToCore(
        websocketTask,
        "websocketTask",
        8192, // Larger stack for WebSocket operations
        NULL,
        1, // Lower priority than PID
        &websocketTaskHandle,
        0); // Pin to core 0 (same as other communication tasks)
    
    delay(500); // Delay between task creation
    Serial.println("WebSocket task created");
  }

  Serial.println("All tasks created successfully!");
  strategy_waiting = 1;
}

void loop(){
}
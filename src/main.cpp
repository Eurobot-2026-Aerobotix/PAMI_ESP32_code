
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
TaskHandle_t emergencyBreakTaskHandle;
int websocket=0;

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
  //custom_delay(500);
  int goto_cmd = 0, orientate_cmd = 1, set_position_cmd = 2;
   int commands[] = {//goto_cmd, 500, 0, 300, // Go to (0, 300) at 500mm/s speed
  //                   goto_cmd, 0, 0, 300,
  //                   goto_cmd, 0, 0, 300,
  //                   orientate_cmd, 0, 0,
                    // goto_cmd, 1100, 250, 300, // Go to (0, 0) at 500mm/s speed
                    // goto_cmd, 1540, -70, 300,
                    // goto_cmd, 1595, 415, 200,
                    ////////
                    // set_position_cmd, 337, 149, 0,
                    // goto_cmd, 1200, 350, 600,
                    // goto_cmd, 1955, 140, 500,
                    // goto_cmd, 1935, 635, 300,
                    }; // Orientate to 0 degrees
  int num_commands = sizeof(commands) / sizeof(commands[0]);
  int i = 0;
  strategy_start_time = millis();
  while (i < num_commands)
  {
    int command_type = commands[i];
    if (command_type == goto_cmd)
    {
      int x = commands[i + 1];
      int y = commands[i + 2];
      int speed = commands[i + 3];
      while(go_to(x, y, speed)){
        wait_for_obstacle_clearance();
        Serial.println("obstacle cleared, resuming...");
      }
      i += 4;
    }
    else if (command_type == orientate_cmd)
    {
      int angle = commands[i + 1];
      int is_radian = commands[i + 2];
      orientate(angle, is_radian);
      i += 3;
    }
    else if (command_type == set_position_cmd) { 
      int x = commands[i + 1]; 
      int y = commands[i + 2]; 
      int phi = commands[i + 3]; 
      set_position(x, y, phi, phi); 
      i += 4; 
    }
  }
  // 3. Send completion signal
  send_action_done(1);

  Serial.println("Strategy execution completed!");
}

void communicationTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay for responsive communication
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("Communication task started");
  
  // Setup micro-ROS (will handle WiFi connection internally)
  if (setup_micro_ros("harissa", "harissaharissa2026", "10.42.1.1", 8888))
  {
    Serial.println("micro-ROS setup successful");
  }
  else
  {
    Serial.println("micro-ROS setup failed");
  }
  
  for (;;) // Infinite loop - CRITICAL!
  {
    // Only run communication loop if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
      communication_loop();
    } else {
      // If WiFi disconnected, try to reconnect
      Serial.println("WiFi disconnected in task, attempting reconnection...");
      connect_wifi("harissa", "harissaharissa2026");
      vTaskDelay(pdMS_TO_TICKS(5000)); // Wait 5 seconds before next attempt
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void emergencyBreakTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(30);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for (;;)
  {
    if (( ((get_ultrasonic_distance() < min_distance && trajectory == STRAIGHT) || (millis() - strategy_start_time >= max_running_time)) && wheel_state_R != EMERGENCY_BREAK && wheel_state_L != EMERGENCY_BREAK) ) 
    {
      prepare_emergency_break();
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void PIDTask(void *pvParameters){
  const TickType_t xDelay = pdMS_TO_TICKS(1); // Increased to 10ms delay for stability
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("PID task started");

  // Initialize previous_speed_control_time
  previous_speed_control_time = micros();

  for (;;){
    // Add error checking
    if (previous_speed_control_time == 0){
      previous_speed_control_time = micros();
      vTaskDelayUntil(&xLastWakeTime, xDelay);
      continue;
    }

    update_position();

    // right motor speed control
    if (control_loop_state == SPEED){
      float dt = (micros() - previous_speed_control_time) / 1000000.0; // in seconds

      // Prevent division by zero or very small numbers
      if (dt < 0.001)
      {
        vTaskDelayUntil(&xLastWakeTime, xDelay);
        continue;
      }

      float error = target_speed_R - current_speed_R;

      speed_derivative_R = speed_derivative_R * 0.999 + 0.001 * (error - previous_speed_error_R) / dt; // unit = mm/s²
      speed_integral_R += error * dt;

      previous_speed_error_R = error;

      PWM_R = PWM_R * 0.0 + 1.0 * (PWM_offset_R + kpS_R * error + kdS_R * speed_derivative_R + kiS_R * speed_integral_R); // the first part is a simple filter to reduce jitter
      PWM_R = (PWM_R > PWM_max) ? PWM_max : (PWM_R < -PWM_max) ? -PWM_max : PWM_R;
      PWM_R = (target_speed_R == 0) ? 0 : PWM_R;

      speed_integral_R = ((PWM_R == PWM_max) || (PWM_R == -PWM_max)) ? (speed_integral_R - 2 * error * dt) : speed_integral_R;

      // left motor speed control
      error = target_speed_L - current_speed_L;
      speed_derivative_L = speed_derivative_L * 0.999 + 0.001 * (error - previous_speed_error_L) / dt; // unit = mm/s²
      speed_integral_L += error * dt;

      previous_speed_error_L = error;
      previous_speed_control_time = micros();

      PWM_L = PWM_L * 0.95 + 0.05 * (PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L);
      PWM_L = (PWM_L > PWM_max) ? PWM_max : (PWM_L < -PWM_max) ? -PWM_max : PWM_L;
      PWM_L = (target_speed_L == 0) ? 0 : PWM_L;

      speed_integral_L = ((PWM_L == PWM_max) || (PWM_L == -PWM_max)) ? (speed_integral_L - 2 * error * dt) : speed_integral_L;
    }
    else if (control_loop_state == POSITION){
      float dt = (micros() - previous_position_control_time)/1000000.0;

      if (dt < 0.001)
      {
        vTaskDelayUntil(&xLastWakeTime, xDelay);
        continue;
      }
      // right motor position control
      float error = (target_encoder_count_R - encoder_count_R)*2; 

      position_derivative_R = (error - previous_position_error_R)*dt; // unit = tick/s²

      position_integral_R = ( SGN(error) == SGN( previous_position_error_R ) )? position_integral_R + error*dt : 0;

      previous_position_error_R = error;

      PWM_R = kpP_R*error + kdP_R*position_derivative_R + kiP_R*position_integral_R; // the first part is a simple filter to reduce jitter
      PWM_R = (PWM_R > PWM_R_forward_min + position_max_response)? PWM_R_forward_min + position_max_response : (PWM_R < PWM_R_backward_min - position_max_response)? PWM_R_backward_min - position_max_response : PWM_R;

      position_integral_R = ( (PWM_R == PWM_max) || (PWM_R == -PWM_max) )? (position_integral_R - 2*error*dt) : position_integral_R;

      // left motor position control
      error = (target_encoder_count_L - encoder_count_L);

      position_derivative_L = (error - previous_position_error_L)*dt; // unit = mm/s²
      position_integral_L = ( SGN(error) == SGN( previous_position_error_L) )? position_integral_L + error*dt : 0;

      previous_position_error_L = error;

      PWM_L = kpP_L*error + kdP_L*position_derivative_L + kiP_L*position_integral_L; // the first part is a simple filter to reduce jitter
      PWM_L = (PWM_L > PWM_L_forward_min + position_max_response)? PWM_L_forward_min + position_max_response : (PWM_L < PWM_L_backward_min - position_max_response)? PWM_L_backward_min - position_max_response : PWM_L;

      position_integral_L = ( (PWM_L == PWM_max) || (PWM_L == -PWM_max) )? (position_integral_L - 2*error*dt) : position_integral_L;

      previous_position_error_L = error;
      previous_position_control_time = micros();
    }
    // PWM_R = 300;
    // PWM_L = 300;
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
          websocket_send_data(0xAA, current_speed_L);
          websocket_send_data(0xAB, current_speed_R);
          websocket_send_data(0xAC, target_speed_L);
          websocket_send_data(0xAD, target_speed_R);
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
  set_motor_pins(22, 26, 19, 18, 23, 17);

  set_ultrasonic_pins(14, 12);

  Serial.println("Setting encoder channels...");
  set_encoder_channels(4, 16, 15, 2, 350);

  Serial.println("Setting PWM parameters...");
  set_PWM_min_max(1023, 240, 210, -240, -210);

  Serial.println("Setting wheel parameters...");
  set_wheel_diameter_and_spacing(57.844475682412635014651419709637, 57.784471039588555517748254336494, 92.938153961874487863117321276497);
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

  //Create tasks with lower stack sizes first
  //   xTaskCreatePinnedToCore(
  //    communicationTask,
  //     "communicationTask",
  //     8192,
  //     NULL,
  //     1, // Lower priority
  //      &communicationTaskHandle,
  //      0); // Pin to core 0

  //  delay(500); // Delay between task creation
  //  Serial.println("Communication task created");

  xTaskCreatePinnedToCore(
      commandExecutionTask,
      "commandExecutionTask",
      4096, // Back to smaller stack
      NULL,
      1, // Lower priority
      &commandExecutionTaskHandle,
      1); // Pin to core 0
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

  xTaskCreatePinnedToCore(
      emergencyBreakTask,
      "emergencyBreakTask",
      2048, // Stack size
      NULL,
      2, // High priority
      &emergencyBreakTaskHandle,
      1); // Pin to core 1

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
  control_loop_state = DISACTIVATED;
  delay(1000);
  strategy_waiting = 0;
  prepare_go_to(300, 0, 300);
  // prepare_rotate(1800, 0);
  //prepare_go_to(1000, 0, 300);
  // while(1){
  //   digitalWrite(21, HIGH);
  // }
  //prepare_go_to(500, 0, 300);
  //set_target_encoder_counts(0, 0);
}

void loop(){
  // for(int i=0; i<100; i++){
  //   set_target_speeds(i*2, 0);
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  // for(int i=0; i<200; i++){
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  // set_target_speeds(0, 0);
  // for(int i=0; i<200; i++){
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  // for(int i=0; i<100; i++){
  //   set_target_speeds(0, i*2);
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  // for(int i=0; i<200; i++){
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  // set_target_speeds(0, 0);
  // for(int i=0; i<200; i++){
  //   send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //   delay(10);
  // }
  send_values_to_usart(encoder_count_R, encoder_count_L, 0, 0);
  delay(10);
}
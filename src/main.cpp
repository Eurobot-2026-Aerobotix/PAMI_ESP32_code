
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

// External variables from communication
extern bool crate_colors[8];

/// tasks
TaskHandle_t communicationTaskHandle;
TaskHandle_t PIDTaskHandle;
TaskHandle_t commandExecutionTaskHandle;
TaskHandle_t websocketTaskHandle;
TaskHandle_t emergencyBreakTaskHandle;
int websocket=0;

void kill_comms();

static bool has_command_operands(int current_index, int required_count, int num_commands)
{
  return current_index >= 0 && required_count >= 0 && (current_index + required_count) <= num_commands;
}

// Custom strategy implementation
void strategy()
{
  kill_comms();

  // Calculate total commands: ROS commands + yellow/blue strategy commands
  
  int* commands = strategy_commands; // Assuming strategy_commands is filled with ROS commands
  int num_commands = strategy_length; // Assuming strategy_length is set to the number of ROS

  //  int commands[] = {
  //                 SET_POSITION, 0, 0, 0,
  //                 GOTO, 1000, 0, 200, 1,
  // //               //   GOTO, 200, 200, 350, 1,
  // //               //  GOTO, 400, 200, 350, 1,
  // //               //     GOTO, 400, 400, 350, 1,
  // //               //     GOTO, 600, 400, 350, 1,
  // //               //      GOTO, 600, 600, 350, 1,
  // //               //   GOTO, 600, 400, 350, -1,
  // //               //   GOTO, 400, 400, 350, -1,


  //                 // SET_POSITION, 0, 0, 0,
  //                 // GOTO, 700, 0, 200, 1,
  //                 // PICKUP, 1, 1, 1,
  //                 // GOTO, 0, 0, 200, 1,
  //                 // ORIENTATE, 0,
  //                 // PUTDOWN, 1, 1,
  //                  }; // Example commands: go to (1000, 1000) at speed 100, then orientate to 90 degrees
  // int num_commands = sizeof(commands) / sizeof(commands[0]);

  int i = 0;
  strategy_start_time = millis();
  // rotating_speed = 150;
  // rotate(3600, 0);
  // while(1){custom_delay(10);};
  while (i < num_commands)
  {
    int command_type = commands[i];
    if (command_type == GOTO)
    {
      if (!has_command_operands(i, 5, num_commands)) {
        Serial.println("[ERROR] Truncated GOTO command in strategy buffer");
        break;
      }
      int x = commands[i + 1];
      int y = commands[i + 2];
      int speed = commands[i + 3];
      int direction = commands[i + 4];
     // Serial.printf("[CMD] goto_cmd: x=%d, y=%d, speed=%d, direction=%d\n", x, y, speed, direction);
      while(go_to(x, y, speed, direction)){
        wait_for_obstacle_clearance();
        Serial.println("obstacle cleared, resuming...");
        Serial.println("mahdi y7ebb louled");
      }
     // Serial.printf("[POS] After goto: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 5;
    }
    else if (command_type == ORIENTATE)
    {
      if (!has_command_operands(i, 2, num_commands)) {
        Serial.println("[ERROR] Truncated ORIENTATE command in strategy buffer");
        break;
      }
      int angle = commands[i + 1];
      orientate(angle, 0);
      i += 2;
    }
    else if (command_type == SET_POSITION) { 
      if (!has_command_operands(i, 4, num_commands)) {
        Serial.println("[ERROR] Truncated SET_POSITION command in strategy buffer");
        break;
      }
      int x = commands[i + 1];
      int y = commands[i + 2];
      int phi = commands[i + 3];
      PWM_R = 0;
      PWM_L = 0;
      stop_motors();
      control_loop_state = DISACTIVATED;
      custom_delay(300);
      set_position(x, y, deg_to_rad(phi), deg_to_rad(phi));
      i += 4; 
    }else if (command_type == WAIT_FOR) {
      if (!has_command_operands(i, 2, num_commands)) {
        Serial.println("[ERROR] Truncated WAIT_FOR command in strategy buffer");
        break;
      }
      int flag = commands[i + 1];
      while(millis() - strategy_start_time < flag){
        custom_delay(5);
      }
      i += 2;
    }else if (command_type == BACKWARDS) {
      if (!has_command_operands(i, 6, num_commands)) {
        Serial.println("[ERROR] Truncated BACKWARDS command in strategy buffer");
        break;
      }
      int distance = commands[i + 1];
      int speed = commands[i + 2];
      int x = commands[i + 3];
      int y = commands[i + 4];
      int direction = commands[i + 5];
     // Serial.printf("[CMD] forward_cmd: distance=%d, speed=%d, x=%d, y=%d, direction=%d\n", distance, speed, x, y, direction);
      move_forward(distance,speed, x, y,direction);
      i += 6;
    }
    else if (command_type == PICKUP){
      if (!has_command_operands(i, 4, num_commands)) {
        Serial.println("[ERROR] Truncated PICKUP command in strategy buffer");
        break;
      }
    //  Serial.printf("[CMD] pickup_cmd: arg1=%d, arg2=%d, arg3=%d\n", commands[i + 1], commands[i + 2], commands[i + 3]);
      pickup(commands[i + 1], commands[i + 2], commands[i + 3]);
    //  Serial.printf("[POS] After pickup: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 4;
    }
    else if (command_type == PUTDOWN){
      if (!has_command_operands(i, 3, num_commands)) {
        Serial.println("[ERROR] Truncated PUTDOWN command in strategy buffer");
        break;
      }
      int bechwaya = commands[i+1];
      int nid_lot = commands[i+2];
      Serial.println("[CMD] putdown_cmd");
      putdown(bechwaya, nid_lot);
      i += 3;
    }
    else if (command_type == WALKING_POSITION){
      if (!has_command_operands(i, 2, num_commands)) {
        Serial.println("[ERROR] Truncated WALKING_POSITION command in strategy buffer");
        break;
      }
      int position = commands[i + 1];
      Serial.println("[CMD] walking_position_cmd");
      walking_position(position);
      i += 2;
    }
    else if (command_type == PREPARE){
      Serial.println("[CMD] prepare_cmd");
      prepare_servo_up();
      i++;
    }
    else if (command_type == LETGO){
      Serial.println("[CMD] letgo_cmd");
      letgo();
      i++;
    }
    else if (command_type == BEZIER){
      if (!has_command_operands(i, 9, num_commands)) {
        Serial.println("[ERROR] Truncated BEZIER command in strategy buffer");
        break;
      }
      int x1 = commands[i + 1];
      int y1 = commands[i + 2];
      int x2 = commands[i + 3];
      int y2 = commands[i + 4];
      int x3 = commands[i + 5];
      int y3 = commands[i + 6];
      int speed = commands[i + 7];
      int direction = commands[i + 8];
      while(bezier_curve(x1, y1, x2, y2, x3, y3, speed, direction)){
        wait_for_obstacle_clearance();
        Serial.println("obstacle cleared, resuming...");
      }
      i += 9;
    }
    else if (command_type == DELAY){
      if (!has_command_operands(i, 2, num_commands)) {
        Serial.println("[ERROR] Truncated DELAY command in strategy buffer");
        break;
      }
      int delay_time = commands[i + 1];
   //   Serial.printf("[CMD] delay_cmd: custom_delay(%d)\n", delay_time);
      custom_delay(delay_time);
      i += 2;
    }
    else
    {
      Serial.print("[ERROR] Unknown strategy opcode: ");
      Serial.println(command_type);
      break;
    }
  }
  // control_loop_state = DISACTIVATED;
	// stop_motors();
  // 3. Send completion signal

  Serial.println("Strategy execution completed!");
}

void communicationTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(50); // 50ms delay for responsive communication
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("Communication task started");
  
  // Setup micro-ROS (will handle WiFi connection internally)
  if (setup_micro_ros("rouge", "t0rb@g26", "192.168.31.2", 8888))
  {
    Serial.println("micro-ROS setup successful");
  }
  else
  {
    Serial.println("micro-ROS setup failed");
  }
  
  for (;;) // Infinite loop - CRITICAL!
  {
    if (comm_stop_requested)
    {
      Serial.println("Communication stop requested, cleaning up micro-ROS...");
      cleanup_micro_ros();
      communicationTaskHandle = NULL;
      vTaskDelete(NULL);
    }

    // Only run communication loop if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
      communication_loop();
    } else {
      // If WiFi disconnected, try to reconnect
      Serial.println("WiFi disconnected in task, attempting reconnection...");
      connect_wifi("rouge", "t0rb@g26");
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
    if (( (( (!ultrasonic_blocked) && get_ultrasonic_distance() < min_distance && trajectory == STRAIGHT) || (millis() - strategy_start_time >= max_running_time)) && wheel_state_R != EMERGENCY_BREAK && wheel_state_L != EMERGENCY_BREAK) ) 
    {
     prepare_emergency_break();
    }
      //send_values_to_usart(encoder_count_L, encoder_count_R, current_phi*100,0);

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void emergencyBreakTaskWithoutUltrason(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(30);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    if ((millis() - strategy_start_time >= max_running_time) &&
        wheel_state_R != EMERGENCY_BREAK &&
        wheel_state_L != EMERGENCY_BREAK)
    {
      prepare_emergency_break();
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void PIDTask(void *pvParameters){
  const TickType_t xDelay = pdMS_TO_TICKS(1);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  Serial.println("PID task started");

  // Initialize previous_speed_control_time
  previous_speed_control_time = micros();
  float dt = 0.001;

  for (;;){

    update_position(dt);

    // right motor speed control
    if (control_loop_state == SPEED){
      // Prevent division by zero or very small numbers

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

      PWM_L = PWM_L * 0.0 + 1.0 * (PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L);
      PWM_L = PWM_L * 0.0 + 1.0 * (PWM_offset_L + kpS_L * error + kdS_L * speed_derivative_L + kiS_L * speed_integral_L);
      PWM_L = (PWM_L > PWM_max) ? PWM_max : (PWM_L < -PWM_max) ? -PWM_max : PWM_L;
      PWM_L = (target_speed_L == 0) ? 0 : PWM_L;

      speed_integral_L = ((PWM_L == PWM_max) || (PWM_L == -PWM_max)) ? (speed_integral_L - 2 * error * dt) : speed_integral_L;
    }
    else if (control_loop_state == POSITION){
      // right motor position control
      float error = (target_encoder_count_R - encoder_count_R); 

      position_derivative_R = (error - previous_position_error_R)/dt; // unit = tick/s²

      position_integral_R = ( SGN(error) == SGN( previous_position_error_R ) )? position_integral_R + error*dt : 0;

      previous_position_error_R = error;

      PWM_R = kpP_R*error + kdP_R*position_derivative_R + kiP_R*position_integral_R; // the first part is a simple filter to reduce jitter
      PWM_R = (PWM_R > PWM_R_forward_min + position_max_response)? PWM_R_forward_min + position_max_response : (PWM_R < PWM_R_backward_min - position_max_response)? PWM_R_backward_min - position_max_response : PWM_R;

      position_integral_R = ( (PWM_R == PWM_max) || (PWM_R == -PWM_max) )? (position_integral_R - 2*error*dt) : position_integral_R;

      // left motor position control
      error = (target_encoder_count_L - encoder_count_L);

      position_derivative_L = (error - previous_position_error_L)/dt; // unit = mm/s²
      position_integral_L = ( SGN(error) == SGN( previous_position_error_L) )? position_integral_L + error*dt : 0;

      previous_position_error_L = error;

      PWM_L = kpP_L*error + kdP_L*position_derivative_L + kiP_L*position_integral_L; // the first part is a simple filter to reduce jitter
      PWM_L = (PWM_L > PWM_L_forward_min + position_max_response)? PWM_L_forward_min + position_max_response : (PWM_L < PWM_L_backward_min - position_max_response)? PWM_L_backward_min - position_max_response : PWM_L;

      position_integral_L = ( (PWM_L == PWM_max) || (PWM_L == -PWM_max) )? (position_integral_L - 2*error*dt) : position_integral_L;

      previous_position_error_L = error;
    }
//set_target_speeds(600, 600); 
  //  delay(10);
    run_right_motor();
    run_left_motor();
   //send_values_to_usart(current_speed_R, current_speed_L, target_speed_R, target_speed_L);
  // delay(10);
  // update target speeds in case they were changed by the command execution task
//send_values_to_usart(encoder_count_R,encoder_count_L, 0, 0); // send encoder counts for debugging
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
      // custom_delay(85000);
      strategy();
    }
    execute_command();
    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void kill_comms(){
  if (communicationTaskHandle == NULL) {
    return;
  }

  comm_stop_requested = true;

  TickType_t start_time = xTaskGetTickCount();
  while (communicationTaskHandle != NULL && (xTaskGetTickCount() - start_time) < pdMS_TO_TICKS(500)) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (communicationTaskHandle != NULL) {
    Serial.println("[WARN] Forcing communication task shutdown");
    vTaskDelete(communicationTaskHandle);
    communicationTaskHandle = NULL;
  }
}

 void setup()
{
  // Initialize Serial for debugging
  Serial.begin(115200);

  Serial.println("=== PAMI ESP32 Setup ===");

  Serial.println("Setting motor pins...");
  set_motor_pins(19, 18, 21, 22, 17, 23);  // shah 19, 18, 21, 22, 17, 23

  set_system_pins(13, 33, 26, 27);// example: electrovalve=26, pump_left=33, pump_right=32, servo=27

  set_tirettes_pins(35, 39);

  set_agent_led_pin(25);

  Serial.println("Setting encoder channels...");
  set_encoder_channels(32, 15, 4, 16 ,1400); // badalthom  2, 15, 4, 16, 

  Serial.println("Setting PWM parameters...");
  set_ultrasonic_pins(14, 34); // badalthom //14

  set_PWM_min_max(1023, 80, 80, -80, -80); //yess    //130  120

  Serial.println("Setting wheel parameters...");
  set_wheel_diameter_and_spacing(34.731840562040404, 34.6927423194805, 110.10888033901348);
  HAL_setup();

  Serial.println("Creating tasks...");
  
  // Create communication task
  xTaskCreatePinnedToCore(
      communicationTask,
      "communicationTask",
      12288,
      NULL,
      1, // Lower priority
      &communicationTaskHandle,
      0); // Pin to core 0

  delay(500); // Delay between task creation
  Serial.println("Communication task created");


  xTaskCreatePinnedToCore(
      commandExecutionTask,
      "commandExecutionTask",
      12288,
      NULL,
      1, // Lower priority
      &commandExecutionTaskHandle,
      0); // Pin to core 0
  Serial.println("Command execution task created");

  // Create PID task last (it might be causing issues)
  xTaskCreatePinnedToCore(
      PIDTask,
      "PIDTask",
      6144,
      NULL,
      2, // Medium priority
      &PIDTaskHandle,
      1); // Pin to core 1
  Serial.println("PID task created");

  // xTaskCreatePinnedToCore(
  //     emergencyBreakTaskWithoutUltrason,
  //     "emergencyBreakTask",
  //     2048, // Stack size
  //     NULL,
  //     2, // High priority
  //     &emergencyBreakTaskHandle,
  //     1); // Pin to core 1

  delay(500); // Delay between task creation
  Serial.println("WebSocket task created");

  Serial.println("All tasks created successfully!");
  control_loop_state = DISACTIVATED;
  delay(1000);
  strategy_waiting = 0;
  if (get_tirette_state() == 0){
    kill_comms();
    for(int i=0; i<2; i++){
      set_agent_led_on();
      custom_delay(300);
      set_agent_led_off();
      custom_delay(300);
    }
    is_yellow = get_color_switch_state();
    load_strategy(strategy_commands, &strategy_length, is_yellow);
    bool state = true;
    while(get_tirette_state() == 0){
      custom_delay(30);
      set_agent_led_state(state);
      state = !state;
    }
    set_agent_led_on();
    strategy_waiting = 1; //1 means default mode
  }
}
void loop(){
  // send_values_to_usart(control_loop_state, PWM_L, encoder_count_R, encoder_count_L);
  // custom_delay(30);
    // PWM_R=500;
    // PWM_L=500;
    // delay(2000);
   
    // PWM_R=-500;
    // PWM_L=-500;
    // delay(2000);
    
    // PWM_R=500;
    // PWM_L=0;
    // delay(2000);


    // PWM_R=0;
    // PWM_L=500;
    // delay(2000);
  //  control_loop_state = SPEED;
  //  for(int i=0; i<100; i++){
  //    set_target_speeds(i*2, i*2);

    //  send_values_to_usart(encoder_count_L, encoder_count_R, target_speed_L, target_speed_R);
    //  custom_delay(30);
  //  }
  //  for(int i=0; i<200; i++){
  //    send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //    delay(10);
  //    }
  //  set_target_speeds(0, 0);
  //  for(int i=0; i<200; i++){
  //    send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //  delay(10);
  //  }
  //  reset_speed_control_variables();

  // send_values_to_usart(current_speed_L, current_speed_R, fabs(PWM_L)/(MAX(fabs(current_speed_L), 10))*100, target_speed_R);
  // delay(10);
  // send_values_to_usart(current_speed_R, current_speed_L, target_speed_R, target_speed_L);
  // delay(10);
  // send_values_to_usart(debugger*100, 0, target_speed_R, target_speed_L); // send encoder counts and position for debugging
  // send_values_to_usart(encoder_count_R, encoder_count_L, 0, 0); // send encoder counts and position for debugging
  // delay(20);
  // send_values_to_usart(current_x, current_y, current_phi * 100, 0);
  // delay(10);
}


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
// Custom strategy implementation
void strategy(){
  Serial.println("PAMI Robot Strategy Started!");

  Serial.println("Current crate colors:");
  for (int i = 0; i < 8; i++){
    Serial.print("Crate ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(crate_colors[i] ? "YELLOW" : "BLUE");
  }
  

  int goto_cmd = 0, orientate_cmd = 1, set_position_cmd = 2, pickup_cmd = 3, putdown_cmd = 4, walking_position_cmd = 5, prepare_cmd = 6 , letgo_cmd = 7, go_backward_cmd = 8, forward_cmd = 9, move_linear_cmd = 10, rotate_cmd = 11, party_time_cmd = 12, fix_x_cmd = 13, delay_cmd = 14, print_analog12_cmd = 15, bezier_cmd = 16; 

  static const int master_cina_yellow[] = {  
        // *************** strategy with  up crates **************************

                              set_position_cmd, 66, 680, 90,
                               goto_cmd, 66, 1100, 400, 1,
                              goto_cmd, 150, 1100, 400, 1,
                               pickup_cmd, 1, 1, 0,    // pick up 1 close
                              goto_cmd, 65, 668, 400, 1,
                               orientate_cmd, -90, 0,
                              // bezier_cmd , 0, 1000, 65, 1000, 65, 668, 300, 1,
                              putdown_cmd, 1, 1,      // drop 1 close
                               goto_cmd, 80, 770, 400, -1,
                               goto_cmd, -40, 770, 400, -1, 
                              set_position_cmd, 64, 770, 0, // homing 2'
                               goto_cmd, 175, 770, 300, 1,
                              pickup_cmd, 1, 1, 1, // pick up 2 niggas
                              goto_cmd, 130, 770, 400, -1,
                              goto_cmd, 86, 686, 400, -1,
                              orientate_cmd, 120, 0,
                              orientate_cmd, -90, 0,
                              putdown_cmd, 0, 1, // drop niggas over close ones
                               goto_cmd, 80, 737, 400,-1,
                               goto_cmd, -45, 737, 400,-1,
                               set_position_cmd, 30, 737, 0, // homing 2' 
                              goto_cmd, 170, 737, 400, 1,
                               pickup_cmd, 1, 1, 0, // pick up 2 yellow  
                                  goto_cmd, -60, 737, 300, -1,
                               set_position_cmd, 25, 737, 0, // homing 2'
                              goto_cmd, 55, 737, 400, 1,
                               goto_cmd, 55, 1160, 400, 1,
                              goto_cmd, 335, 1160, 400, 1,
                               putdown_cmd, 1, 0,     // drop yellows
                              goto_cmd, -40, 1170, 400, -1,
                               set_position_cmd, 25, 1170, 0, // homing 2 
                              goto_cmd, 40, 1170, 400, 1,
                               goto_cmd, 40, 1340, 400, 1,
                              goto_cmd, 80, 1340, 400, 1,
                             pickup_cmd, 1, 1, 0, // pick up 2 far
                              goto_cmd, -40, 1340, 400, -1,
                               set_position_cmd, 25, 1340, 0, // homing 2 
                               goto_cmd, 50, 1340, 400, 1,
                              goto_cmd, 100, 700, 400, -1,
                               goto_cmd, 310, 700, 400, -1,
                              orientate_cmd, -90, 0,
                              goto_cmd,310, 600,300,1,
                              putdown_cmd, 1, 1,      //drop 2 far
                              set_position_cmd,310,672,-90,
                               //set_position_cmd, 310, 672, 90, // homing 3
                               goto_cmd, 310, 1210, 400, -1,
                               goto_cmd, 310,1190, 400, 1,
                               goto_cmd, 40,1190, 400, -1,
                               set_position_cmd, 0,1190, 0,
                               goto_cmd, 20,1190, 400, 1,
                               goto_cmd, 20, 1250, 400, 1,
                               goto_cmd, 380, 1250, 400, 1,
                               goto_cmd, 340, 1250, 400, -1,
                               goto_cmd, 550, 1250, 400, 1,
                               orientate_cmd, 40, 0, 
                              party_time_cmd










        // *************** strategy without up crates **************************
                              // set_position_cmd, 66, 680, 90,
                              // goto_cmd, 66, 1105, 550, 1,
                              // goto_cmd, 160, 1105, 400, 1,
                              // pickup_cmd, 1, 1, 0,    // pick up 1 close
                              // goto_cmd, 80, 668, 550, 1,
                              // orientate_cmd, -90, 0,
                              // putdown_cmd, 1, 1,  // drop 1 close
                              // goto_cmd, 80, 771, 400, -1,
                              // orientate_cmd, 0, 0,
                              // goto_cmd, 160, 708, 300, 1,
                              // walking_position_cmd, 0,
                              // orientate_cmd, 20, 0,      // push niggas aside
                              // goto_cmd,180, 715, 300, 1,
                              // goto_cmd, 105, 740, 300, -1,
                              // walking_position_cmd, 1,
                              // goto_cmd, 105, 620, 300, -1,
                              // set_position_cmd, 105, 672, 90, // homing 2'
                              // goto_cmd, 105, 740, 300, 1,
                              // walking_position_cmd, 0,
                              // goto_cmd, 205, 740, 400, 1,
                              // pickup_cmd, 1, 1, 0, // pick up 2 yellow  
                              // goto_cmd, 20, 740, 200, -1,
                              // set_position_cmd, 25, 740, 0, // homing 3'
                              // goto_cmd, 65, 740, 400, 1,
                              // goto_cmd, 65, 1200, 400, -1,
                              // goto_cmd, 330, 1200, 500, 1,
                              // putdown_cmd, 1, 0,     // drop yellows
                              // goto_cmd,0 , 1200, 250, -1,
                              // set_position_cmd, 25, 1200, 0, // homing 4
                              // goto_cmd, 35, 1200, 400, 1,
                              // goto_cmd, 40, 1365, 500, 1,
                              // goto_cmd, 80, 1365, 500, 1,
                              // pickup_cmd, 1, 1, 0, // pick up 2 far
                              // goto_cmd, -10, 1365, 400, -1,
                              // set_position_cmd, 25, 1365, 0, // homing 5
                              // goto_cmd, 50, 1365, 400, 1,
                              // bezier_cmd ,100, 720, 100, 720, 310, 700, 300, 1,
                              // goto_cmd, 310, 580, 500, 1,
                              // putdown_cmd, 1, 1,      //drop 2 far
                              // set_position_cmd, 310, 668, -90, // homing 6
                              // goto_cmd, 310, 1200, 200, -1,
                              // goto_cmd, 310, 1100, 500, 1,
                              // walking_position_cmd, 0,
                              // goto_cmd, 310, 1220, 500, 1, // throwing niggas
                              // pickup_cmd, 1, 1, 0,
                              // putdown_cmd, 1, 1,
                              // goto_cmd, 310, 1170, 500, -1,
                              // walking_position_cmd, 1,
                              // goto_cmd, 120, 1170, 400, 1,
                              // goto_cmd, 120, 1300, 400, 1,
                              // goto_cmd, 380, 1300, 200, 1,
                              // goto_cmd, -60, 1300, 500, -1,
                              // set_position_cmd, 10, 1300, 0,
                              // goto_cmd, 30, 1300, 500, 1,
                              // goto_cmd, 370, 1750, 500, 1,
                              // party_time_cmd
                            };
  static const int master_cina_blue[] =   {
  set_position_cmd, 66, 2320, -90,
  goto_cmd, 66, 1895, 550, 1,
  goto_cmd, 160, 1895, 400, 1,
  pickup_cmd, 1, 1, 0,
  goto_cmd, 80, 2332, 550, 1,
  orientate_cmd, 90, 0,
  putdown_cmd, 1, 1,
  goto_cmd, 80, 2229, 400, -1,
  orientate_cmd, 0, 0,
  goto_cmd, 160, 2292, 300, 1,
  walking_position_cmd, 0,
  orientate_cmd, -20, 0,
  goto_cmd, 180, 2285, 300, 1,
  goto_cmd, 105, 2260, 300, -1,
  walking_position_cmd, 1,
  goto_cmd, 105, 2380, 300, -1,
  set_position_cmd, 105, 2328, -90,
  goto_cmd, 105, 2260, 300, 1,
  walking_position_cmd, 0,
  goto_cmd, 205, 2260, 400, 1,
  pickup_cmd, 1, 1, 0,
  goto_cmd, 20, 2260, 200, -1,
  set_position_cmd, 25, 2260, 0,
  goto_cmd, 65, 2260, 400, 1,
  goto_cmd, 65, 1800, 400, -1,
  goto_cmd, 330, 1800, 500, 1,
  putdown_cmd, 1, 0,
  goto_cmd, 0, 1800, 250, -1,
  set_position_cmd, 25, 1800, 0,
  goto_cmd, 35, 1800, 400, 1,
  goto_cmd, 40, 1635, 500, 1,
  goto_cmd, 80, 1635, 500, 1,
  pickup_cmd, 1, 1, 0,
  goto_cmd, -10, 1635, 400, -1,
  set_position_cmd, 25, 1635, 0,
  goto_cmd, 50, 1635, 400, 1,
  goto_cmd, 100, 2280, 550, 1,
  goto_cmd, 310, 2300, 500, -1,
  orientate_cmd, 90, 0,
  goto_cmd, 310, 2420, 500, 1,
  putdown_cmd, 1, 1,
  set_position_cmd, 310, 2332, 90,
  goto_cmd, 310, 1800, 200, -1,
  goto_cmd, 310, 1900, 500, 1,
  walking_position_cmd, 0,
  goto_cmd, 310, 1780, 500, 1,
  pickup_cmd, 1, 1, 0,
  putdown_cmd, 1, 1,
  goto_cmd, 310, 1830, 500, -1,
  walking_position_cmd, 1,
  goto_cmd, 120, 1830, 400, 1,
  goto_cmd, 120, 1700, 400, 1,
  goto_cmd, 380, 1700, 200, 1,
  goto_cmd, -60, 1700, 500, -1,
  set_position_cmd, 10, 1700, 0,
  goto_cmd, 30, 1700, 500, 1,
  goto_cmd, 370, 1250, 500, 1,
  party_time_cmd
};


  const int *commands = is_yellow ? master_cina_yellow : master_cina_blue;
  int num_commands = is_yellow
      ? (int)(sizeof(master_cina_yellow) / sizeof(master_cina_yellow[0]))
      : (int)(sizeof(master_cina_blue) / sizeof(master_cina_blue[0]));
  extern size_t commands_count;
  //int num_commands = commands_count;
  //int num_commands = sizeof(commands) / sizeof(commands[0]); // For testing, replace with commands_count in production
  if (communicationTaskHandle != NULL) {
      vTaskDelete(communicationTaskHandle);
      communicationTaskHandle = NULL;
      Serial.println("Communication task deleted, strategy is now in control.");
  }
  int i = 0;
  strategy_start_time = millis();
  // rotate(1800, 0);
  // while(1);
  while (i < num_commands){
    Serial.print("current command");
    Serial.println(i);
    int command_type = commands[i];
    if (command_type == goto_cmd){
      int x = commands[i + 1];
      int y = commands[i + 2];
      int speed = commands[i + 3];
      int direction = commands[i + 4];
      Serial.printf("[CMD] goto_cmd: x=%d, y=%d, speed=%d, direction=%d\n", x, y, speed, direction);
      while(go_to(x, y, speed, direction)){
        wait_for_obstacle_clearance();
        Serial.println("obstacle cleared, resuming...");
        Serial.println("mahdi y7ebb louled");
      }
      Serial.printf("[POS] After goto: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 5;
    }
    else if (command_type == rotate_cmd) {
      int angle = commands[i + 1];
      int is_radian = commands[i + 2];
      Serial.printf("[CMD] rotate_cmd: angle=%d, is_radian=%d\n", angle, is_radian);
      rotate(angle, is_radian);
      Serial.printf("[POS] After rotate: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 3;
    }
    else if (command_type == orientate_cmd){
      int angle = commands[i + 1];
      int is_radian = commands[i + 2];
      Serial.printf("[CMD] orientate_cmd: angle=%d, is_radian=%d\n", angle, is_radian);
      orientate(angle, is_radian);
      Serial.printf("[POS] After orientate: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 3;
    }
    else if (command_type == set_position_cmd){
      int x = commands[i + 1];
      int y = commands[i + 2];
      int phi_degrees = commands[i + 3];
      Serial.printf("[CMD] set_position_cmd: x=%d, y=%d, phi=%d degrees\n", x, y, phi_degrees);
      float phi_radians = deg_to_rad(phi_degrees);
      set_position(x, y, phi_radians, phi_radians);
      Serial.printf("[POS] After set_position: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n",   current_x, current_y, rad_to_deg(current_phi));
      i += 4;
    }
    else if (command_type == pickup_cmd){
      Serial.printf("[CMD] pickup_cmd: arg1=%d, arg2=%d, arg3=%d\n", commands[i + 1], commands[i + 2], commands[i + 3]);
      pickup(commands[i + 1], commands[i + 2], commands[i + 3]);
      Serial.printf("[POS] After pickup: robot_x=%.1f, robot_y=%.1f, robot_phi=%.1f deg\n", current_x, current_y, rad_to_deg(current_phi));
      i += 4;
    }
    else if (command_type == putdown_cmd){
      int bechwaya = commands[i+1];
      int nid_lot = commands[i+2];
      Serial.println("[CMD] putdown_cmd");
      putdown(bechwaya, nid_lot);
      i += 3;
    }
    else if (command_type == walking_position_cmd){
      int position = commands[i + 1];
      Serial.println("[CMD] walking_position_cmd");
      walking_position(position);
      i += 2;
    }
    else if (command_type == prepare_cmd){
      Serial.println("[CMD] prepare_cmd");
      prepare_servo_up();
      i++;
    }
    else if (command_type == letgo_cmd){
      Serial.println("[CMD] letgo_cmd");
      letgo();
      i++;
    }
    else if(command_type == forward_cmd){
      int distance = commands[i + 1];
      int speed = commands[i + 2];
      int x = commands[i + 3];
      int y = commands[i + 4];
      int direction = commands[i + 5];
      Serial.printf("[CMD] forward_cmd: distance=%d, speed=%d, x=%d, y=%d, direction=%d\n", distance, speed, x, y, direction);
      // while(go_backward_to(x, y, speed)){
      //   wait_for_obstacle_clearance();
      //   Serial.println("obstacle cleared, resuming...");
      //   }
      move_forward(distance,speed, x, y,direction);
      i += 6;
    }
    else if (command_type == party_time_cmd){
      party_time();
    }
    else if (command_type == fix_x_cmd){
      double new_x = commands[i + 1];
      control_loop_state = DISACTIVATED;
      set_position(new_x, current_y, current_phi, absolute_phi);
      i += 2;
    }else if (command_type == bezier_cmd){
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
    
    // else if(command_type == move_linear_cmd){
    //   int distance = commands[i + 1];
    //   int speed = commands[i + 2];
    //   int x = commands[i + 3];
    //   int y = commands[i + 4];
    //   int direction = commands[i + 5];
    //   // while(go_backward_to(x, y, speed)){
    //   //   wait_for_obstacle_clearance();
    //   //   Serial.println("obstacle cleared, resuming...");
    //   //   }
    //   int result = move_linear(distance, speed, x, y, 1); // direction = 1 for forward
    //   i += 6;
    // }

    else if (command_type == delay_cmd) {
      int delay_time = commands[i + 1];
      Serial.printf("[CMD] delay_cmd: custom_delay(%d)\n", delay_time);
      custom_delay(delay_time);
      i += 2;
    }
    else if (command_type == print_analog12_cmd) {
      int analogValue = analogRead(12);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(1000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(1000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(10000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(10000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(10000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
       custom_delay(10000);
      Serial.printf("[CMD] print_analog12_cmd: Analog value on pin 12 = %d\n", analogValue);
      i += 1;
      // Add a delay to ensure the message is printed before executing the next command
    }
    else{
      Serial.print("Unknown command type: ");
      Serial.println(command_type);
      break; // Exit the loop on unknown command
    }
  }
  // 3. Send completion signal
  //send_action_done(1);

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
 //PWM_L= 500;// MIN =320temchi  319 temchich   min=-320 temchi -319temchich  
 //PWM_R= 500;//MIN =320temchi  319 temchich    min=-320 temchi -319temchich  
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
      //custom_delay(85000);
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
        processReceivedString(receivedMsg);
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
  set_motor_pins( 19, 18, 21, 22, 17, 23);  // shah 19, 18, 21, 22, 17, 23

  
  set_system_pins(32, 26, 33, 27);// bdlthom //25

  Serial.println("Setting encoder channels...");
  set_encoder_channels(2, 15, 4, 16, 1400); // badalthom  2, 15, 4, 16, 

  Serial.println("Setting PWM parameters...");
  set_ultrasonic_pins(14, 34); // badalthom //14

  set_PWM_min_max(1023, 100, 120, -100, -120); //yess    //130  120

  Serial.println("Setting wheel parameters...");
  set_wheel_diameter_and_spacing(47.746482927568600730665129011754, 47.669870635475871651225659202322, 120.4418940144483039743055580586);
  HAL_setup();
  
  // Initialize agent LED
  pinMode(AGENT_LED_PIN, OUTPUT);
  digitalWrite(AGENT_LED_PIN, LOW);
  
  // Initialize WebSocket if enabled
  extern int websocket_enabled;
  websocket_enabled = websocket;  // Set from the main websocket variable
  if (websocket_enabled == 1) {
    Serial.println("Initializing WebSocket...");
    websocket_init();
    delay(1000);
  }


  Serial.println("Creating tasks...");
  
  // Create communication task
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
      8192, // Back to smaller stack
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

  xTaskCreatePinnedToCore(
      emergencyBreakTaskWithoutUltrason,
      "emergencyBreakTask",
      2048, // Stack size
      NULL,
      2, // High priority
      &emergencyBreakTaskHandle,
      1); // Pin to core 1

  // Create WebSocket task if enabled
 /* if (websocket_enabled == 1) {
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
*/
  Serial.println("All tasks created successfully!");
  control_loop_state = DISACTIVATED;
  delay(100);
  strategy_waiting = true; // Start the strategy after setup;

}
void loop(){
  
  //  for(int i=0; i<100; i++){
  //    set_target_speeds(i*2, i*2);

  //    send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
  //    delay(10);
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

  // send_values_to_usart(current_speed_L, current_speed_R, fabs(PWM_L)/(MAX(fabs(current_speed_L), 10))*100, target_speed_R);
  // delay(10);
  // send_values_to_usart(current_speed_R, current_speed_L, target_speed_R, target_speed_L);
  // delay(10);
  // send_values_to_usart(debugger*100, 0, target_speed_R, target_speed_L); // send encoder counts and position for debugging
  // send_values_to_usart(encoder_count_R, encoder_count_L, 0, 0); // send encoder counts and position for debugging
  // delay(10);
  // send_values_to_usart(current_x, current_y, current_phi * 100, 0);
  // delay(10);
}

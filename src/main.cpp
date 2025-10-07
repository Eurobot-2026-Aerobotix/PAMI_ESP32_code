#include <Arduino.h>
#include "HAL.h"
#include "odometry.h"
#include "navigation.h"
#include "command_controller.h"

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

/// tasks
TaskHandle_t communicationTaskHandle;
TaskHandle_t PIDTaskHandle;
TaskHandle_t commandExecutionTaskHandle;

// micro-ROS variables
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
bool microros_initialized = false;

#define RCCHECK(fn)                      \
  {                                      \
    rcl_ret_t temp_rc = fn;              \
    if ((temp_rc != RCL_RET_OK))         \
    {                                    \
      Serial.println("micro-ROS error"); \
      microros_initialized = false;      \
    }                                    \
  }
#define RCSOFTCHECK(fn)                       \
  {                                           \
    rcl_ret_t temp_rc = fn;                   \
    if ((temp_rc != RCL_RET_OK))              \
    {                                         \
      Serial.println("micro-ROS soft error"); \
    }                                         \
  }

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

void communicationTask(void *pvParameters)
{
  const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 second delay
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS in communication task...");

  // Set up WiFi transport - replace with your WiFi credentials and PC IP
  set_microros_wifi_transports("eurobot", "eurobot2026", "10.42.0.1", 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Create support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "pami_esp32_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "pami_data"));

  msg.data = 0;
  microros_initialized = true;
  Serial.println("micro-ROS initialized successfully!");

  while (1)
  {
    if (microros_initialized)
    {
      // Publish message
      Serial.print("Publishing message: ");
      Serial.println(msg.data);

      rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
      if (ret != RCL_RET_OK)
      {
        Serial.print("Publish failed with error: ");
        Serial.println(ret);
        microros_initialized = false;
      }
      else
      {
        Serial.println("Message published successfully!");
      }

      msg.data++;
    }
    else
    {
      Serial.println("micro-ROS not initialized, retrying...");
      delay(1000);
    }

    vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}

void PIDTask(void *pvParameters)
{
  const TickType_t xDelay = 2;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  update_position();
  float tmp = (micros() - previous_speed_control_time) / 1000000.0; // in seconds
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

void commandExecutionTask(void *pvParameters)
{
  execute_command();
}

void setup()
{
  // Initialize Serial for debugging
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("=== PAMI ESP32 micro-ROS Setup ===");

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
      1);

  xTaskCreatePinnedToCore(
      PIDTask,
      "PIDTask",
      2048,
      NULL,
      3,
      &PIDTaskHandle,
      1);

  xTaskCreatePinnedToCore(
      commandExecutionTask,
      "commandExecutionTask",
      2048,
      NULL,
      1,
      &commandExecutionTaskHandle,
      1);
}

void loop()
{
}

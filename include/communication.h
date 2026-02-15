#pragma once

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

// Macro definitions
#define LED_PIN 13
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            return false;            \
        }                            \
    }

#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

// Connection states
enum CommunicationState
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
};

// Global variables (extern declarations)
extern rclc_support_t support;
extern rcl_node_t node;
extern rclc_executor_t executor;
extern rcl_allocator_t allocator;
extern rcl_publisher_t action_done_publisher;
extern rcl_subscription_t subscriber;
extern rcl_subscription_t crates_color_subscriber;
extern rcl_subscription_t strategy_subscriber;
extern std_msgs__msg__Int32 start_msg;
extern std_msgs__msg__Int32 crates_color_msg;
extern std_msgs__msg__Int32 action_done_msg;
extern bool micro_ros_init_successful;
extern enum CommunicationState comm_state;
extern bool strategy_waiting;
extern bool crate_colors[8];

// Strategy data arrays
extern float received_floats[200];
extern size_t received_floats_count;
extern int commands[200];
extern size_t commands_count;
// Core functions
bool setup_micro_ros(const char *wifi_ssid, const char *wifi_password, const char *agent_ip, int agent_port);
void communication_loop();
void cleanup_micro_ros();
bool connect_wifi(const char *ssid, const char *password);
// Entity management
bool create_entities();
void destroy_entities();

// Callback functions
void start_callback(const void *msgin);
void crates_color_callback(const void *msgin);
void strategy_callback(const void *msgin);

// Utility functions
void update_crate_colors(int32_t decimal_value);
void send_action_done(int value);
bool is_agent_connected();
void check_wifi_connection(const char *ssid, const char *password);
// Strategy function (to be implemented by user)
void strategy();
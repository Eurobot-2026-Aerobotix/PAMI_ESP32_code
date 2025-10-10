#include "communication.h"

// Global variable definitions
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t action_done_publisher;
rcl_subscription_t subscriber;
rcl_subscription_t crates_color_subscriber;
std_msgs__msg__Int32 start_msg;
std_msgs__msg__Int32 crates_color_msg;
std_msgs__msg__Int32 action_done_msg;
bool micro_ros_init_successful = false;
enum CommunicationState comm_state = WAITING_AGENT;
bool crate_colors[8] = {false}; // Initialize all as blue (0 = blue, 1 = yellow)

// Setup micro-ROS with WiFi connection and agent configuration
bool setup_micro_ros(const char *wifi_ssid, const char *wifi_password, const char *agent_ip, int agent_port)
{
    Serial.println("=== Setting up micro-ROS ===");

    // Setup WiFi transport (cast to non-const for the library)
    Serial.print("Connecting to WiFi: ");
    Serial.println(wifi_ssid);
    set_microros_wifi_transports((char *)wifi_ssid, (char *)wifi_password, (char *)agent_ip, agent_port);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize message data
    start_msg.data = 0;
    crates_color_msg.data = 0;
    action_done_msg.data = 0;

    // Set initial state
    comm_state = WAITING_AGENT;
    micro_ros_init_successful = true;

    Serial.println("micro-ROS setup completed");
    return true;
}

// Main communication loop - should be called regularly from main loop
void communication_loop()
{
    switch (comm_state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, comm_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT; if (comm_state == AGENT_AVAILABLE) { Serial.println("Agent detected!"); });
        break;

    case AGENT_AVAILABLE:
        Serial.println("Creating entities...");
        comm_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (comm_state == WAITING_AGENT)
        {
            Serial.println("Failed to create entities, retrying...");
            destroy_entities();
        }
        else
        {
            Serial.println("Successfully connected to agent!");
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200,
                           comm_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (comm_state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        else
        {
            Serial.println("Agent disconnected!");
        }
        break;

    case AGENT_DISCONNECTED:
        Serial.println("Cleaning up and reconnecting...");
        destroy_entities();
        comm_state = WAITING_AGENT;
        break;

    default:
        break;
    }

    // Update LED status
    digitalWrite(LED_PIN, (comm_state == AGENT_CONNECTED) ? HIGH : LOW);
}

// Create all ROS entities (publishers, subscribers, timers)
bool create_entities()
{
    allocator = rcl_get_default_allocator();

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "pami_esp32_node", "", &support));

    // Create action_done publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &action_done_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "action_done"));

    // Create start subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "start"));

    // Create crates_color subscriber
    RCCHECK(rclc_subscription_init_best_effort(
        &crates_color_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "crates_color"));

    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &start_msg, &start_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &crates_color_subscriber, &crates_color_msg, &crates_color_callback, ON_NEW_DATA));

    return true;
}

// Clean up all ROS entities
void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&action_done_publisher, &node);
    rcl_subscription_fini(&subscriber, &node);
    rcl_subscription_fini(&crates_color_subscriber, &node);
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// Callback for start topic subscription
void start_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    Serial.print("Received start signal: ");
    Serial.println(msg->data);

    if (msg->data == 1)
    {
        Serial.println("Start signal detected (value = 1)!");
        strategy(); // Call the strategy function
    }
}

// Callback for crates_color topic subscription
void crates_color_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    Serial.print("Received crates color data: ");
    Serial.println(msg->data);

    // Update the crate colors based on the received decimal number
    update_crate_colors(msg->data);
}

// Convert decimal to binary and update crate colors array
void update_crate_colors(int32_t decimal_value)
{
    Serial.print("Converting decimal ");
    Serial.print(decimal_value);
    Serial.print(" to binary: ");

    // Convert decimal to binary and store in array
    for (int i = 0; i < 8; i++)
    {
        crate_colors[i] = (decimal_value >> i) & 1;
        Serial.print(crate_colors[i]);
    }
    Serial.println();

    // Print crate colors in human readable format
    Serial.println("Crate colors (from crate 1 to 8):");
    for (int i = 0; i < 8; i++)
    {
        Serial.print("Crate ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(crate_colors[i] ? "YELLOW" : "BLUE");
    }
    Serial.println("---");
}

// Send action done message
void send_action_done(int value)
{
    if (comm_state == AGENT_CONNECTED)
    {
        action_done_msg.data = value;
        rcl_ret_t ret = rcl_publish(&action_done_publisher, &action_done_msg, NULL);
        if (ret != RCL_RET_OK)
        {
            Serial.print("Failed to publish action_done: ");
            Serial.println(ret);
        }
        else
        {
            Serial.print("Published action_done: ");
            Serial.println(value);
        }
    }
    else
    {
        Serial.println("Cannot send action_done: Agent not connected");
    }
}

// Check if agent is connected
bool is_agent_connected()
{
    return (comm_state == AGENT_CONNECTED);
}

// Cleanup function
void cleanup_micro_ros()
{
    if (comm_state != WAITING_AGENT)
    {
        destroy_entities();
        comm_state = WAITING_AGENT;
    }
    micro_ros_init_successful = false;
    Serial.println("micro-ROS cleaned up");
}

// Default strategy implementation (weak symbol - can be overridden)
__attribute__((weak)) void strategy()
{
    Serial.println("Default strategy function called! Implement your own strategy() function.");

    // Example action: send action done message
    send_action_done(1);
}
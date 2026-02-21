#include "communication.h"
#include <std_msgs/msg/float32_multi_array.h>

// Micro-ROS name constant

const char* micro_ros_name = "pami2";
// Dynamic topic names (constructed from micro_ros_name)
char topic_action_done[50];
char topic_start[50];
char topic_strategy[50];

// Global variable definitions
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t action_done_publisher;
rcl_subscription_t subscriber;
rcl_subscription_t strategy_subscriber;
std_msgs__msg__Int32 start_msg;
std_msgs__msg__Int32 action_done_msg;
std_msgs__msg__Float32MultiArray strategy_msg;
float received_floats[100]; // Adjust size as needed
size_t received_floats_count = 0;
int commands[100]; // Integer array for converted commands
size_t commands_count = 0;
bool micro_ros_init_successful = false;
enum CommunicationState comm_state = WAITING_AGENT;

// WiFi reconnection variables
unsigned long last_wifi_check = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000; // Check every 5 seconds
const int MAX_WIFI_RETRY = 10;

// Store WiFi credentials for reconnection
static String stored_wifi_ssid = "";
static String stored_wifi_password = "";

// WiFi connection function with retry logic
bool connect_wifi(const char *ssid, const char *password) {
    // Store credentials for reconnection
    stored_wifi_ssid = String(ssid);
    stored_wifi_password = String(password);
    
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Configure WiFi to be more aggressive with reconnection
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    
    WiFi.begin(ssid, password);
    
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < MAX_WIFI_RETRY) {
        delay(500);
        Serial.print(".");
        retry_count++;
        
        // If connection attempt fails, try again
        if (retry_count % 5 == 0) {
            Serial.println("\nRetrying connection...");
            WiFi.disconnect();
            delay(100);
            WiFi.begin(ssid, password);
        }
    }
    
    // Give WiFi extra time to finish connecting after association errors
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWaiting for WiFi to stabilize...");
        delay(2000);
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

// Check and maintain WiFi connection
void check_wifi_connection(const char *ssid, const char *password) {
    if (millis() - last_wifi_check > WIFI_CHECK_INTERVAL) {
        last_wifi_check = millis();
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Reconnecting...");
            connect_wifi(ssid, password);
        }
    }
}

// Setup micro-ROS with WiFi connection and agent configuration
bool setup_micro_ros(const char *wifi_ssid, const char *wifi_password, const char *agent_ip, int agent_port)
{
    Serial.println("=== Setting up micro-ROS ===");

    // Connect to WiFi with retry logic
    if (!connect_wifi(wifi_ssid, wifi_password)) {
        Serial.println("Failed to connect to WiFi - retrying in background");
        // Don't return false, allow micro-ROS setup to continue
        // WiFi will be retried in the communication loop
    }

    // Setup micro-ROS transport
    Serial.println("Setting up micro-ROS transport...");
    set_microros_wifi_transports((char *)wifi_ssid, (char *)wifi_password, (char *)agent_ip, agent_port);

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize message data
    start_msg.data = 0;
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
        EXECUTE_EVERY_N_MS(1000, comm_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 3)) ? AGENT_AVAILABLE : WAITING_AGENT; if (comm_state == AGENT_AVAILABLE) { Serial.println("Agent detected!"); });
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
        EXECUTE_EVERY_N_MS(1000,
                           comm_state = (RMW_RET_OK == rmw_uros_ping_agent(500, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
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
        // Add a small delay before reconnecting to let the agent stabilize
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
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

    // Construct topic names dynamically
    snprintf(topic_action_done, sizeof(topic_action_done), "%s/action_done", micro_ros_name);
    snprintf(topic_start, sizeof(topic_start), "%s/start", micro_ros_name);
    snprintf(topic_strategy, sizeof(topic_strategy), "%s/strategy", micro_ros_name);

    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, micro_ros_name, "", &support));

    // Create action_done publisher
    RCCHECK(rclc_publisher_init_default(
        &action_done_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        topic_action_done));

    // Create start subscriber
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        topic_start));

    // Create strategy subscriber
    RCCHECK(rclc_subscription_init_default(
        &strategy_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        topic_strategy));
        // Allocate memory for incoming array
        strategy_msg.data.data = (float *)malloc(60 * sizeof(float));
        strategy_msg.data.size = 0;
        strategy_msg.data.capacity = 60;
    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &start_msg, &start_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &strategy_subscriber, &strategy_msg, &strategy_callback, ON_NEW_DATA));

    return true;
}

// Clean up all ROS entities
void destroy_entities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&action_done_publisher, &node);
    rcl_subscription_fini(&subscriber, &node);
    rcl_subscription_fini(&strategy_subscriber, &node);
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
        strategy_waiting=true;
        Serial.println("Start signal detected (value = 1)!");
        strategy(); // Call the strategy function
    }
}

// Callback for strategy topic subscription
void strategy_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    
    Serial.println("Received float table:");
    received_floats_count = msg->data.size;
    commands_count = msg->data.size;
    
    for (size_t i = 0; i < msg->data.size && i < 100; i++)
    {
        received_floats[i] = msg->data.data[i];
        commands[i] = (int)msg->data.data[i]; // Convert float to int
        
        Serial.print("  [");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(received_floats[i], 4);
        Serial.print(" -> command: ");
        Serial.println(commands[i]);
    }
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
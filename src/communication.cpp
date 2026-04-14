#include "communication.h"
#include <std_msgs/msg/float32_multi_array.h>

// Micro-ROS name constant

const char* micro_ros_name = "pami6";
// Dynamic topic names (constructed from micro_ros_name)
char topic_action_done[50];
char topic_start[50];
char topic_strategy[50];
char service_status[50];

// Global variable definitions
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t action_done_publisher;
rcl_subscription_t subscriber;
rcl_subscription_t strategy_subscriber;
rcl_service_t status_service;
std_msgs__msg__Int32 start_msg;
std_msgs__msg__Int32 action_done_msg;
std_msgs__msg__Float32MultiArray strategy_msg;
std_srvs__srv__Trigger_Request status_req;
std_srvs__srv__Trigger_Response status_res;
float received_floats[100]; // Adjust size as needed
size_t received_floats_count = 0;
int recieved_commands[100]; // Integer array for converted recieved_commands
int deploy_pos[3];
int first_delay = 1;
size_t commands_count = 0;
bool micro_ros_init_successful = false;
enum CommunicationState comm_state = WAITING_AGENT;
volatile bool comm_stop_requested = false;
static bool entities_created = false;

static const size_t STRATEGY_MSG_CAPACITY = 300;
static const size_t MAX_RECEIVED_FLOATS = 100;
static const size_t MAX_STRATEGY_COMMANDS = 300;

// WiFi reconnection variables
unsigned long last_wifi_check = 0;
const unsigned long WIFI_CHECK_INTERVAL = 5000; // Check every 5 seconds
const int MAX_WIFI_RETRY = 10;

// Store WiFi credentials for reconnection
static String stored_wifi_ssid = "";
static String stored_wifi_password = "";
bool pami_status=false;
// WiFi connection function with retry logic
bool connect_wifi(const char *ssid, const char *password) {
    // Store credentials for reconnection
    stored_wifi_ssid = String(ssid);
    stored_wifi_password = String(password);
    
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);
    
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Disable WiFi sleep for reliable real-time communication
    WiFi.setSleep(false);
    
    // Configure WiFi to be more aggressive with reconnection
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    
    WiFi.begin(ssid, password);
    
    int retry_count = 0;
    while (WiFi.status() != WL_CONNECTED && retry_count < MAX_WIFI_RETRY) {
        delay(300); // Reduced from 500ms to 300ms
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
    
    // Reduced stabilization time from 2000ms to 500ms
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWaiting for WiFi to stabilize...");
        delay(500);
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

    // Add random startup delay to avoid all ESPs connecting simultaneously (0-3 seconds)
    unsigned long random_delay = random(0, 3000);
    Serial.print("Random startup delay: ");
    Serial.print(random_delay);
    Serial.println("ms");
    delay(random_delay);

    // Connect to WiFi with retry logic
    if (!connect_wifi(wifi_ssid, wifi_password)) {
        Serial.println("Failed to connect to WiFi - retrying in background");
        // Don't return false, allow micro-ROS setup to continue
        // WiFi will be retried in the communication loop
    }

    // Setup micro-ROS transport
    Serial.println("Setting up micro-ROS transport...");
    set_microros_wifi_transports((char *)wifi_ssid, (char *)wifi_password, (char *)agent_ip, agent_port);

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
    // Add slight random delay to desynchronize agent checks across multiple ESPs
    static unsigned long check_offset = random(0, 500);
    
    switch (comm_state)
    {
    case WAITING_AGENT:
        // Relaxed timeout to 300ms with 3 attempts for more reliable detection
        EXECUTE_EVERY_N_MS(1000 + check_offset, comm_state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_AVAILABLE : WAITING_AGENT; if (comm_state == AGENT_AVAILABLE) { Serial.println("Agent detected!"); });
        break;

    case AGENT_AVAILABLE:
        Serial.println("Creating entities...");
        Serial.print("Free heap before entity creation: ");
        Serial.print(ESP.getFreeHeap());
        Serial.println(" bytes");
        comm_state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (comm_state == WAITING_AGENT)
        {
            Serial.println("Failed to create entities, retrying...");
            Serial.print("Free heap after failure: ");
            Serial.print(ESP.getFreeHeap());
            Serial.println(" bytes");
            destroy_entities();
            
            // Give agent time to clean up session before retrying (500ms)
            Serial.println("Waiting for agent cleanup...");
            delay(500);
        }
        else
        {
            Serial.println("Successfully connected to agent!");
            Serial.print("Free heap after success: ");
            Serial.print(ESP.getFreeHeap());
            Serial.println(" bytes");
        }
        break;

    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(1000,
                           comm_state = (RMW_RET_OK == rmw_uros_ping_agent(300, 3)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (comm_state == AGENT_CONNECTED)
        {
            // Reduced timeout to allow faster callback processing and prevent blocking
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
            
        }
        else
        {
            Serial.println("Agent disconnected!");
        }
        break;

    case AGENT_DISCONNECTED:
        Serial.println("Cleaning up and reconnecting...");
        destroy_entities();
        // Reduced delay from 1000ms to 300ms for faster reconnection
        vTaskDelay(pdMS_TO_TICKS(300));
        comm_state = WAITING_AGENT;
        break;

    default:
        break;
    }

    // Update LED status
    set_agent_led_state(comm_state == AGENT_CONNECTED);
}

// Create all ROS entities (publishers, subscribers, timers)
bool create_entities()
{
    Serial.println("DEBUG: Starting entity creation...");
    allocator = rcl_get_default_allocator();

    // Construct topic names dynamically
    snprintf(topic_action_done, sizeof(topic_action_done), "%s/action_done", micro_ros_name);
    snprintf(topic_start, sizeof(topic_start), "%s/start", micro_ros_name);
    snprintf(topic_strategy, sizeof(topic_strategy), "%s/strategy", micro_ros_name);
    snprintf(service_status, sizeof(service_status), "%s/status", micro_ros_name);

    // Create init_options
    Serial.println("DEBUG: Initializing support...");
    rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to init support, code: ");
        Serial.println(rc);
        return false;
    }

    // Create node
    Serial.println("DEBUG: Creating node...");
    rc = rclc_node_init_default(&node, micro_ros_name, "", &support);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to create node, code: ");
        Serial.println(rc);
        return false;
    }

    // Create action_done publisher
    Serial.println("DEBUG: Creating action_done publisher...");
    rc = rclc_publisher_init_default(
        &action_done_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        topic_action_done);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to create publisher, code: ");
        Serial.println(rc);
        return false;
    }

    // Create start subscriber
    Serial.println("DEBUG: Creating start subscriber...");
    rc = rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        topic_start);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to create start subscriber, code: ");
        Serial.println(rc);
        return false;
    }

    // Create strategy subscriber
    Serial.println("DEBUG: Creating strategy subscriber...");
    rc = rclc_subscription_init_default(
        &strategy_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        topic_strategy);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to create strategy subscriber, code: ");
        Serial.println(rc);
        return false;
    }
    
    // Allocate memory for incoming array
    Serial.println("DEBUG: Allocating memory for strategy message...");
    strategy_msg.data.data = (float *)malloc(STRATEGY_MSG_CAPACITY * sizeof(float));
    if (strategy_msg.data.data == NULL) {
        Serial.println("ERROR: Failed to allocate memory for strategy message!");
        return false;
    }
    strategy_msg.data.size = 0;
    strategy_msg.data.capacity = STRATEGY_MSG_CAPACITY;
    
    // Create status service
    Serial.println("DEBUG: Creating status service...");
    rc = rclc_service_init_default(
        &status_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        service_status);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to create status service, code: ");
        Serial.println(rc);
        return false;
    }
    
    // Create executor (3 handles: 2 subscriptions + 1 service)
    Serial.println("DEBUG: Creating executor...");
    executor = rclc_executor_get_zero_initialized_executor();
    rc = rclc_executor_init(&executor, &support.context, 3, &allocator);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to init executor, code: ");
        Serial.println(rc);
        return false;
    }
    
    Serial.println("DEBUG: Adding start subscription to executor...");
    rc = rclc_executor_add_subscription(&executor, &subscriber, &start_msg, &start_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to add start subscription, code: ");
        Serial.println(rc);
        return false;
    }
    
    Serial.println("DEBUG: Adding strategy subscription to executor...");
    rc = rclc_executor_add_subscription(&executor, &strategy_subscriber, &strategy_msg, &strategy_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to add strategy subscription, code: ");
        Serial.println(rc);
        return false;
    }
    
    Serial.println("DEBUG: Adding status service to executor...");
    rc = rclc_executor_add_service(&executor, &status_service, &status_req, &status_res, &status_service_callback);
    if (rc != RCL_RET_OK) {
        Serial.print("ERROR: Failed to add status service, code: ");
        Serial.println(rc);
        return false;
    }

    Serial.println("DEBUG: All entities created successfully!");
    entities_created = true;
    return true;
}

// Clean up all ROS entities
void destroy_entities()
{
    Serial.println("DEBUG: Cleaning up entities...");

    if (!entities_created && support.context.impl == NULL) {
        Serial.println("DEBUG: No active entities/context, skipping cleanup");
        return;
    }
    
    // Set destroy timeout to 0 for immediate cleanup when context is valid.
    if (support.context.impl != NULL) {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        if (rmw_context != NULL) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    // Destroy entities in reverse order of creation
    rcl_ret_t rc;
    
    if (entities_created) {
        rc = rcl_publisher_fini(&action_done_publisher, &node);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy publisher, code: ");
            Serial.println(rc);
        }
    
        rc = rcl_subscription_fini(&subscriber, &node);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy start subscriber, code: ");
            Serial.println(rc);
        }
    
        rc = rcl_subscription_fini(&strategy_subscriber, &node);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy strategy subscriber, code: ");
            Serial.println(rc);
        }
    
        rc = rcl_service_fini(&status_service, &node);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy status service, code: ");
            Serial.println(rc);
        }

        rc = rclc_executor_fini(&executor);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy executor, code: ");
            Serial.println(rc);
        }

        rc = rcl_node_fini(&node);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy node, code: ");
            Serial.println(rc);
        }

        rc = rclc_support_fini(&support);
        if (rc != RCL_RET_OK) {
            Serial.print("WARN: Failed to destroy support, code: ");
            Serial.println(rc);
        }
    }
    
    // Free allocated memory for strategy message
    if (strategy_msg.data.data != NULL) {
        free(strategy_msg.data.data);
        strategy_msg.data.data = NULL;
    }
    
    entities_created = false;
    support.context.impl = NULL;
    
    Serial.println("DEBUG: Entity cleanup complete");
}

// LED blink function - non-blocking version
void LED_BLINK()
{
    set_agent_led_off();
}

// Callback for start topic subscription
void start_callback(const void *msgin)
{
    const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
    Serial.print("Received start signal: ");
    Serial.println(msg->data);

    // Blink LED to indicate message received
    LED_BLINK();
    
    if (msg->data == 1)
    {
        pami_status = false;  // Set status to false when start signal received
        strategy_waiting=true;
        Serial.println("Start signal detected (value = 1)!");
        Serial.println("pami_status set to FALSE");
        //strategy(); // Call the strategy function
    }

}

// Callback for strategy topic subscription
void strategy_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    const size_t msg_size = msg->data.size;
    const size_t safe_copy_count = (msg_size < MAX_RECEIVED_FLOATS) ? msg_size : MAX_RECEIVED_FLOATS;
    const size_t safe_strategy_count = (msg_size < MAX_STRATEGY_COMMANDS) ? msg_size : MAX_STRATEGY_COMMANDS;
    
    Serial.println("Received float table:");
    received_floats_count = safe_copy_count;
    commands_count = safe_strategy_count;

    if (msg_size > STRATEGY_MSG_CAPACITY) {
        Serial.print("WARN: Incoming strategy too large for message buffer. Size: ");
        Serial.print(msg_size);
        Serial.print(", capacity: ");
        Serial.println(STRATEGY_MSG_CAPACITY);
    }

    if (msg_size > MAX_STRATEGY_COMMANDS) {
        Serial.print("WARN: Strategy truncated to ");
        Serial.print(MAX_STRATEGY_COMMANDS);
        Serial.println(" elements");
    }
    
    // Blink LED to indicate message received
    LED_BLINK();
    
    for (size_t i = 0; i < safe_copy_count; i++)
    {
        received_floats[i] = msg->data.data[i];
        recieved_commands[i] = (int)msg->data.data[i]; // Convert float to int
        
        Serial.print("  [");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(received_floats[i], 4);
        Serial.print(" -> command: ");
        Serial.println(recieved_commands[i]);
    }

    if (safe_copy_count > 2) {
        is_yellow = (recieved_commands[2] < 1500);
    }

    for (size_t i = 0; i < safe_strategy_count; i++){
        strategy_commands[i] = (int)msg->data.data[i];
    }
    strategy_length = (int)safe_strategy_count;
    
    save_strategy(strategy_commands, strategy_length, is_yellow);
    
    pami_status = true;  // Set status to true when strategy received
    Serial.println("Strategy received - pami_status set to TRUE");
}

// Service callback for status service
void status_service_callback(const void *req, void *res)
{
    std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
    
    // Set response success to pami_status
    response->success = pami_status;
    
    // Set message based on status
    const char* msg = pami_status ? "PAMI is ready" : "PAMI is busy";
    
    // Allocate memory for the string
    response->message.data = (char*)malloc(strlen(msg) + 1);
    if (response->message.data != NULL) {
        strcpy(response->message.data, msg);
        response->message.size = strlen(msg);
        response->message.capacity = strlen(msg) + 1;
    } else {
        // Fallback if allocation fails
        response->message.data = (char*)"";
        response->message.size = 0;
        response->message.capacity = 0;
    }
    
    Serial.print("Status service called - returning: ");
    Serial.println(pami_status ? "true" : "false");
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
    comm_stop_requested = false;
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
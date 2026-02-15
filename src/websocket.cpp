#include "websocket.h"
#include "navigation.h"
#include <command_controller.h>

using namespace websockets;

// === WebSocket Control Variable ===
int websocket_enabled = 0;  // Set to 1 to enable WebSocket functionality

// === WiFi Configuration ===
const char* WIFI_SSID = "IEEE";      // Your WiFi network name
const char* WIFI_PASSWORD = "";  // Your WiFi password

// === Server Configuration ===
const char* SERVER_IP = "172.18.173.51";  // Your PC's IP address - Change this!
const int SERVER_PORT = 8765;

// === WebSocket Client ===
WebsocketsClient client;

// === Internal Variables ===
static String input = "";
static bool wifi_connected = false;
static bool websocket_connected = false;
static unsigned long last_wifi_check = 0;
static unsigned long last_websocket_check = 0;
static bool reconnecting = false;

// === String Reception Variables ===
static String received_string = "";
static bool new_string_received = false;

// === WiFi Connection ===
void websocket_init() {
    if (websocket_enabled != 1) {
        return;
    }
    
    Serial.println("\n🚀 ESP32 WebSocket Client Starting...");
    
    // Connect to WiFi
    Serial.printf("📡 Connecting to WiFi network: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected = true;
        Serial.println("\n✅ WiFi Connected!");
        Serial.print("📍 ESP32 IP Address: ");
        Serial.println(WiFi.localIP());
        
        // Setup WebSocket callbacks
        client.onMessage([](WebsocketsMessage message) {
            if (message.isText()) {
                // Handle text messages (Strings)
                received_string = message.data();
                new_string_received = true;
                
                // Process the received string
                processReceivedString(received_string);
                
            } else if (message.isBinary()) {
                // Handle binary messages
                String binaryData = message.data();
                for (int i = 0; i < binaryData.length(); i++) {
                    Serial.printf("0x%02X ", (uint8_t)binaryData[i]);
                }
                Serial.println();
            } else {
                // Fallback for other message types
            }
        });
        
        client.onEvent([](WebsocketsEvent event, String data) {
            if (event == WebsocketsEvent::ConnectionClosed) {
                Serial.println("❌ WebSocket Connection Closed");
                websocket_connected = false;
            } else if (event == WebsocketsEvent::ConnectionOpened) {
                Serial.println("✅ WebSocket Connected!");
                websocket_connected = true;
            }
        });
        
        // Attempt initial connection
        websocket_connect();
    } else {
        Serial.println("\n❌ WiFi Connection Failed!");
        wifi_connected = false;
    }
}

void websocket_connect() {
    if (websocket_enabled != 1 || !wifi_connected) {
        return;
    }
    
    // Close any existing connection first
    if (client.available()) {
        client.close();
        delay(100); // Small delay to ensure clean disconnect
    }
    
    String wsUrl = String("ws://") + SERVER_IP + ":" + SERVER_PORT;
    Serial.printf("🔌 Connecting to WebSocket: %s\n", wsUrl.c_str());
    
    bool connected = client.connect(wsUrl.c_str());
    
    // Give a moment for the connection to establish
    if (connected) {
        delay(100);
        // Double-check the connection is actually established
        connected = client.available();
    }
    
    websocket_connected = connected;
    
    if (connected) {
        Serial.println("✅ WebSocket Connection Established!");
    } else {
        Serial.println("❌ WebSocket Connection Failed!");
    }
}

void websocket_disconnect() {
    if (websocket_connected) {
        client.close();
        websocket_connected = false;
        Serial.println("🔌 WebSocket Disconnected");
    }
}

bool websocket_is_connected() {
    if (websocket_enabled != 1 || !wifi_connected) {
        return false;
    }
    
    // Check actual WebSocket connection status
    bool actually_connected = client.available();
    
    // Update our internal flag if it's wrong
    if (actually_connected != websocket_connected) {
        websocket_connected = actually_connected;
        if (actually_connected) {
            Serial.println("✅ WebSocket connection detected!");
        } else {
            Serial.println("❌ WebSocket connection lost!");
        }
    }
    
    return actually_connected;
}

void websocket_poll() {
    if (websocket_enabled != 1) {
        return;
    }
    
    if (wifi_connected && websocket_connected) {
        client.poll();
    }
}

void websocket_reconnect_if_needed() {
    if (websocket_enabled != 1) {
        return;
    }
    
    unsigned long current_time = millis();
    
    // Check WiFi connection every 5 seconds to avoid spam
    if (current_time - last_wifi_check > 5000) {
        last_wifi_check = current_time;
        
        if (WiFi.status() != WL_CONNECTED) {
            if (wifi_connected) {
                wifi_connected = false;
                websocket_connected = false;
                Serial.println("⚠️ WiFi disconnected, will attempt reconnection...");
                reconnecting = true;
            }
            
            if (reconnecting) {
                Serial.print("📡 Attempting WiFi reconnection...");
                WiFi.reconnect();
                
                // Give WiFi some time to connect (non-blocking check)
                int attempts = 0;
                while (WiFi.status() != WL_CONNECTED && attempts < 10) {
                    delay(500);
                    Serial.print(".");
                    attempts++;
                }
                
                if (WiFi.status() == WL_CONNECTED) {
                    wifi_connected = true;
                    reconnecting = false;
                    Serial.println("\n✅ WiFi reconnected!");
                    Serial.print("📍 ESP32 IP Address: ");
                    Serial.println(WiFi.localIP());
                    // Reset WebSocket check timer to attempt connection immediately
                    last_websocket_check = 0;
                } else {
                    Serial.println("\n❌ WiFi reconnection failed, will try again later...");
                }
            }
        } else if (!wifi_connected && !reconnecting) {
            wifi_connected = true;
            Serial.println("✅ WiFi connection restored!");
            // Reset WebSocket check timer to attempt connection immediately
            last_websocket_check = 0;
        }
    }
    
    // Check WebSocket connection every 3 seconds (only if WiFi is connected)
    if (wifi_connected && (current_time - last_websocket_check > 3000)) {
        last_websocket_check = current_time;
        
        // More robust WebSocket connection checking
        bool ws_actually_connected = client.available();
        
        if (!ws_actually_connected) {
            if (websocket_connected) {
                websocket_connected = false;
                Serial.println("⚠️ WebSocket connection lost, attempting reconnection...");
            } else {
                Serial.println("🔄 Attempting WebSocket connection...");
            }
            
            // Always attempt to reconnect when not connected
            websocket_connect();
            
            // Verify connection after attempt
            if (client.available()) {
                websocket_connected = true;
                Serial.println("✅ WebSocket reconnected successfully!");
            } else {
                websocket_connected = false;
                Serial.println("❌ WebSocket reconnection failed, will retry...");
            }
        } else if (!websocket_connected) {
            // Connection is available but our flag was wrong - update it
            websocket_connected = true;
            Serial.println("✅ WebSocket connection restored!");
        }
    }
}

// === Send data packet ===
void websocket_send_data(uint8_t header, float value) {
    if (!websocket_is_connected()) {
        return;
    }
    
    uint8_t packet[5];
    packet[0] = header;
    memcpy(&packet[1], &value, 4);  // Copy float as little-endian bytes
    
    client.sendBinary((const char*)packet, 5);
}


// === String Processing Function ===
void processReceivedString(const String& str) {
    // Process different commands or data from server
    if (str.startsWith("CMD:")) {
        // Handle commands from server
        String command = str.substring(4); // Remove "CMD:" prefix
        Serial.println("🎯 Command received: " + command);
        
        if (command == "START") {
            Serial.println("▶️ Start command received!");
            //prepare_move_forward(200, 300);
            prepare_rotate(180, 0);
        } else if (command == "STOP") {
            Serial.println("⏹️ Stop command received!");
            set_target_speeds(0, 0);
        } else if (command.startsWith("T")) {
            // Example: "SPEED:100" to set speed to 100
            command = command.substring(1);
            if (command.startsWith("L")) {
                int value = command.substring(1).toFloat();
                set_target_speeds(target_speed_R, value);
                Serial.println("Left motor target speed set to: " + String(value));
            }else if (command.startsWith("R")) {
                int value = command.substring(1).toFloat();
                set_target_speeds(value, target_speed_L);
                Serial.println("Right motor target speed set to: " + String(value));
            }
        }else if (command.startsWith("L")) {
            // Example: "SPEED:100" to set speed to 100
            String cmd = command.substring(1);
            if (cmd.startsWith("I")) {
                float value = cmd.substring(1).toFloat();
                kiS_L = value;
                Serial.println("Left motor integral gain set to: " + String(kiS_L));
            }else if (cmd.startsWith("P")) {
                float value = cmd.substring(1).toFloat();
                kpS_L = value;
                Serial.println("Left motor proportional gain set to: " + String(kpS_L));
            }
        } else if (command.startsWith("R")) {
            // Example: "SPEED:100" to set speed to 100
            String cmd = command.substring(1);
            if (cmd.startsWith("I")) {
                float value = cmd.substring(1).toFloat();
                kiS_R = value;
                Serial.println("Right motor integral gain set to: " + String(kiS_R));
            }else if (cmd.startsWith("P")) {
                float value = cmd.substring(1).toFloat();
                kpS_R = value;
                Serial.println("Right motor proportional gain set to: " + String(kpS_R));
            }
        }
        
    } else if (str.startsWith("CONFIG:")) {
        // Handle configuration data
        String config = str.substring(7);
        Serial.println("⚙️ Configuration: " + config);
        
    } else if (str.startsWith("DATA:")) {
        // Handle data from server
        String data = str.substring(5);
        Serial.println("📊 Data received: " + data);
        
    } else {
        // Handle plain text messages
        //Serial.println("💬 Message: " + str);
    }
}

// === Check for New String ===
bool websocket_has_new_string() {
    return new_string_received;
}

// === Get Last Received String ===
String websocket_get_last_string() {
    new_string_received = false; // Reset flag
    return received_string;
}

// === Data Generation Functions (Replace with your actual sensors) ===
float generateValue1() {
    return sin(millis() / 1000.0) * 500;  // Example: sine wave
}

float generateValue2() {
    return cos(millis() / 1000.0) * 500;  // Example: cosine wave
}

float generateValue3() {
    return random(-800, 800);  // Example: random value
}

float generateValue4() {
    return (millis() % 2000) - 1000;  // Example: sawtooth wave
}

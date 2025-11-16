#ifndef WEBSOCKET_H
#define WEBSOCKET_H

#include <WiFi.h>
#include <ArduinoWebsockets.h>

// WebSocket configuration
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* SERVER_IP;
extern const int SERVER_PORT;

// WebSocket control variable
extern int websocket_enabled;

// WebSocket client instance
extern websockets::WebsocketsClient client;

// Function declarations
void websocket_init();
void websocket_connect();
void websocket_disconnect();
bool websocket_is_connected();
void websocket_poll();
void websocket_send_data(uint8_t header, float value);
void websocket_send_serial_data();
void websocket_reconnect_if_needed();

// FreeRTOS task function
void websocketTask(void *pvParameters);

// String reception functions
void processReceivedString(const String& str);
bool websocket_has_new_string();
String websocket_get_last_string();

// Data generation functions (examples - replace with your actual sensors)
float generateValue1();
float generateValue2();
float generateValue3();
float generateValue4();

#endif // WEBSOCKET_H
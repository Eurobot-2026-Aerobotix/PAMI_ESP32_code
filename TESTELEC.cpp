#include <Arduino.h>
#include <WiFi.h>
#include "HAL.h"

// Test program: activate both electrovannes for 2 seconds, then deactivate
void setup() {
	Serial.begin(115200);

	// --- WiFi Setup ---
	const char* ssid = "IEEE"; // Change to your WiFi SSID
	const char* password = ""; // Change to your WiFi password
	Serial.print("Connecting to WiFi: ");
	Serial.println(ssid);
	WiFi.begin(ssid, password);
	int wifi_attempts = 0;
	while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
		delay(500);
		Serial.print(".");
		wifi_attempts++;
	}
	if (WiFi.status() == WL_CONNECTED) {
		Serial.println("\nWiFi connected!");
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
	} else {
		Serial.println("\nWiFi connection failed.");
	}

	Serial.println("Setting all HAL output pins HIGH...");
	set_system_pins(25, 26, 33, 27);// bdlthom
	HAL_setup();
	// Set all output pins HIGH
	digitalWrite(electrovalve_R_pin, LOW);
	digitalWrite(electrovalve_L_pin, LOW);
	digitalWrite(pump_pin, HIGH);

	Serial.println("All HAL output pins set HIGH.");
	// Hold state for observation
	delay(10000);

	digitalWrite(electrovalve_R_pin, HIGH);
	digitalWrite(electrovalve_L_pin, HIGH);
	digitalWrite(pump_pin, LOW);
	Serial.println("All HAL output pins set LOW.");
}

void loop() {
	// Do nothing, or could repeat the test if desired
}

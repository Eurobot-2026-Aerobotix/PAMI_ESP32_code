#include <Arduino.h>
#include <Arduino.h>
#include "navigation.h"
#include "command_controller.h"
#include "communication.h"
#include "websocket.h"


#define TRIG_PIN 14
#define ECHO_PIN 34

// unsigned long pickup_time = 0;
// bool did_pickup = false;
// bool did_putdown = false;

void setup() {
	Serial.begin(115200);
	pinMode(TRIG_PIN, OUTPUT);
	pinMode(ECHO_PIN, INPUT);
	set_system_pins(26, 33, 13, 27);
	HAL_setup();
	Serial.println("Ultrasonic sensor test (trig=14, echo=34)");
}


void loop() {

	//if (!did_pickup) {
		move_servo(0);
		Serial.println("Pickup called");
		//pickup_time = millis();
	//	did_pickup = true;
	//}
delay(3000); // Wait for pickup to complete
	// Send a 10us HIGH pulse on trig
	digitalWrite(TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	// Read echo pulse width
	long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms
	float distance_cm = duration * 0.0343 / 2.0;

	Serial.print("Distance: ");
	Serial.print(distance_cm);
	Serial.println(" cm");

	// After 5 seconds, call putdown once
	//if (did_pickup && !did_putdown && millis() - pickup_time > 5000) {
		putdown(1, 1);
	//	did_putdown = true;
		Serial.println("Putdown called");
	//}

	delay(2000); // Wait before next measurement
}
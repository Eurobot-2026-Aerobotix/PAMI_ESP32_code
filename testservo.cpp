
#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 27
Servo myServo;

String inputString = "";
bool stringComplete = false;

void setup() {
	Serial.begin(115200);
	myServo.attach(SERVO_PIN);
	Serial.println("Enter angle (0-180):");
	inputString.reserve(10);
}

void loop() {
	while (Serial.available()) {
		char inChar = (char)Serial.read();
		if (inChar == '\n' || inChar == '\r') {
			if (inputString.length() > 0) {
				stringComplete = true;
			}
		} else if (isDigit(inChar)) {
			inputString += inChar;
		}
	}

	if (stringComplete) {
		int angle = inputString.toInt();
		if (angle >= 0 && angle <= 180) {
			myServo.write(angle);
			Serial.print("Servo moved to: ");
			Serial.println(angle);
		} else {
			Serial.println("Invalid angle. Enter 0-180.");
		}
		inputString = "";
		stringComplete = false;
		Serial.println("Enter angle (0-180):");
	}
}

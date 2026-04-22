
#include <Arduino.h>

void setup() {
	Serial.begin(115200);
	pinMode(2, INPUT_PULLUP);


pinMode(15, INPUT_PULLUP);


	Serial.println("...............");
}

void loop() {
	// Nothing to do in loop
	Serial.println(".....pin15..........");
	Serial.println(digitalRead(15));
	Serial.println(".....pin2..........");
	Serial.println(digitalRead(2));


}

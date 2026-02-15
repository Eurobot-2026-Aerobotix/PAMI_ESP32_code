#include <Arduino.h>

#define PIN1 25
#define PIN2 26
#define PIN3 33

void setup() {
  Serial.begin(115200);

  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(PIN3, OUTPUT);

  Serial.println("Ready. Commands:");
  Serial.println("1 -> PIN25 ON");
  Serial.println("2 -> PIN25 OFF");
  Serial.println("3 -> PIN26 ON");
  Serial.println("4 -> PIN26 OFF");
  Serial.println("5 -> PIN33 ON");
  Serial.println("6 -> PIN33 OFF");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == '1') {
      digitalWrite(PIN1, HIGH);
      Serial.println("PIN25 ON");
    }
    else if (command == '2') {
      digitalWrite(PIN1, LOW);
      Serial.println("PIN25 OFF");
    }
    else if (command == '3') {
      digitalWrite(PIN2, HIGH);
      Serial.println("PIN26 ON");
    }
    else if (command == '4') {
      digitalWrite(PIN2, LOW);
      Serial.println("PIN26 OFF");
    }
    else if (command == '5') {
      digitalWrite(PIN3, HIGH);
      Serial.println("PIN33 ON");
    }
    else if (command == '6') {
      digitalWrite(PIN3, LOW);
      Serial.println("PIN33 OFF");
    }
  }
}

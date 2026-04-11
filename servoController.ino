#include <Servo.h>

Servo panServo;

int pan = 90;
String data = "";

void setup() {
  Serial.begin(9600);
  panServo.attach(9);  // ONLY ONE SERVO

  panServo.write(pan);
}

void loop() {
  if (Serial.available()) {
    data = Serial.readStringUntil('\n');

    int panVal = data.toInt();

    pan = constrain(panVal, 0, 180);
    panServo.write(pan);
  }
}

#include <Servo.h>
#define SERVO_PIN 12
Servo servo;
void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
  servo.write(0);
}

void loop() {
  servo.write(0);
  delay(1000);
  servo.write(63);
  delay(1000);
}
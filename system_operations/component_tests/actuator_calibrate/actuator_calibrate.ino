#include "helpers.hpp"

#define DRV11_PWM_PIN 6
#define DRV11_DIR1_PIN 34
#define DRV11_DIR2_PIN 36
#define DRV12_PWM_PIN 7
#define DRV12_DIR1_PIN 38
#define DRV12_DIR2_PIN 40

#define POTL_PIN A1
#define POTR_PIN A0

float act_max_vel = 25;

PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);

PWM_Driver left_driver(DRV12_PWM_PIN, DRV12_DIR1_PIN, DRV12_DIR2_PIN, false);
Actuator act_left(left_driver, pidL, POTL_PIN, 0, 1023, 191, act_max_vel);

PWM_Driver right_driver(DRV11_PWM_PIN, DRV11_DIR1_PIN, DRV11_DIR2_PIN, false);
Actuator act_right(right_driver, pidR, POTR_PIN, 50, 1023, 191, act_max_vel);

int done = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Actuator calibration started");

  act_left.stop();
  act_right.stop();
  delay(1000);
}

void loop() {
  if (done == 0) {
    act_left.vel_ctrl(10);
    act_right.vel_ctrl(10);
    delay(10000);
    act_left.stop();
    act_right.stop();

    int potL = analogRead(POTL_PIN);
    int potR = analogRead(POTR_PIN);
    Serial.print(potL);
    Serial.print(" ");
    Serial.println(potR);

    done = 1;
  }
}

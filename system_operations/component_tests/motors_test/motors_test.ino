#include "helpers.hpp"
const int DACL1_PIN = 2;
const int DACL2_PIN = 3;
const int DACR1_PIN = 4;
const int DACR2_PIN = 5;
const int EN_PIN = 32;  

int motor_max_vel = 30; 

void setup() {
  Serial.begin(115200);

  OutPin motor_left_dac1(DACL1_PIN);
  OutPin motor_left_dac2(DACL2_PIN);
  OutPin motor_right_dac1(DACR1_PIN);
  OutPin motor_right_dac2(DACR2_PIN);
  OutPin motor_enable(EN_PIN);
  Motor motor_left(motor_left_dac1, motor_left_dac2, motor_enable, motor_max_vel, false);
  Motor motor_right(motor_right_dac1, motor_right_dac2, motor_enable, motor_max_vel, true);
}

void loop() {
  motor_right.motor_ctrl(10);
}
            
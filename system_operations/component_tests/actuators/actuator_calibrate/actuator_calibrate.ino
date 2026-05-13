#include "helpers.hpp"






//LEFT
#define DRV12_PWM_PIN 8
#define DRV12_DIR1_PIN 38
#define DRV12_DIR2_PIN 40
#define POTL_PIN A0
//35, 865

//RIGHT
#define DRV11_PWM_PIN 6
#define DRV11_DIR1_PIN 34
#define DRV11_DIR2_PIN 36
#define POTR_PIN A2
// 35, 876

//BUCKET
#define DRV21_PWM_PIN 7
#define DRV21_DIR1_PIN 48
#define DRV21_DIR2_PIN 46
#define POTB_PIN A1
// 37 ,800





//IBusReader ibus(Serial1);

PWM_Driver left_driver(DRV11_PWM_PIN, DRV11_DIR1_PIN, DRV11_DIR2_PIN, false);
PWM_Driver right_driver(DRV12_PWM_PIN, DRV12_DIR1_PIN, DRV12_DIR2_PIN, false);
//PWM_Driver bucket_driver(DRV21_PWM_PIN, DRV21_DIR1_PIN, DRV21_DIR2_PIN, true);

unsigned long startTime;
bool running = true;

void setup() {
  Serial.begin(115200);
  //ibus.begin(115200);
  left_driver.stop();
  right_driver.stop();
  //bucket_driver.stop();
  startTime = millis();
}

void loop() {

    //int potL = analogRead(POTL_PIN);
    //int potR = analogRead(POTR_PIN);
    //int potB = analogRead(POTB_PIN);

    //Serial.print(potL);
    //Serial.print(" ");
    //Serial.print(potR);
    //Serial.print(" ");
    //Serial.println(potB);

  if (running) {
    //bucket_driver.set_speed(255);
    right_driver.set_speed(-100);
    left_driver.set_speed(-100);
    if (millis() - startTime >= 3000) {
      //bucket_driver.stop();
      left_driver.stop();
      right_driver.stop();
      running = false;
    }
  }

  delay(50);
}
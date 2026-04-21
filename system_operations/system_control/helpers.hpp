#pragma once
#ifndef HELPERS_H
#define HELPERS_H

#pragma once
#define MAIN_ROBOT 1

//Condition that changes actuator code to accept pwm, and GND, instead of current setup that uses pinout, pwm, and GND
bool MD04_drivers = false;

#include <Arduino.h>
#include "arduino_lib.hpp"

#define MEDIAN_SIZE 8 // Median filter window size for potentionmeter smoothing


//------------------------------------------------------------
//                       Other classes
//------------------------------------------------------------
class PID {
private:
  float error;
  float prev_error;
  float derivative;
  float integral;

  float p, i, d, s;

public:
  PID(float p, float i, float d, float s=0) : p(p), i(i), d(d), s(s) {
    error = 0;
    prev_error = 0;
    derivative = 0;
  }

  float update(float error, float rel_error=0) {
    derivative = error - prev_error;
    integral += error;
    prev_error = error;
    return p * error + i * integral + d * derivative + s * rel_error;
  }

  void resetIntegral() {
    integral = 0;
  }
};

class Median {
private:
  int history_size;
  int* history;
  int current_idx;

public:
  Median(int history_size) : history_size(history_size) {
    history = new int[history_size];
    current_idx = 0;
    memset(history, 0, sizeof(history));
  }

  ~Median() {
    delete[] history;
  }

  int update(int new_val) {
    history[current_idx] = new_val;
    current_idx++;
    current_idx %= history_size;

    int sorted[history_size];
    memcpy(sorted, history, history_size * sizeof(history[0]));

    qsort(sorted, history_size, sizeof(sorted[0]), [](const void *a, const void *b) {
      if (*(int *)a > *(int *)b)
        return 1;
      else if (*(int *)a < *(int *)b)
        return -1;
      return 0;
    });

    if (history_size % 2 == 1)
      return (sorted[history_size / 2] + sorted[history_size / 2 + 1]) / 2;
    else
      return sorted[history_size / 2];
  }
};

class PWM_Driver {
  OutPin pwm_pin;
  OutPin dir1_pin;
  OutPin dir2_pin;
  bool invert = false;

public:
  PWM_Driver(OutPin pwm_pin, OutPin dir1_pin, OutPin dir2_pin, bool invert=false) 
    : pwm_pin(pwm_pin), dir1_pin(dir1_pin), dir2_pin(dir2_pin), invert(invert) {}

  void set_speed(int speed) {
    if(MD04_drivers==false){
    speed = constrain(speed, -255, 255);
    if (invert) {
      speed = -speed;
    }
    if (speed > 0) {
      dir1_pin.write(1);
      dir2_pin.write(0);
    } else if (speed < 0) {
      dir1_pin.write(0);
      dir2_pin.write(1);
    } else {
      dir1_pin.write(0);
      dir2_pin.write(0);
    }
    pwm_pin.write_pwm_raw(abs(speed));
  }
  else{
    Serial.println(speed);
    if (speed > 128) {
      dir1_pin.write(1);
      dir2_pin.write(speed);
    } else if (speed < 128) {
      dir1_pin.write(0);
      dir2_pin.write(speed);
    } else {
      dir1_pin.write(0);
      dir2_pin.write(0);
    }
  }
  }

  void stop() {
    dir1_pin.write(0);
    dir2_pin.write(0);
    pwm_pin.write_pwm_raw(0);
  }
};

//------------------------------------------------------------
//                Actuator controllor class
//------------------------------------------------------------
class Actuator {
  // Class which talks to the basic actuator drivers
  PWM_Driver pwm_driver;
  float stroke;
  float pot_min;
  float pot_max;
  float act_max_vel;
  float pos_mm;
  float min_pos;
  float max_pos;
  float curved_speed = 0;
  float ramp_speed_time=0.5;
  unsigned long last_accel_time = 0;
  PID pid;

  SmoothedInput<MEDIAN_SIZE> pot;

  float f_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

public:
  Actuator(PWM_Driver driver, PID pid, InPin pot, float pot_min, float pot_max, float stroke, float act_max_vel, 
           float min_pos=0, float max_pos=0)
    : pwm_driver(driver), stroke(stroke), pot_min(pot_min), pot_max(pot_max), act_max_vel(act_max_vel), 
      pid(pid), pot(pot), min_pos(min_pos), max_pos(max_pos == 0 ? stroke : max_pos) {
      }
    
  float update_pos() {
    float analog_raw = pot.read_analog_raw();
    // Serial.println(analog_raw);
    pos_mm = f_map(analog_raw, pot_min, pot_max, 0, stroke);
    return pos_mm;
  }

  float get_pos() {
    return pos_mm;
  }

  void tgt_ctrl(int tgt) {
    float tgt_error = tgt - pos_mm;
    float speed = pid.update(tgt_error);
    vel_ctrl(speed);
  }

  void tgt_ctrl(int tgt, float other_pos) {
    float tgt_error = tgt - pos_mm;
    float rel_error = other_pos - pos_mm;
    float speed = pid.update(tgt_error, rel_error);
    vel_ctrl(speed);
  }

  void vel_ctrl(int speed) {
    if(MD04_drivers==false){
    speed = constrain(speed, -act_max_vel, act_max_vel);
    speed = speed / act_max_vel * 255;
    }
    pwm_driver.set_speed(speed);
  }

  void curved_vel_ctrl(int speed, float factor) {
    float dt = (millis() - last_accel_time) / 1000.0;
    last_accel_time = millis();
    float delta = speed - curved_speed;
    float max_delta = abs(speed)/ramp_speed_time*dt;
    if(curved_speed*speed >= 0){
        if(delta > max_delta) delta = max_delta;
        if(delta < -max_delta) delta = -max_delta;
    }
    curved_speed += delta;
    if(speed == 0) curved_speed = 0;
    int out = MD04_drivers ? curved_speed : constrain(curved_speed,-act_max_vel,act_max_vel)/act_max_vel*255;
    if(curved_speed != 0) out += factor;
    pwm_driver.set_speed(out);
  }

  void stop() {
    pwm_driver.stop();
  }

  void resetPIDIntegral() {
    pid.resetIntegral();
  }
};

//------------------------------------------------------------
//                 Motor controller classes
//------------------------------------------------------------
class Motor {
  // Class which talkers to the (MMP SA-715A) motor driver
  OutPin dac1, dac2, enable;
  int motor_max_vel;
  bool reverse;
  float curved_speed = 0;
  float ramp_speed_time = 0.75;
  unsigned long last_update = 0;
  unsigned long direction_change_block_until = 0;

public:
  Motor(OutPin d1, OutPin d2, OutPin en, int maxv, bool rev)
      : dac1(d1), dac2(d2), enable(en), motor_max_vel(maxv), reverse(rev) {
      last_update = millis();
  }

  void motor_ctrl(int target_speed) {
      unsigned long now = millis();
      float dt = (now - last_update) / 1000.0;
      last_update = now;

      target_speed = constrain(target_speed, -motor_max_vel, motor_max_vel);

      bool target_dir = (target_speed > 0);
      bool current_dir = (curved_speed > 0);

      if (curved_speed != 0 && target_dir != current_dir) {
          float max_delta = motor_max_vel / ramp_speed_time * dt;
          if (curved_speed > 0) curved_speed -= max_delta;
          else curved_speed += max_delta;

          if (abs(curved_speed) < 1) {
              curved_speed = 0;
              direction_change_block_until = now + 150; 
          }
      } else {
          if (now < direction_change_block_until) {
              curved_speed = 0;
          } else {
              float delta = target_speed - curved_speed;
              float max_delta = motor_max_vel / ramp_speed_time * dt;
              if (delta > max_delta) delta = max_delta;
              if (delta < -max_delta) delta = -max_delta;
              curved_speed += delta;
          }
      }
      if (curved_speed == 0) {
          stop();
          return;
      }

      int pwm = map(abs((int)curved_speed), 0, motor_max_vel, 0, 255);
      enable.write(1);
      bool forward = (curved_speed > 0);
      if (reverse) forward = !forward;
      if (forward) {
          dac1.write_pwm_raw(pwm);
          dac2.write_pwm_raw(0);
      } else {
          dac1.write_pwm_raw(0);
          dac2.write_pwm_raw(pwm);
      }
  }

  void stop() {
      enable.write(0);
      dac1.write_pwm_raw(0);
      dac2.write_pwm_raw(0);
  }
};

class SimpleMotor {
  // Class that talkers to the basic arduino motors
  int pwmPin;
  int dirPin1;
  int dirPin2;
  int maxSpeed; // e.g., 30 rpm

public:
  SimpleMotor(int pwm, int dir1, int dir2, int maxVel)
    : pwmPin(pwm), dirPin1(dir1), dirPin2(dir2), maxSpeed(maxVel) {}

  void begin() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    stop();
  }

  // speed: -maxSpeed to +maxSpeed
  void setSpeed(int speed) {
    // constrain speed to -maxSpeed..+maxSpeed
    speed = constrain(speed, -maxSpeed, maxSpeed);

    if (speed > 0) {
      digitalWrite(dirPin1, HIGH);
      digitalWrite(dirPin2, LOW);
      analogWrite(pwmPin, map(speed, 0, maxSpeed, 0, 255));
    } else if (speed < 0) {
      digitalWrite(dirPin1, LOW);
      digitalWrite(dirPin2, HIGH);
      analogWrite(pwmPin, map(-speed, 0, maxSpeed, 0, 255));
    } else {
      stop();
    }
  }

  void stop() {
    analogWrite(pwmPin, 0);
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
  }
};

class SimpleServo {
  // Class that talks to a simple servo motor
  private:
      uint8_t pin;
      int minPulse;   // microseconds for 0 degrees
      int maxPulse;   // microseconds for 180 degrees
      int currentAngle;
  
  public:
      SimpleServo(uint8_t pin, int minPulse = 1000, int maxPulse = 2000)
          : pin(pin), minPulse(minPulse), maxPulse(maxPulse), currentAngle(90) {}
  
      void attach() {
          pinMode(pin, OUTPUT);
          write(90);  // center on attach
      }
  
      void write(int angle) {
          angle = constrain(angle, 0, 180);
          currentAngle = angle;
  
          // map angle to pulse width
          int pulseWidth = map(angle, 0, 180, minPulse, maxPulse);
  
          // generate one PWM frame
          digitalWrite(pin, HIGH);
          delayMicroseconds(pulseWidth);
          digitalWrite(pin, LOW);
  
          // rest of 20ms frame
          delayMicroseconds(20000 - pulseWidth);
      }
  
      void writeContinuous(int angle) {
          // continuous mode: run PWM forever
          int pulseWidth = map(angle, 0, 180, minPulse, maxPulse);
  
          digitalWrite(pin, HIGH);
          delayMicroseconds(pulseWidth);
          digitalWrite(pin, LOW);
      }
  
      int read() const {
          return currentAngle;
      }
  };

//------------------------------------------------------------
//                     IMU talker class
//------------------------------------------------------------
class MPU6050 {
  // Class which reads daya from cheap (MPU6050) IMU sensor
  public:
      MPU6050(uint8_t address = 0x68) : _address(address) {}
      void initialize() {
          Wire.begin();
          // Wake up MPU6050
          writeRegister(0x6B, 0x00); // PWR_MGMT_1 = 0 (wake up)
      }
      bool testConnection() {
          uint8_t whoAmI = readRegister(0x75);
          return (whoAmI == 0x68);
      }
      // Read raw gyro values (deg/s)
      void getRotation(int16_t *gx, int16_t *gy, int16_t *gz) {
          *gx = readRegister16(0x43);
          *gy = readRegister16(0x45);
          *gz = readRegister16(0x47);
      }
      // Read raw accelerometer values
      void getAcceleration(int16_t *ax, int16_t *ay, int16_t *az) {
          *ax = readRegister16(0x3B);
          *ay = readRegister16(0x3D);
          *az = readRegister16(0x3F);
      }
  private:
      uint8_t _address;
      void writeRegister(uint8_t reg, uint8_t value) {
          Wire.beginTransmission(_address);
          Wire.write(reg);
          Wire.write(value);
          Wire.endTransmission(true);
      }
      uint8_t readRegister(uint8_t reg) {
          Wire.beginTransmission(_address);
          Wire.write(reg);
          Wire.endTransmission(false);
          Wire.requestFrom(_address, (uint8_t)1);
          return Wire.read();
      }
      int16_t readRegister16(uint8_t reg) {
          Wire.beginTransmission(_address);
          Wire.write(reg);
          Wire.endTransmission(false);
          Wire.requestFrom(_address, (uint8_t)2);
          int16_t val = (Wire.read() << 8) | Wire.read();
          return val;
      }
};

//------------------------------------------------------------
//                  Controller talker class
//------------------------------------------------------------
class IBusReader {
  // Class which handles and remaps the (FSIA6B IBUS) controller commands info usable format for controlling robot
  public:
      IBusReader(HardwareSerial &serial) : serial(serial) {}
      void begin(unsigned long baud = 115200) { 
        serial.begin(baud); 
        lastUpdate = millis(); 
        // This lets the controller do the mapping of it's own ranges, adjust as you move the joystick
        if (dynamicRanges==true) {
          maxJoyValues[0] = 1800; maxJoyValues[1] = 1800; maxJoyValues[2] = 1800; maxJoyValues[3] = 1800;
          minJoyValues[0] = 1200; minJoyValues[1] = 1200; minJoyValues[2] = 1200; minJoyValues[3] = 1200;
        }
        for(int i=0;i<4;i++) lastValues[i]=0; // init lastValues
      }
      // Call this each loop — non-blocking
      bool update() {
        bool changed = false;
        while (serial.available()) {
            uint8_t val = serial.read();
            // Sync header
            if (idx == 0 && val != 0x20) return false;
            if (idx == 1 && val != 0x40) { idx = 0; return false; }
            buffer[idx++] = val;
            // Process full packet
            if (idx == 32) {
                idx = 0;
                uint16_t chksum = 0xFFFF;
                for (int i = 0; i < 30; i++) chksum -= buffer[i];
                uint16_t pktChksum = buffer[30] | (buffer[31] << 8);
                if (chksum == pktChksum) {
                    for (int i = 0; i < 8; i++) {
                        int pos = 2 + (i * 2);
                        channels[i] = buffer[pos] | (buffer[pos + 1] << 8);
                        if (dynamicRanges) {
                            if (channels[i] < minJoyValues[i]) minJoyValues[i] = channels[i];
                            if (channels[i] > maxJoyValues[i]) maxJoyValues[i] = channels[i];
                        }
                    }
                    bool anyChanged = false;
                    for (int i = 0; i < 4; i++) {
                        int16_t val;
                        switch (i) { case 0: val = channels[3]; break; case 1: val = channels[2]; break;
                                     case 2: val = channels[0]; break; case 3: val = channels[1]; break; }
                        int16_t newJoy;
                        if (val >= centers[i] - softZones[i] && val <= centers[i] + softZones[i]) {
                            newJoy = 0;
                        } else if (val < centers[i] - softZones[i]) {
                            newJoy = map(val, minJoyValues[i], centers[i] - softZones[i], outMin[i], 0);
                            newJoy = constrain(newJoy, outMin[i], 0);
                        } else {
                            newJoy = map(val, centers[i] + softZones[i], maxJoyValues[i], 0, outMax[i]);
                            newJoy = constrain(newJoy, 0, outMax[i]);
                        }
                        if (abs(newJoy - lastValues[i]) >= threshold) anyChanged = true;
                        joystick[i] = newJoy;
                    }
                    joystick[4] = constrain(map(channels[6], 1000, 2000, 0, 1),0,1);
                    joystick[5] = constrain(map(channels[7], 1000, 2000, 0, 1),0,1);
                    if (anyChanged) lastUpdate = millis();
                    for (int i = 0; i < 4; i++) lastValues[i] = joystick[i];
                    changed = true;
                }
            }
        }
        // Always check timeout, even if serial buffer was empty
        if (millis() - lastUpdate > timeoutMs) {
            for (int i = 0; i < 4; i++) joystick[i] = 0;
        }
        return changed;
      }    
      int16_t *getJoystick(bool raw_values=false) { 
        if(raw_values==false) return joystick;
        else return channels;
      }
  private:
      HardwareSerial &serial;
      bool dynamicRanges=false;
      uint8_t buffer[32];
      int idx = 0;
      int threshold = 1;
      int16_t channels[12];
      int16_t joystick[6];
      int16_t lastValues[4]; // store last mapped joystick values
      unsigned long lastUpdate = 0;
      const unsigned long timeoutMs = 5000; // 5s timeout
      // Joystick settings (leftJoyX, leftJoyY, rightJoyX, rightJoyY) (MotorX, MotorY, actuatorX, actuatorY)
      #if  MAIN_ROBOT==1
        int16_t maxJoyValues[4]=    {2000, 2000, 2000, 2000}; // Joystick max values
        int16_t minJoyValues[4]=    {1000, 1000, 1000, 1000}; // Joystick min values
        int16_t centers[4] =        {1564, 1553, 1583, 1575}; // Center of each joystick
        int16_t softZones[4] =      {20, 20, 20, 20};         // Joystick drift ranges
        int16_t outMin[4] =         {-30, -30, -30, -30};     // Mapped min range
        int16_t outMax[4] =         {30, 30, 30, 30};         // Mapped max range
      #else
        int16_t maxJoyValues[4]=    {1979, 1971, 2000, 2000}; // Joystick max values
        int16_t minJoyValues[4]=    {1045, 1000, 1071, 1060}; // Joystick min values
        int16_t centers[4] =        {1621, 1623, 1620, 1616}; // Center of each joystick
        int16_t softZones[4] =      {70, 70, 50, 50};         // Joystick drift ranges
        int16_t outMin[4] =         {-30, -30, -30, -30};     // Mapped min range
        int16_t outMax[4] =         {30, 30, 30, 30};         // Mapped max range
      #endif
};

//------------------------------------------------------------
//  Data screen and messages controller class (2.42OLED-IIC)
//------------------------------------------------------------

const uint8_t font8x8[][6] PROGMEM = { // 8x8 font stored in memery for screen
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 0  (null)
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 1
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 2
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 3
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 4
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 5
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 6
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 7
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 8
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 9
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 10
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 11
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 12
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 13
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 14
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 15
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 16
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 17
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 18
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 19
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 20
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 21
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 22
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 23
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 24
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 25
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 26
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 27
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 28
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 29
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 30
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 31
  {0x00,0x00,0x00,0x00,0x00,0x00}, // 32  space
  {0x00,0x00,0x5C,0x5C,0x00,0x00}, // 33  !
  {0x00,0x0E,0x00,0x0E,0x00,0x00}, // 34  "
  {0x28,0x7C,0x28,0x7C,0x28,0x00}, // 35  #
  {0x00,0x24,0x2A,0x7F,0x2A,0x12}, // 36  $
  {0x00,0x46,0x26,0x10,0xC8,0xC4}, // 37  %
  {0x00,0x34,0x4A,0x54,0x20,0x50}, // 38  &
  {0x00,0x00,0x0E,0x00,0x00,0x00}, // 39  '
  {0x00,0x3C,0x42,0x00,0x00,0x00}, // 40  (
  {0x00,0x42,0x3C,0x00,0x00,0x00}, // 41  )
  {0x10,0x54,0x38,0x54,0x10,0x00}, // 42  *
  {0x10,0x10,0x7C,0x10,0x10,0x00}, // 43  +
  {0x00,0xC0,0x40,0x00,0x00,0x00}, // 44  ,
  {0x10,0x10,0x10,0x10,0x00,0x00}, // 45  -
  {0x00,0x40,0xC0,0x00,0x00,0x00}, // 46  .
  {0x40,0x20,0x10,0x08,0x04,0x00}, // 47  /
  {0x3C,0x42,0x42,0x42,0x3C,0x00}, // 48  0
  {0x00,0x44,0x7E,0x40,0x00,0x00}, // 49  1
  {0x64,0x52,0x52,0x52,0x4C,0x00}, // 50  2
  {0x24,0x42,0x42,0x4A,0x34,0x00}, // 51  3
  {0x30,0x28,0x24,0x7E,0x20,0x00}, // 52  4
  {0x2E,0x4A,0x4A,0x4A,0x32,0x00}, // 53  5
  {0x3C,0x4A,0x4A,0x4A,0x30,0x00}, // 54  6
  {0x02,0x02,0x62,0x12,0x0E,0x00}, // 55  7
  {0x34,0x4A,0x4A,0x4A,0x34,0x00}, // 56  8
  {0x0C,0x52,0x52,0x52,0x3C,0x00}, // 57  9
  {0x00,0x44,0xC0,0x00,0x00,0x00}, // 58  :
  {0x00,0xC4,0x40,0x00,0x00,0x00}, // 59  ;
  {0x10,0x28,0x44,0x00,0x00,0x00}, // 60  <
  {0x28,0x28,0x28,0x28,0x00,0x00}, // 61  =
  {0x44,0x28,0x10,0x00,0x00,0x00}, // 62  >
  {0x04,0x02,0x52,0x0A,0x04,0x00}, // 63  ?
  {0x3C,0x42,0x5A,0x56,0x2C,0x00}, // 64  @
  {0x7C,0x12,0x12,0x12,0x7C,0x00}, // 65  A
  {0x7E,0x4A,0x4A,0x4A,0x34,0x00}, // 66  B
  {0x3C,0x42,0x42,0x42,0x24,0x00}, // 67  C
  {0x7E,0x42,0x42,0x42,0x3C,0x00}, // 68  D
  {0x7E,0x4A,0x4A,0x4A,0x42,0x00}, // 69  E
  {0x7E,0x0A,0x0A,0x0A,0x02,0x00}, // 70  F
  {0x3C,0x42,0x52,0x52,0x34,0x00}, // 71  G
  {0x7E,0x08,0x08,0x08,0x7E,0x00}, // 72  H
  {0x00,0x42,0x7E,0x42,0x00,0x00}, // 73  I
  {0x20,0x42,0x3E,0x02,0x00,0x00}, // 74  J
  {0x7E,0x18,0x24,0x42,0x00,0x00}, // 75  K
  {0x7E,0x40,0x40,0x40,0x00,0x00}, // 76  L
  {0x7E,0x0C,0x30,0x0C,0x7E,0x00}, // 77  M
  {0x7E,0x0C,0x30,0x40,0x7E,0x00}, // 78  N
  {0x3C,0x42,0x42,0x42,0x3C,0x00}, // 79  O
  {0x7E,0x12,0x12,0x12,0x0C,0x00}, // 80  P
  {0x3C,0x42,0x52,0x22,0x5C,0x00}, // 81  Q
  {0x7E,0x12,0x12,0x32,0x4C,0x00}, // 82  R
  {0x24,0x4A,0x4A,0x52,0x24,0x00}, // 83  S
  {0x02,0x02,0x7E,0x02,0x02,0x00}, // 84  T
  {0x3E,0x40,0x40,0x40,0x3E,0x00}, // 85  U
  {0x1E,0x60,0x40,0x60,0x1E,0x00}, // 86  V
  {0x3E,0x40,0x38,0x40,0x3E,0x00}, // 87  W
  {0x42,0x24,0x18,0x24,0x42,0x00}, // 88  X
  {0x06,0x08,0x70,0x08,0x06,0x00}, // 89  Y
  {0x42,0x62,0x52,0x4A,0x46,0x00}, // 90  Z
  {0x7E,0x42,0x42,0x00,0x00,0x00}, // 91  [
  {0x04,0x08,0x10,0x20,0x40,0x00}, // 92  backslash
  {0x42,0x42,0x7E,0x00,0x00,0x00}, // 93  ]
  {0x08,0x04,0x7E,0x04,0x08,0x00}, // 94  ^
  {0x80,0x80,0x80,0x80,0x80,0x00}, // 95  _
  {0x00,0x02,0x04,0x08,0x00,0x00}, // 96  `
  {0x20,0x54,0x54,0x54,0x78,0x00}, // 97  a
  {0x7E,0x48,0x48,0x48,0x30,0x00}, // 98  b
  {0x38,0x44,0x44,0x44,0x20,0x00}, // 99  c
  {0x30,0x48,0x48,0x48,0x7E,0x00}, // 100 d
  {0x38,0x54,0x54,0x54,0x58,0x00}, // 101 e
  {0x00,0x48,0x7C,0x4A,0x02,0x00}, // 102 f
  {0x98,0xA4,0xA4,0xA4,0x7C,0x00}, // 103 g
  {0x7E,0x08,0x08,0x08,0x70,0x00}, // 104 h
  {0x00,0x48,0x7A,0x40,0x00,0x00}, // 105 i
  {0x40,0x80,0x84,0x7A,0x00,0x00}, // 106 j
  {0x7E,0x10,0x28,0x44,0x00,0x00}, // 107 k
  {0x00,0x42,0x7E,0x40,0x00,0x00}, // 108 l
  {0x7C,0x04,0x78,0x04,0x78,0x00}, // 109 m
  {0x7C,0x04,0x04,0x04,0x78,0x00}, // 110 n
  {0x38,0x44,0x44,0x44,0x38,0x00}, // 111 o
  {0xFC,0x24,0x24,0x24,0x18,0x00}, // 112 p
  {0x18,0x24,0x24,0x24,0xFC,0x00}, // 113 q
  {0x7C,0x08,0x04,0x04,0x08,0x00}, // 114 r
  {0x48,0x54,0x54,0x54,0x24,0x00}, // 115 s
  {0x04,0x3E,0x44,0x40,0x20,0x00}, // 116 t
  {0x3C,0x40,0x40,0x40,0x3C,0x00}, // 117 u
  {0x1C,0x20,0x40,0x20,0x1C,0x00}, // 118 v
  {0x3C,0x40,0x38,0x40,0x3C,0x00}, // 119 w
  {0x44,0x28,0x10,0x28,0x44,0x00}, // 120 x
  {0x9C,0xA0,0x60,0x20,0x1C,0x00}, // 121 y
  {0x44,0x64,0x54,0x4C,0x44,0x00}, // 122 z
  {0x10,0x6C,0x44,0x00,0x00,0x00}, // 123 {
  {0x00,0x7E,0x00,0x00,0x00,0x00}, // 124 |
  {0x44,0x6C,0x10,0x00,0x00,0x00}, // 125 }
  {0x08,0x04,0x08,0x10,0x08,0x00}, // 126 ~
  {0x00,0x00,0x00,0x00,0x00,0x00}  // 127 DEL

  /* FONT TEST:
      oled.println("abcdefghijklm");
      oled.println("nopqrstuvwxyz");
      oled.println("ABCDEFGHIJKLM");
      oled.println("NOPQRSTUVWXYZ");
      oled.println("0123456789");
      oled.println(":;!#$%&'()*+,-./");
      oled.println("<=>?@[]^_`{|}~\"\\");
  */
};
class OLEDIIC_interface {
  // Class which handles talking to the (2.42OLED-IIC) screen and sending it text using a custom system
  private:
      uint8_t screenAddress;
      uint8_t screenWidth;
      uint8_t screenHeight;
      uint8_t screenRow;
  
      uint8_t cursorX = 0;
      uint8_t cursorY = 0;
      
      // Send I2C command
      void sendCommand(uint8_t cmd) {
          Wire.beginTransmission(screenAddress);
          Wire.write(0x00);
          Wire.write(cmd);
          Wire.endTransmission();
      }
  
      // Add font character to screen
      void printChar(char character) {
          // Read index from ASCII table
          uint8_t charIndex = character; 
          
          // Turn out of bound characters into spaces 
          if (character < 32 || character > 127) charIndex = 32;  
          
          // If character space, do special handling, else send font data to screen
          if(character==32){sendData(0x00);sendData(0x00);sendData(0x00);sendData(0x00);return;}
          for (uint8_t i = 0; i < 6; i++) {
              uint8_t data = pgm_read_byte(&font8x8[charIndex][i]);
              sendData(data);
          }
      }
  
  public:
      // Screen address and size variables
      OLEDIIC_interface(uint8_t address, uint8_t width, uint8_t height): screenAddress(address), screenWidth(width), screenHeight(height), screenRow(height / 8){}
  
      // OLED initialization sequence 
      void begin() {
          Wire.begin();
  
          // Start display off
          sendCommand(0xAE);     
  
          // Setup clock divide ratio, with a refreash rate of ~100Hz
          sendCommand(0xD5);     
          sendCommand(0x80);     
          
          // Set number of screen lines, 64
          sendCommand(0xA8);     
          sendCommand(0x3F);     
          
          // Set display offset, none
          sendCommand(0xD3);     
          sendCommand(0x00);     
          
          // Set start line to 0
          sendCommand(0x40); 
          
          // Enable capacitor to ensure screen has sificent voltage
          sendCommand(0x8D);     
          sendCommand(0x14);     
          
          // Set memory addressing mode
          sendCommand(0x20);    
          sendCommand(0x00);  
          
          // Set screen orientation 
          sendCommand(0xA1);     
          sendCommand(0xC8);  
          
          // Set default com pin hardware mappings
          sendCommand(0xDA);     
          sendCommand(0x12);     
          
          // Set the contrast to the default
          sendCommand(0x81);    
          sendCommand(0xCF);     
          
          // Pixle charge time, defult, affects brightness and stability
          sendCommand(0xD9);     
          sendCommand(0xF1);    
          
          // Set idial voltage to prevent ghosting, VCOMH deselect level (0.77xVCC)
          sendCommand(0xDB);     
          sendCommand(0x40);     
          
          // Set default RAM usage for graphics
          sendCommand(0xA4);    
          sendCommand(0xA6);    
          
          // Set screen scroll mode
          sendCommand(0x2F);
          
          // Turn on and clear display
          sendCommand(0xAF);     
          clear(false);
      }
  
      // Send data to screen
      void sendData(uint8_t data) {
          Wire.beginTransmission(screenAddress);
          Wire.write(0x40);  // Data mode
          Wire.write(data);
          Wire.endTransmission();
      }
  
      // Set screen to black
      void clear(bool useWhiteSpace) {
          // Fill entire screen with zeros
          if(useWhiteSpace){
            uint8_t posX = cursorX;
            while (posX < screenWidth) {
              sendData(0x00);    
              posX+=1;    
            }
          }
          else{
            for (uint8_t posY = 0; posY < screenRow; posY++) {
                setCursor(posY, 0);
                for (uint8_t posX = 0; posX < screenWidth; posX++) {
                  sendData(0x00);
                }
            }
        }
      }
  
      // Set cursor position on screen
      void setCursor(uint8_t posY, uint8_t posX) {
          cursorY = posY;
          cursorX = posX;
          
          sendCommand(0xB0 + posY);                       
          sendCommand(0x00 + (posX & 0x0F));            
          sendCommand(0x10 + ((posX >> 4) & 0x0F));   
      }
  
      // Handle regular print strings
      void print(const char* str) {
          while (*str) {
              printChar(*str);
              str++;
          }
      }
  
      // Handle println strings
      void println(const char* str) {
          print(str);
          cursorY++;
          cursorX = 0;
          if (cursorY < screenRow) setCursor(cursorY, 0);
      }
  
      // Draw single pixle 
      void drawPixel(uint8_t posY, uint8_t posX, uint8_t pattern) {
          setCursor(posY, posX);
          sendData(pattern);
      }
  };

class MessageStore {
  // Class which handles how the diffrent types of infomation is displayed onto the screen
  private:
      OLEDIIC_interface &screen;
      static const uint16_t MAX_MESSAGES = 12;
      static const uint16_t MESSAGE_SIZE = 128;
      char messages[MAX_MESSAGES][MESSAGE_SIZE];

      int currentPage=0;
      bool cyclePage=true;
      uint8_t id_index[6];
      unsigned long last_message_time = 0;
      unsigned long timeout_cycle_time = -1;
      int8_t cycle_priod=5;
      int cycle_time=5000;
      int wait_time=0;
      // Sizes of each message type, TODO
      struct PageSize {
        uint8_t start;
        uint8_t end;
      };
      PageSize ranges[6] = {
        {1, 1},    // FAULT
        {2, 2},    // CWARNING
        {3, 3},    // WARNING
        {4, 7},    // COMMS
        {8, 8},    // STATUS
        {9, 12}    // DATA
      };
      const int NUM_PAGES = sizeof(ranges)/sizeof(ranges[0]);
  
  public:
      MessageStore(OLEDIIC_interface &screenClass): screen(screenClass) {}
      //void addMessage(uint8_t id, char* msg) {

      //}
  
      // Sets message to id in data base
      void setMessage(uint8_t id, char* msg) { 
        strncpy(messages[id], msg, MESSAGE_SIZE - 1);
        messages[id][MESSAGE_SIZE - 1] = '\0';
       }

      // Gets value of message from database
      char* getMessage(uint8_t id) { return messages[id]; }
      
      // Deletes a message id from database
      void deleteMessage(uint8_t id) { messages[id][0] = '\0'; }

      // Checks of a type of message is empty for message cycling
      bool rangeEmpty(uint8_t start, uint8_t end) {
          for (uint8_t i = start; i <= end; i++) {
              if (messages[i] != nullptr) return false;
          }
          return true;
      }

      int getNextValidPage() {
        int page = currentPage;
        for (int i = 0; i < NUM_PAGES; i++) {
            page+=1;
            if(page>=NUM_PAGES) page=0;
            if (!rangeEmpty(ranges[page].start, ranges[page].end)) return page;
        }
        return currentPage;
    }

      void systemMessageUpdate(unsigned long current_time) {
        if(cyclePage==false || (current_time-last_message_time<cycle_time+wait_time)) return;
        last_message_time=current_time;
        wait_time=0;

        currentPage=getNextValidPage();
        //char str[12];             
        //sprintf(str, "%d", currentPage);

        screen.setCursor(0, 0);
        screen.println("test");

        //screen.clear(false);
        //screen.setCursor(0, 0);
        //screen.print(messages[ranges[currentPage].start]);
        //screen.print(" | ");
        //screen.print(str);
      }
  
      void addMessageToArduino(int messageId, int operation, uint8_t* message) {
        switch (operation) {
          case 0:
            for (int i = 0; i < 6; i++) {
              if (messageId >= ranges[i].start && messageId <= ranges[i].end) {
                messageId = ranges[i].start;
                break;
              }
            }
            setMessage(messageId, message);
            //screen.setCursor(0, 0);
            //screen.print(message);
            //screen.print(" | ");
            //char id[4];                         
            //sprintf(id, "%d", messageId);  
            //screen.print(id);
            //screen.print(messages[messageId]);
            //screen.clear(true);
            break;
          case 1:
            cyclePage=true;
            break;
          case 2:
            cyclePage=false;
            break;
          case 3:
            last_message_time=millis()-cycle_time-wait_time;
            break;
          case 4:
            setMessage(messageId, message);
            break;
          case 5:
            deleteMessage(messageId);
            break;
          case 6:
            wait_time=10*1000;
            screen.setCursor(0, 0);
            screen.print(message);
            screen.clear(true);
            break;
          case 7:
            wait_time=60*1000;
            screen.setCursor(0, 0);
            screen.print(message);
            screen.clear(true);
            break;
          case 8:
            cyclePage=false;
            screen.setCursor(0, 0);
            screen.print(message);
            screen.clear(true);
            break;
        }
      }
  };
  #endif
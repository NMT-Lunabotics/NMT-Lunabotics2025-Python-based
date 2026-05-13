#pragma once
#ifndef HELPERS_H
#define HELPERS_H

//Condition that changes actuator code to accept pwm, and GND, instead of current setup that uses pinout, pwm, and GND
bool MD04_drivers = false;

#include <Arduino.h>
#include <Wire.h>
#include "arduino_lib.hpp"

#define MEDIAN_SIZE 8 // Median filter window size for potentionmeter smoothing

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

//////// Actuator Class ////////
class Actuator {
  PWM_Driver pwm_driver;
  float stroke;
  float pot_min;
  float pot_max;
  float act_max_vel;
  float pos_mm;
  float min_pos;
  float max_pos;
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

  void stop() {
    pwm_driver.stop();
  }

  void resetPIDIntegral() {
    pid.resetIntegral();
  }
};

///////// Motor Class ////////
class Motor {
    OutPin dac1;
    OutPin dac2;
    OutPin enable;

    int motor_max_vel; //rpm

    bool reverse = false;

public:
    Motor(OutPin dac1, OutPin dac2, OutPin enable, int motor_max_vel, bool reverse) : dac1(dac1), dac2(dac2), enable(enable), motor_max_vel(motor_max_vel), reverse(reverse) {}

    void motor_ctrl(int signed_speed) {
        // Convert motor speeds (in rpm) to PWM values (0-255)
        // Constrain speeds to max velocity first
        if (signed_speed == 0) {
            stop();
            return;
        }
        enable.write(1);

        signed_speed = constrain(signed_speed, -motor_max_vel, motor_max_vel);
        // Map the absolute speed values to PWM range
        int motor_speed = map(abs(signed_speed), 0, motor_max_vel, 0, 255);
        if (signed_speed > 0) {
            dac1.write_pwm_raw(motor_speed);
            dac2.write_pwm_raw(0);
        } else {
            dac1.write_pwm_raw(0);
            dac2.write_pwm_raw(motor_speed);
        }
    }

    void stop() {
      enable.write(0);
      dac1.write_pwm_raw(0);
      dac2.write_pwm_raw(0);
    }
};

class SimpleMotor {
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



///////// MPU6050 IMU Class /////////
class MPU6050 {
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

///////// FSIA6B IBUS Class /////////
class IBusReader {
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
      int16_t channels[8];
      int16_t joystick[5];
      int16_t lastValues[4]; // store last mapped joystick values
      unsigned long lastUpdate = 0;
      const unsigned long timeoutMs = 2000; // 0.5s timeout
      // Joystick settings (leftJoyX, leftJoyY, rightJoyX, rightJoyY) (MotorX, MotorY, actuatorX, actuatorY)
      int16_t maxJoyValues[4]=    {1979, 1971, 2000, 2000}; // Joystick max values
      int16_t minJoyValues[4]=    {1045, 1000, 1071, 1060}; // Joystick min values
      int16_t centers[4] =        {1621, 1623, 1620, 1616}; // Center of each joystick
      int16_t softZones[4] =      {50, 50, 50, 50};        // Joystick drift ranges
      int16_t outMin[4] =         {-30, -30, -30, -30};     // Mapped min range
      int16_t outMax[4] =         {30, 30, 30, 30};         // Mapped max range
};

class LCD2004 {
  private:
      struct LogMessage {
          char code;
          int id;
          int priority;
          const char* msg;
          unsigned long time;
      };
  
      #define MAX_LOGS 20
      LogMessage logBuffer[MAX_LOGS];
      bool showErrorId;
      bool showErrorIdBetweenScreens;
      int logCount = 0;
      uint8_t addr, cols, rows;
      uint8_t backlight = 0x08;
  
      void write(uint8_t data) {
          // Write I2C data to display at address and set backlight at same time
          Wire.beginTransmission(addr);
          Wire.write(data | backlight);
          Wire.endTransmission();
      }
      void pulseEnable(uint8_t data) {
          // Pulses lcd enable pin to tell it to start recviving commands
          write(data | 0x04);
          delayMicroseconds(1);
          write(data & ~0x04);
          delayMicroseconds(50);
      }
      void write4bits(uint8_t value) {
          // Enables lcd and latches the lcd to I2C data
          write(value);
          pulseEnable(value);
      }
      void send(uint8_t value, uint8_t mode) {
          write4bits((value & 0xF0) | mode);
          write4bits(((value << 4) & 0xF0) | mode);
      }
      void command(uint8_t value) { 
          // Sends command to lcd using 0x00
          send(value, 0x00); 
      }
      void writeChar(char c) { 
          // Writes character to lcd using 0x01
          send(c, 0x01); 
      }
      void clear() {
          // Clear display by sending a blank msg using command
          command(0x01);
          delay(2);
      }
      void setCursor(uint8_t col, uint8_t row) {
          // Set cursor position on screen
          static const uint8_t rowOffsets[4] = {0x00, 0x40, 0x14, 0x54};
          command(0x80 | (col + rowOffsets[row]));
      }
  
      void cleanExpiredMessages() {
          unsigned long now = millis();
          for (int i = 0; i < logCount;) {
              if (logBuffer[i].time != (unsigned long)-1 && now > logBuffer[i].time) {
                  for (int j = i; j < logCount - 1; j++) logBuffer[j] = logBuffer[j + 1];
                  logCount--;
              } else i++;
          }
      }
  
      int getMessagesByCode(char code, LogMessage* out[], int maxCount) {
          cleanExpiredMessages();
          int count = 0;
          for (int i = 0; i < logCount && count < maxCount; i++) {
              if (logBuffer[i].code == code) out[count++] = &logBuffer[i];
          }
          if (count > 1) {
              qsort(out, count, sizeof(LogMessage*), [](const void* a, const void* b) -> int {
                  LogMessage* m1 = *(LogMessage**)a;
                  LogMessage* m2 = *(LogMessage**)b;
                  return m1->priority - m2->priority;
              });
          }
          return count;
      }
  
  
  
  public:
      // Lcd class with it's address, and size
      LCD2004(uint8_t address = 0x27, uint8_t cols_ = 20, uint8_t rows_ = 4) : addr(address), cols(cols_), rows(rows_) {}
      void begin() {
          // Start wire I2C connection
          Wire.begin();
          delay(50);
          // Start talking to screen while it is initalizing, then swich to 4-bit mode(uses less pins) 
          write4bits(0x30); delayMicroseconds(4500);
          write4bits(0x30); delayMicroseconds(4500);
          write4bits(0x30); delayMicroseconds(150);
          write4bits(0x20);
          // Set default display settings, turn of display(stops flickers), clear display, set default writing mode, turn on display with no cursor
          command(0x28);
          command(0x08);
          command(0x01);
          delay(2);
          command(0x06);
          command(0x0C);
      }
      void backlightOn(bool on = true) {
          // Turn lcd screen backlight on/off
          backlight = on ? 0x08 : 0x00;
          write(0);
      }
  
      void displayLogs(uint16_t pageDelayMs = 3000) {
          // Every loop clear old messages and clear screen if we do not have messages to display
          cleanExpiredMessages();
          if (logCount == 0) { clear(); return; }
          //  Loop through predefined data types to display on cycle
          char codes[] = { 'F','E','C' };
          for (int c = 0; c < 3; c++) {
              LogMessage* messages[MAX_LOGS];
              int count = getMessagesByCode(codes[c], messages, MAX_LOGS);
              if (count == 0) continue;
      
              // Build a single text stream for all messages of this code
              String combinedText = "";
              for (int i = 0; i < count; i++) {
                  combinedText += messages[i]->msg;
                  combinedText += " "; // separator between messages
              }
      
              size_t textIndex = 0;
              while (textIndex < combinedText.length()) {
                  clear();
                  uint8_t row = 0;
      
                  // Print header
                  char header[8];
                  bool singleMsg = (count == 1);
                  if (singleMsg) snprintf(header, sizeof(header), "[%c%02d] ", messages[0]->code, messages[0]->id);
                  else snprintf(header, sizeof(header), "[%c] ", messages[0]->code);
                  uint8_t col = 0;
                  setCursor(0, row);
                  for (size_t h = 0; header[h] && col < cols; h++, col++) writeChar(header[h]);
      
                  // Fill remaining space on screen
                  while (row < rows && textIndex < combinedText.length()) {
                      setCursor(col, row);
                      while (col < cols && textIndex < combinedText.length()) {
                          char c = combinedText[textIndex++];
                          if (c == '\n') { col = cols; break; } // force newline
                          writeChar(c);
                          col++;
                      }
                      row++;
                      col = 0;
                  }
      
                  delay(pageDelayMs);
              }
          }
      
          // Clear if nothing left
          if (logCount == 0) clear();
      }
      
  
      void setMessageLog(char code, int id, int priority, const char* msg, long timeSeconds = -1, bool showErrorId = true, bool showErrorIdBetweenScreens = true) {
          unsigned long set_time = (timeSeconds > 0) ? timeSeconds * 1000 + millis() : (unsigned long)-1;
          LogMessage entry = { code, id, priority, msg, set_time, showErrorId, showErrorIdBetweenScreens };
          for (int i = 0; i < logCount; i++) {
              if (logBuffer[i].code == code && logBuffer[i].id == id) {
                  logBuffer[i] = entry;
                  return;
              }
          }
          if (logCount < MAX_LOGS) logBuffer[logCount++] = entry;
          else Serial.println("Log buffer full");
      }
      
  };
#endif
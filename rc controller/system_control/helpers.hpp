#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>
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
    speed = constrain(speed, -act_max_vel, act_max_vel);
    speed = speed / act_max_vel * 255;
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
        
        if (reverse) {
            signed_speed = -signed_speed;
        }

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

#endif

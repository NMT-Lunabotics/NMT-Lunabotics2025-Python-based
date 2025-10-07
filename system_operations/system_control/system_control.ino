#include "helpers.hpp"
MPU6050 IMU;
// #include <Servo.h>

//--------------- Debug settings ---------------

// List of components flags to enable/disable for testing
bool error_leds_enabled = true;
bool motors_enabled = true;
bool backet_actuator_enabled = false;
bool arm_actuators_enabled = false;
bool servo_motor_enabled = true;

// List of faults to disable
bool serial_communication_timeout_fault = true;
bool component_timeout_faults = true;

// Debug mode flag
bool debug_mode = false; 
bool sensor_output = 0; //1: IMU

//--------------- Actuators ---------------

// Driver 1 pins
#define DRV11_PWM_PIN 6
#define DRV11_DIR1_PIN 34
#define DRV11_DIR2_PIN 36
#define DRV12_PWM_PIN 7
#define DRV12_DIR1_PIN 38
#define DRV12_DIR2_PIN 40

// Driver 2 pins
#define DRV21_PWM_PIN 9
#define DRV21_DIR1_PIN 44
#define DRV21_DIR2_PIN 42
#define DRV22_PWM_PIN 8
#define DRV22_DIR1_PIN 46
#define DRV22_DIR2_PIN 48

// Actuator potentiometer read pins
#define POTL_PIN A1
#define POTR_PIN A0
#define POTB_PIN A3

// Allowed actuator stroke lengths (mm)
#define ALR_STROKE 191
#define AB_STROKE 140

// Actuator potentiometer min and max values
#define AL_POT_MIN 47
#define AL_POT_MAX 893
#define AR_POT_MIN 0
#define AR_POT_MAX 840
#define AB_POT_MIN 30
#define AB_POT_MAX 782

float bucket_min = 20;            // mm
float bucket_max = 110;           // mm
float bucket_absolute_max = 115;  // mm
float act_end_tolerance = 1;      // mm

float act_max_vel = 25;   // mm/s
float act_fix_err = 3.0;  // mm
float act_max_err = 5.0;  // mm

// Actuator target position
int aLR_tgt = -1;
int aB_tgt = -1;

// Actuator speed and position variables
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

//--------------- MOTORS ---------------

const int DACL1_PIN = 2;
const int DACL2_PIN = 3;
const int DACR1_PIN = 4;
const int DACR2_PIN = 5;
const int EN_PIN = 32;  // Common for both motors

// Max allowed motor velocity (rpm)
int motor_max_vel = 30; 

// Motor speed and target rotation and radius variables
int mL_speed = 0;
int mR_speed = 0;

int mLR_rotation_speed = 0;
int mLR_rotation = 0;
int mLR_arc_radius = 0;
float robot_width =1.5; //m

// TODO implement servo logic
//  #define SERVO_PIN 22

// Servo
bool servo_state = false;

//--------------- LEDS ---------------

// LED pins 
#define LEDR_PIN 24
#define LEDY_PIN 26
#define LEDG_PIN 28
#define LEDB_PIN 30
// LED saved states
short int led_r = 0;
short int led_y = 0;
short int led_g = 0;
short int led_b = 0;
// Available LED states
enum LedState { OFF = 0, NONE = -1, ON = 1, BLINK = 2 };

//--------------- IMU ---------------
// IMU gyroscope and accelerometer variables for robot rotations
#define IMU_offset_bias_samples 25
#define IMU_filter_constant 0.2
float IMU_offset_bias=0;
float IMU_rate = 0;
float IMU_yaw=0;
unsigned long last_IMU_time =0;
float IMU_raw_home_bias = 0;
float IMU_local_home_bias = 0;
bool reset_IMU_local_home=false;
bool update_IMU_raw_home=true;
float IMU_filter_rate=0;

#define IMU_angle_bias_samples 25
float IMU_yaw_scale=0;

//--------------- SYSTEM VARIABLES ---------------

// Timing
int update_rate = 200;                // hz
int update_actuator_feedback = 1000;  // hz
int feedback_rate = 10;               // hz
int reset_int_rate = 10;              // hz
unsigned long last_update_time = 0;
unsigned long last_update_actuator_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long led_blink_time=0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 2000;  // 2 second timeout for failed serial commication
unsigned long last_message_time = 0;
bool emergency_stop = false;
bool doomsday = false;
String system_fault_msg = "";
bool system_started = false;

// Motor and actuator timeout variables, shutdown motors and actuators if a message is not reviced within timeout period
unsigned long last_actuator_cmd_time = 0; 
unsigned long last_motor_cmd_time = 0;
int motor_timeout = 2000;
int actuator_timeout = 2000;

// Serial and state
bool serial_connection_established=false;
bool receiving_message = false;
bool at_bucket_min = false;
bool at_bucket_max = false;
bool dual_actuator_correct = false;
int serial_index = 0;
int expected_length = -1;
const int SERIAL_BUFFER_SIZE = 128;
byte serial_buffer[SERIAL_BUFFER_SIZE];

// Set up PID controllers
PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);
PID pidB(3.0, 0.001, 0.4);
float vel_gain = 2.5;

// Set up actuators
PWM_Driver left_driver(DRV12_PWM_PIN, DRV12_DIR1_PIN, DRV12_DIR2_PIN, false);
Actuator act_left(left_driver, pidL, POTL_PIN, AL_POT_MIN, AL_POT_MAX, ALR_STROKE, act_max_vel);

PWM_Driver right_driver(DRV11_PWM_PIN, DRV11_DIR1_PIN, DRV11_DIR2_PIN, false);
Actuator act_right(right_driver, pidR, POTR_PIN, AR_POT_MIN, AR_POT_MAX, ALR_STROKE, act_max_vel);

PWM_Driver bucket_driver(DRV21_PWM_PIN, DRV21_DIR1_PIN, DRV21_DIR2_PIN, true);
Actuator act_bucket(bucket_driver, pidB, POTB_PIN, AB_POT_MIN, AB_POT_MAX, AB_STROKE, act_max_vel, bucket_min, bucket_max);

// Set up motors
OutPin motor_left_dac1(DACL1_PIN);
OutPin motor_left_dac2(DACL2_PIN);
OutPin motor_right_dac1(DACR1_PIN);
OutPin motor_right_dac2(DACR2_PIN);
OutPin motor_enable(EN_PIN);
Motor motor_left(motor_left_dac1, motor_left_dac2, motor_enable, motor_max_vel, false);
Motor motor_right(motor_right_dac1, motor_right_dac2, motor_enable, motor_max_vel, true);

// Set up LEDs
OutPin ledr_pin(LEDR_PIN);
OutPin ledy_pin(LEDY_PIN);
OutPin ledg_pin(LEDG_PIN);
OutPin ledb_pin(LEDB_PIN);

// void processMessage(byte* data, int length);
void stop_all();
void systemFault(bool criticalError = false,String fault_msg="", String error_msg="", LedState y =NONE, LedState g =NONE, LedState b=NONE);
void calibrateIMU();
void calibrateIMUAngle();
void updateIMUData(bool useHomeBias=false);
void sendSerialFeedback(char command, uint8_t* data, size_t dataLen);

void setup() {
  delay(5);
  Serial.begin(115200);
  Serial.flush();
  // Set default led status
  systemFault(false,"","", NONE, BLINK, BLINK);
  // Calibrate the IMU sensors drift factor
  calibrateIMU();
  calibrateIMUAngle();
  Serial.print("IMU_pitch_angle_bias: ");
  Serial.print(IMU_pitch_angle_bias);
  Serial.print(" IMU_roll_angle_bias: ");
  Serial.println(IMU_roll_angle_bias);
  for (int i = 0; i < 10; i++) {
    aL_pos = act_left.update_pos();
    aR_pos = act_right.update_pos();
    aB_pos = act_bucket.update_pos();
  }
  Serial.println("Arduino system_control.ino started.");
}

void loop() {
  current_time = millis();
  // Read serial and process messages while being Non-blocking 
  while (Serial.available() > 0) {
    byte b = Serial.read();
    if (!receiving_message) {
        if (b == 0x02) {
            receiving_message = true;
            serial_index = 0;
            expected_length = -1;
        }
    } else {
        if (expected_length == -1) {
            expected_length = b;
            if (expected_length <= 0 || expected_length > SERIAL_BUFFER_SIZE) {
                receiving_message = false; // invalid length
            }
        } else {
            serial_buffer[serial_index++] = b;
            if (serial_index == expected_length + 1) { // +1 for end byte
                if (serial_buffer[serial_index-1] == 0x03) {
                    processMessage(serial_buffer, expected_length);
                } else {
                    Serial.println("End byte not found");
                }
                receiving_message = false;
            } else if (serial_index >= SERIAL_BUFFER_SIZE) {
                // overflow protection
                receiving_message = false;
            }
        }
    }
}

  if (current_time - last_message_time > estop_timeout && serial_communication_timeout_fault==true && serial_connection_established==true) systemFault(true,"Serial communication timeout.","", NONE, NONE, NONE);
  // Update actuator saved positions
  if (current_time - last_update_actuator_time >= 1000 / update_actuator_feedback) {
    last_update_actuator_time = current_time;
    aL_pos = act_left.update_pos();
    aR_pos = act_right.update_pos();
    aB_pos = act_bucket.update_pos();
  }

  if (current_time - last_update_time >= 1000 / update_rate) {
    last_update_time = current_time;

    // Ensure bucket is in bounds
    if(backet_actuator_enabled==true){
     if (aB_pos > bucket_absolute_max) systemFault(true,"Bucket position out of bounds: " + String(aB_pos),"", NONE, NONE, NONE);
     if (aB_pos < bucket_min || aB_pos > bucket_max) {
         stop_all();
         while (aB_pos < bucket_min) {
             act_bucket.vel_ctrl(5);
             aB_pos = act_bucket.update_pos();
             delay(5);
             systemFault(false,"","Bucket past minimum. Fixing...", BLINK, NONE, NONE);
         }
         while (aB_pos > bucket_max) {
             act_bucket.vel_ctrl(-5);
             aB_pos = act_bucket.update_pos();
             delay(5);
             systemFault(false,"","Bucket past maximum. Fixing...", BLINK, NONE, NONE);
         }
         act_bucket.stop();
     }
    }
    // Correct dual actuator misalignment
    if(arm_actuators_enabled==true){
    float lr_err = abs(aL_pos - aR_pos);
    if (lr_err >= act_fix_err && lr_err < act_max_err) {
      stop_all();
      float prev_err = lr_err;
      while (lr_err >= 0.5 * act_fix_err) {
        act_bucket.stop();
        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        float factor = (aL_pos - aR_pos) * vel_gain;

        act_left.vel_ctrl(aL_speed - factor);
        act_right.vel_ctrl(aR_speed + factor);
        delay(5);

        prev_err = lr_err;
        lr_err = abs(aL_pos - aR_pos);
        if (lr_err > prev_err) systemFault(true,"Actuator diverging fix failed.","", NONE, NONE, NONE);
        else systemFault(false,"","Actuator arms diverging, Fixing actuators...", BLINK, NONE, NONE);
      }
      act_left.stop();
      act_right.stop();
      ledy_pin.write(0);
    } else if (lr_err >= act_max_err) systemFault(true,"Actuator relative error too large: " + String(aL_pos) + " " + String(aR_pos),"", NONE, NONE, NONE);
  }
    // Run motors and actuators
    if (!emergency_stop) {
      // Timeout motors and actuators if a command isn't recived within the timeout period
      if(component_timeout_faults==true&&current_time-last_motor_cmd_time>=motor_timeout&&(mR_speed!=0||mL_speed!=0||mLR_rotation_speed!=0||mLR_rotation!=0)){
        mR_speed=0;
        mL_speed=0;
        mLR_rotation_speed=0;
        mLR_rotation=0;
        systemFault(false,"","Motors timed out, no motor command received within "+ String(motor_timeout/1000)+"s", NONE, NONE, NONE);
      }
       if(component_timeout_faults==true&&current_time-last_actuator_cmd_time>=actuator_timeout && (aL_speed!=0 || aR_speed !=0 || aB_speed!=0 || aLR_tgt != -1 || aB_tgt != -1 )){
        aLR_tgt = -1;
        aB_tgt = -1;
        aL_speed = 0;
        aR_speed = 0;
        aB_speed = 0;
        systemFault(false,"","Actuators timed out, no actuator command received within "+ String(actuator_timeout/1000)+"s", NONE, NONE, NONE);
      }
      // Update actuator positions.
      if (aLR_tgt >= 0) {
        act_left.tgt_ctrl(aLR_tgt);
        act_right.tgt_ctrl(aLR_tgt);
      } else {
        float factor = (aL_pos - aR_pos) * vel_gain;
        act_left.vel_ctrl(aL_speed - factor);
        act_right.vel_ctrl(aR_speed + factor);
      }
      if (aB_tgt >= 0)
        act_bucket.tgt_ctrl(aB_tgt);
      else if ((aB_speed > 0 && aB_pos < bucket_max) || (aB_speed < 0 && aB_pos > bucket_min))
        act_bucket.vel_ctrl(aB_speed);
      else
        act_bucket.stop();
      // Update motor speeds
      if(motors_enabled==true){
        if(mLR_rotation_speed!=0 && mLR_rotation!=0){
          if(mLR_arc_radius!=0){
            float mL_velocity=mLR_rotation_speed*(1-robot_width/mLR_arc_radius);
            float mR_velocity=mLR_rotation_speed*(1+robot_width/mLR_arc_radius);
            motor_left.motor_ctrl(mL_velocity);
            motor_right.motor_ctrl(mR_velocity);
          }
          else{
            motor_left.motor_ctrl(mLR_rotation_speed);
            motor_right.motor_ctrl(-mLR_rotation_speed);
          }
          if((mLR_rotation<0&&IMU_yaw<=mLR_rotation)||(mLR_rotation>0&&IMU_yaw>=mLR_rotation)){
            motor_left.stop();
            motor_right.stop();
            mLR_rotation_speed=0;
            if(reset_IMU_local_home==false) IMU_local_home_bias=IMU_yaw;
            update_IMU_raw_home=true;
            if(debug_mode==true){
              Serial.print("Yaw: ");
              Serial.println(IMU_yaw,6);
            }
            updateIMUData(true);
            sendSerialFeedback('R', nullptr, 0);
          } 
        }
        else{
            motor_left.motor_ctrl(mL_speed);
            motor_right.motor_ctrl(mR_speed);
        }
      }
    } 
    else stop_all();
  }
  // Sync feedback loop timer
  if (current_time - last_feedback_time >= 1000 / feedback_rate) last_feedback_time = current_time;
  // Reset Actuator PID's to prevent windup.
  if (current_time - last_reset_int_time >= 1000 / reset_int_rate) {
    last_reset_int_time = current_time;
    act_left.resetPIDIntegral();
    act_right.resetPIDIntegral();
    act_bucket.resetPIDIntegral();
  }
  
  // System fully started turn on green led
  if(system_started==false){
    system_started=true;
    systemFault(false,"","", NONE, ON, NONE);
  }
    // Run fault function every loop to update leds, and handle critical errors
  systemFault(false, "","", NONE, NONE, NONE);
  if(update_IMU_raw_home==true) updateIMUData(true);
  else updateIMUData(false);
  if(sensor_output==1){
    Serial.print("Yaw: ");
    Serial.println(IMU_yaw,6);
  }
}

// Read serial communication from autonomy computer and set system variables to output.
void processMessage(byte *data, int length) {
  current_time = millis();
  char type = data[0];
  if (debug_mode) {
    Serial.print("Received message of type: ");
    Serial.println(type);
    Serial.print("Length: ");
    Serial.println(length);
  }
  bool cmd_triggered = true;
  switch (type) {
    case 'A':
      {                                                 // (A, actuators) Recvived Arm and Bucket actuator speeds and positions.
        aLR_tgt = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
        aB_tgt = (int16_t)((data[3] << 8) | data[4]);
        aL_speed = -(int8_t)data[5];
        aR_speed = aL_speed;
        aB_speed = -(int8_t)data[6];
        if (debug_mode) {
          Serial.print("Arm Position: ");
          Serial.println(aLR_tgt);
          Serial.print("Bucket Position: ");
          Serial.println(aB_tgt);
          Serial.print("Arm Velocity: ");
          Serial.println(aL_speed);
          Serial.print("Bucket Velocity: ");
          Serial.println(aB_speed);
        }
        last_actuator_cmd_time=current_time;
        break;
      }
    case 'M':
      {                                                   // (M, motors) Recvived left and right motor speeds
        mR_speed = (int8_t)data[1];  
        mL_speed = (int8_t)data[2];
        if (debug_mode) {
          Serial.print("Left Speed: ");
          Serial.println(mL_speed);
          Serial.print("Right Speed: ");
          Serial.println(mR_speed);
        }
        last_motor_cmd_time=current_time;
        break;
      }
    case 'R': 
      {                                                 // (R, rotate) Recvives a rotation speed, angle, radius, and home reset, to allow arc or stationary turns       
        mLR_rotation_speed = (int8_t)data[1]; 
        mLR_rotation = -(int8_t)data[2];
        mLR_arc_radius = (int8_t)data[3];
        reset_IMU_local_home = (int8_t)data[4];
        update_IMU_raw_home=false;
        if(mLR_arc_radius==0&&mLR_rotation_speed==0&&mLR_rotation==0&&reset_IMU_local_home==true) IMU_local_home_bias=IMU_yaw;
        if (debug_mode) {
          Serial.print("Motor rotation Speed: ");
          Serial.println(mLR_rotation_speed);
          Serial.print("Motor rotation arc radius: ");
          Serial.println(mLR_arc_radius);
          Serial.print("Motor rotation angle: ");
          Serial.println(mLR_rotation);
        }
        last_motor_cmd_time=current_time;
        break;
      }
    case 'S':
      {                                                 // (S, servo motor) Recvived servo latch state
        servo_state = data[1];  
        if (debug_mode) {
          Serial.print("Servo State: ");
          Serial.println(servo_state);
        }
        break;
      }
    /*case 'L':
      {                   
        //led_r = data[1];  
        //led_y = data[2];
        //led_g = data[3];
        //led_b = data[4];
        if (debug_mode) {
          Serial.print("Red: ");
          Serial.print(led_r);
          Serial.print(" Yellow: ");
          Serial.print(led_y);
          Serial.print(" Green: ");
          Serial.print(led_g);
          Serial.print(" Blue: ");
          Serial.println(led_b);
        }
        break;
      }*/
    default:
      Serial.println("Unknown message type");
      cmd_triggered=false;
      break;
  }
  last_message_time=current_time;
  if(serial_connection_established==false){
  serial_connection_established=true;
  systemFault(false,"","", NONE, NONE, ON);
  }
}

// Shutdown all eletronic systems in case of E-stop.
void stop_all() {
  act_left.stop();
  act_right.stop();
  act_bucket.stop();
  motor_left.stop();
  motor_right.stop();
}

// Change led state, and blink led if requested
void updateLed(OutPin &led, const LedState &state, int interval=250) {
  if(state==NONE) return;
    if (state == OFF || state == ON) {
        led.write(state == ON ? 1 : 0);
    } else if (state == BLINK) {
        if (current_time - led_blink_time >= interval) {
            led_blink_time = current_time;
            led.write(!led.read());
        }
    }
}


/*
RED: critical system fault(FULL ESTOP)

YELLOW: error occored(Non-critical)
YELLOW BLINKING: error occored(Attempting to correct Non-critical Error)

GREEN: All systems running
GREEN BLINKING: System startup process

BLUE: Communication connected and functioning like normal
BLUE BLINKING: System started, iding, waiting for communication data stream to start
*/

// Handles rotbot critical faults(E-STOP), errors, and led ON/OFF and BLINKING
void systemFault(bool criticalError,String fault_msg, String error_msg, LedState y, LedState g, LedState b) {
  // A criticalError means full robot shutdown, also this logs the shutdown error.
  if(criticalError==true) emergency_stop=true;
  if(fault_msg!="") system_fault_msg=fault_msg;

  // Once a shutdown state is entered, print log, set red led, and shutdown main loop from running until arduino is reset
  if(emergency_stop==true){
    while(true){
    stop_all();
    Serial.println("Critical System Fault (E-STOPPED): " + system_fault_msg + " -Reset arduino to continue.");
    if(error_leds_enabled==true) {
      ledr_pin.write(1);
      ledy_pin.write(0);
      ledg_pin.write(0);
      ledb_pin.write(0);
    }
    }
  }
  // For less critical errros print error once, and turn on error led
  if(error_msg!=""){
    Serial.println("System Error: " + error_msg);
    if(y==NONE) led_y = ON;
  }
  // Update led states
  if(y!=NONE) led_y = y;
  if(g!=NONE) led_g = g;
  if(b!=NONE) led_b = b;
  if(error_leds_enabled==true){
    updateLed(ledg_pin, led_g);
    updateLed(ledy_pin, led_y);
    updateLed(ledb_pin, led_b);
  }
}

void calibrateIMU() {
  // Start IMU sesnor and make sure it can be connected to
    IMU.initialize();
    if (!IMU.testConnection()) return;
    // Read the IMU sensor for specified number of loops and log data
    int16_t gx, gy, gz;
    float IMU_sample_sum = 0;
    for (int i = 0; i < IMU_offset_bias_samples; i++) {
        IMU.getRotation(&gx, &gy, &gz);
        float rate_raw = gz / 131.0;  
        IMU_sample_sum += rate_raw;
        delay(20); 
    }
    // Use logged data to figure out average IMU drift(IMU_offset_bias) factor
    IMU_offset_bias=IMU_sample_sum/IMU_offset_bias_samples;
}

void calibrateIMUAngle() {
  int16_t ax, ay, az;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;
  for (int i = 0; i < IMU_angle_bias_samples; i++) {
      IMU.getAcceleration(&ax, &ay, &az);
      ax_sum += ax;
      ay_sum += ay;
      az_sum += az;
      delay(10);
  }
  float ax_avg = (float)ax_sum / IMU_angle_bias_samples;
  float ay_avg = (float)ay_sum / IMU_angle_bias_samples;
  float az_avg = (float)az_sum / IMU_angle_bias_samples;
  float IMU_pitch_angle_bias = atan2(-ax_avg, sqrt(ay_avg * ay_avg + az_avg * az_avg));
  float IMU_roll_angle_bias = atan2(ay_avg, az_avg);
  IMU_yaw_scale = cos(IMU_pitch_angle_bias) * cos(IMU_roll_angle_bias);
}

void updateIMUData(bool useHomeBias){ 
  int16_t gx, gy, gz; 
  IMU.getRotation(&gx, &gy, &gz); 
  float gz_rate = gz / 131.0; 
  float dt = (current_time - last_IMU_time) / 1000.0; 
  last_IMU_time = current_time; 
  float alpha = dt / (IMU_filter_constant + dt); 
  IMU_filter_rate = alpha * gz_rate + (1 - alpha) * IMU_filter_rate; 
  if(useHomeBias) IMU_raw_home_bias = IMU_rate; 
  float gz_world = IMU_filter_rate * IMU_yaw_scale;
  IMU_rate += (gz_world - IMU_offset_bias) * dt; 
  IMU_yaw = (IMU_rate - IMU_raw_home_bias) + IMU_local_home_bias;
  if(IMU_yaw > 180) IMU_yaw -= 360;
  if(IMU_yaw < -180) IMU_yaw += 360;
}

void sendSerialFeedback(char command, uint8_t* data, size_t dataLen) {
  const uint8_t startByte = 0x02; // Start byte
  const uint8_t endByte   = 0x03; // End Byte
  uint8_t buf[64];   // max size for USB CDC packet
  size_t idx = 0;
  buf[idx++] = startByte; // Create data buffer and push all required data to buffer
  buf[idx++] = dataLen + 1; // length (command + data)
  buf[idx++] = command;
  for (size_t i = 0; i < dataLen; i++) { 
    buf[idx++] = data[i];
  }
  buf[idx++] = endByte;
  Serial.write(buf, idx);   // Write data to serial
}
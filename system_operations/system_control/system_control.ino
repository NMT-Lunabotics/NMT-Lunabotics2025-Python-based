#include "helpers.hpp"
#define MAIN_ROBOT 1

//--------------- MAIN ROBOT SETTINGS ---------------
#if MAIN_ROBOT==1

// List of components flags to enable/disable for testing
#define ERROR_LEDS_ENABLED           1
#define MOTORS_ENABLED               0
#define BUCKET_ACTUATOR_ENABLED      0
#define ARM_ACTUATORS_ENABLED        1
#define SERVO_MOTOR_ENABLED          0
#define IMU_SENSOR_ENABLED           0
#define IBUS_RECIVER_ENABLED         1

// List of faults to disable
#define SERIAL_COMM_TIMEOUT_FAULT    1
#define COMPONENT_TIMEOUT_FAULTS     1

// Debug mode flags
#define DEBUG_MODE                   0
#define SENSOR_OUTPUT                0  // 1: IMU, 2: IBUS, 3: IBUS raw

#else
//--------------- NUC TEST ROBOT SETTINGS ---------------
#define ERROR_LEDS_ENABLED           0
#define MOTORS_ENABLED               1
#define BUCKET_ACTUATOR_ENABLED      0
#define ARM_ACTUATORS_ENABLED        0
#define SERVO_MOTOR_ENABLED          1
#define IMU_SENSOR_ENABLED           0
#define IBUS_RECIVER_ENABLED         0
#define SERIAL_COMM_TIMEOUT_FAULT    1
#define COMPONENT_TIMEOUT_FAULTS     0
#define DEBUG_MODE                   0
#define SENSOR_OUTPUT                0  
#endif

//--------------- Setup used classes ---------------
#if IMU_SENSOR_ENABLED
#if MAIN_ROBOT==1
#define IMU_MANUAL_SCALER 1
#else
#define IMU_MANUAL_SCALER 0.6
#endif
MPU6050 IMU; 
#endif
#if IBUS_RECIVER_ENABLED
IBusReader ibus(Serial1);
#endif
#if SERVO_MOTOR_ENABLED
  SimpleServo servo(11);
#endif

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
#define AL_POT_MIN 47//49
#define AL_POT_MAX 893
#define AR_POT_MIN 0//2
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

// Offset factor to max sure actuators start/stop in same spots
int aL_offset = 0; //mm
int aR_offset = 3; //mm

// Actuator speed and position variables
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

//--------------- MOTORS ---------------
#if MAIN_ROBOT==1
const int DACL1_PIN = 2;
const int DACL2_PIN = 3;
const int DACR1_PIN = 4;
const int DACR2_PIN = 5;
const int EN_PIN = 32;  // Common for both motors
#endif

// Max allowed motor velocity (rpm)
int motor_max_vel = 30; 

// Motor speed and target rotation and radius variables
int mL_speed = 0;
int mR_speed = 0;

int mLR_rotation_speed = 0;
int mLR_rotation = 0;
int mLR_arc_radius = 0;
float robot_width = 7.276186; //m

#if MAIN_ROBOT==1
#define mL_speed_scale 1
#define mR_speed_scale 1
#else
#define mL_speed_scale 1
#define mR_speed_scale 1
#endif

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
#if IMU_SENSOR_ENABLED
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
#endif

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
String system_fault_msg = "";
bool system_started = false;

// Motor and actuator timeout variables, shutdown motors and actuators if a message is not reviced within timeout period
unsigned long last_actuator_cmd_time = 0; 
unsigned long last_motor_cmd_time = 0;
int motor_timeout = 2000;
int actuator_timeout = 2000;

// Serial and state
bool serial_connection_established=false;
bool RC_connection_established=false;
bool receiving_message = false;
bool at_bucket_min = false;
bool at_bucket_max = false;
bool dual_actuator_correct = false;
int serial_index = 0;
int expected_length = -1;
const int MY_SERIAL_BUFFER_SIZE = 128;
byte serial_buffer[MY_SERIAL_BUFFER_SIZE];

// Set up PID controllers velocity gain
float vel_gain = 2.5;

#if ARM_ACTUATORS_ENABLED
// Set up arm actuators and PID controllers
PID pidL(2.2, 0.0022, 0.34, 2.0);
PID pidR(1.85, 0.0018, 0.31, 1.7);

PWM_Driver left_driver(DRV12_PWM_PIN, DRV12_DIR1_PIN, DRV12_DIR2_PIN, false);
Actuator act_left(left_driver, pidL, POTL_PIN, AL_POT_MIN, AL_POT_MAX, ALR_STROKE, act_max_vel,20,150);
PWM_Driver right_driver(DRV11_PWM_PIN, DRV11_DIR1_PIN, DRV11_DIR2_PIN, false);
Actuator act_right(right_driver, pidR, POTR_PIN, AR_POT_MIN, AR_POT_MAX, ALR_STROKE, act_max_vel,20,150);
#endif 

#if BUCKET_ACTUATOR_ENABLED
// Set up bucket actuator and PID controller
PID pidB(3.0, 0.001, 0.4);

PWM_Driver bucket_driver(DRV21_PWM_PIN, DRV21_DIR1_PIN, DRV21_DIR2_PIN, true);
Actuator act_bucket(bucket_driver, pidB, POTB_PIN, AB_POT_MIN, AB_POT_MAX, AB_STROKE, act_max_vel, bucket_min, bucket_max);
#endif

#if MOTORS_ENABLED
// Set up motors
#if MAIN_ROBOT==1
OutPin motor_left_dac1(DACL1_PIN);
OutPin motor_left_dac2(DACL2_PIN);
OutPin motor_right_dac1(DACR1_PIN);
OutPin motor_right_dac2(DACR2_PIN);
OutPin motor_enable(EN_PIN);
Motor motor_left(motor_left_dac1, motor_left_dac2, motor_enable, motor_max_vel, false);
Motor motor_right(motor_right_dac1, motor_right_dac2, motor_enable, motor_max_vel, true);
#else
SimpleMotor simpleMotorRight(3, 4, 9, motor_max_vel);
SimpleMotor simpleMotorLeft(6, 7, 8, motor_max_vel);
#endif
#endif

#if ERROR_LEDS_ENABLED
// Set up LEDs
OutPin ledr_pin(LEDR_PIN);
OutPin ledy_pin(LEDY_PIN);
OutPin ledg_pin(LEDG_PIN);
OutPin ledb_pin(LEDB_PIN);
#endif

// void processMessage(byte* data, int length);
void stop_all();
void processSerialBuffer();
void systemFault(bool criticalError = false,String fault_msg="", String error_msg="", LedState y =NONE, LedState g =NONE, LedState b=NONE);
void processMessage(byte *data, int length);
#if IMU_SENSOR_ENABLED
void calibrateIMU();
void calibrateIMUAngle();
void updateIMUData(bool useHomeBias=false);
#endif
void sendSerialFeedback(char command, uint8_t* data, size_t dataLen);

void setup() {
  delay(5);
  Serial.begin(115200);
  #if MAIN_ROBOT==0
    #if MOTORS_ENABLED
      simpleMotorLeft.begin();
      simpleMotorRight.begin();
    #endif
  #endif
  #if IBUS_RECIVER_ENABLED
  ibus.begin(115200);
  #endif
  Serial.flush();
  // Set default led status
  systemFault(false,"","", NONE, BLINK, BLINK);
  #if IMU_SENSOR_ENABLED
  // Calibrate the IMU sensors drift factor
  calibrateIMU();
  calibrateIMUAngle();
  #endif
  for (int i = 0; i < 10; i++) {
    #if ARM_ACTUATORS_ENABLED
    aL_pos = act_left.update_pos();
    aR_pos = act_right.update_pos();
    #endif
    #if BUCKET_ACTUATOR_ENABLED
    aB_pos = act_bucket.update_pos();
    #endif
  }
  #if SERVO_MOTOR_ENABLED
    servo.attach();
  #endif
  Serial.println("Arduino system_control.ino started.");
}

void loop() {
  current_time = millis();
  processSerialBuffer();
  #if IBUS_RECIVER_ENABLED
  // Read serial and process messages while being Non-blocking
  if (ibus.update()) {
    #if SENSOR_OUTPUT == 3
    int16_t* joy = ibus.getJoystick(true);
    Serial.print("Raw RC controller inputs: ");
    for (int i = 0; i < 12; i++) {  
      Serial.print(joy[i]);
      Serial.print(" ");
    }
    Serial.println("");
  #elif SENSOR_OUTPUT == 2
    int16_t* joy = ibus.getJoystick();
    Serial.print("RC controller inputs: ");
    for (int i = 0; i < 5; i++) {  
      Serial.print(joy[i]);
      Serial.print(" ");
    }
    Serial.println("");
  #else
    int16_t* joy = ibus.getJoystick();
    if(joy[4]==0) {
      mL_speed=0;
      mR_speed=0;
      aL_speed=0;
      aR_speed=0;
      aB_speed=0;
    }
    else{
      int16_t throttle = -joy[0]; 
      int16_t steering = joy[1];
      mR_speed = constrain(throttle - steering, -30, 30);
      mL_speed = constrain(throttle + steering, -30, 30);
      aLR_tgt = -1;
      aB_tgt = -1;
      aL_speed = -joy[3];
      aR_speed = aL_speed;
      aB_speed = joy[2];
      if(RC_connection_established==false){
        RC_connection_established=true;
        systemFault(false,"","", NONE, NONE, ON);
      }
    }
  #endif
  } 
  #endif
  #if SERIAL_COMM_TIMEOUT_FAULT
  if (current_time - last_message_time > estop_timeout && serial_connection_established==true) systemFault(true,"Serial communication timeout.","", NONE, NONE, NONE);
  #endif
  // Update actuator saved positions
  if (current_time - last_update_actuator_time >= 1000 / update_actuator_feedback) {
    last_update_actuator_time = current_time;
    #if ARM_ACTUATORS_ENABLED
    aL_pos = act_left.update_pos();
    aR_pos = act_right.update_pos();
    #endif
    #if BUCKET_ACTUATOR_ENABLED
    aB_pos = act_bucket.update_pos();
    #endif
  }

  if (current_time - last_update_time >= 1000 / update_rate) {
    last_update_time = current_time;

    // Ensure bucket is in bounds
    #if BUCKET_ACTUATOR_ENABLED
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
    #endif
    // Correct dual actuator misalignment
    #if ARM_ACTUATORS_ENABLED
    float lr_err = abs(aL_pos - aR_pos);
    if (lr_err >= act_fix_err && lr_err < act_max_err) {
      stop_all();
      float prev_err = lr_err;
      while (lr_err >= 0.5 * act_fix_err) {
        #if BUCKET_ACTUATOR_ENABLED
          act_bucket.stop();
        #endif
        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        float factor = (aL_pos - aR_pos) * vel_gain;

        Serial.print(aL_pos)
        Serial.print("")
        Serial.print(aR_pos)
        Serial.println("")
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
    } //else if (lr_err >= act_max_err) systemFault(true,"Actuator relative error too large: " + String(aL_pos) + " " + String(aR_pos),"", NONE, NONE, NONE);
  #endif
    // Run motors and actuators
    if (!emergency_stop) {
      #if COMPONENT_TIMEOUT_FAULTS
      // Timeout motors and actuators if a command isn't recived within the timeout period
      if(serial_connection_established==true){
        if(current_time-last_motor_cmd_time>=motor_timeout&&(mR_speed!=0||mL_speed!=0||mLR_rotation_speed!=0||mLR_rotation!=0)){
          mR_speed=0;
          mL_speed=0;
          mLR_rotation_speed=0;
          mLR_rotation=0;
          systemFault(false,"","Motors timed out, no motor command received within "+ String(motor_timeout/1000)+"s", NONE, NONE, NONE);
        }
        if(current_time-last_actuator_cmd_time>=actuator_timeout && (aL_speed!=0 || aR_speed !=0 || aB_speed!=0 || aLR_tgt != -1 || aB_tgt != -1 )){
          aLR_tgt = -1;
          aB_tgt = -1;
          aL_speed = 0;
          aR_speed = 0;
          aB_speed = 0;
          systemFault(false,"","Actuators timed out, no actuator command received within "+ String(actuator_timeout/1000)+"s", NONE, NONE, NONE);
        }
      }
      #endif
      #if ARM_ACTUATORS_ENABLED
      // Update actuator positions.
      if (aLR_tgt >= 0) {
        act_left.tgt_ctrl(aLR_tgt);
        act_right.tgt_ctrl(aLR_tgt);
      } else {
        float factor = (aL_pos - aR_pos) * vel_gain;
        act_left.vel_ctrl(aL_speed - factor);
        act_right.vel_ctrl(aR_speed + factor);
      }
      #endif
      #if BUCKET_ACTUATOR_ENABLED
      if (aB_tgt >= 0)
        act_bucket.tgt_ctrl(aB_tgt);
      else if ((aB_speed > 0 && aB_pos < bucket_max) || (aB_speed < 0 && aB_pos > bucket_min))
        act_bucket.vel_ctrl(aB_speed);
      else
        act_bucket.stop();
      #endif
      #if MOTORS_ENABLED
        // Update motor speeds
        if(mLR_rotation_speed!=0 && mLR_rotation!=0){
          if(mLR_arc_radius!=0){
            float mL_velocity=mLR_rotation_speed*(1-robot_width/mLR_arc_radius);
            float mR_velocity=mLR_rotation_speed*(1+robot_width/mLR_arc_radius);
            #if MAIN_ROBOT==1
            motor_left.motor_ctrl(mL_velocity);
            motor_right.motor_ctrl(mR_velocity);
            #else
              simpleMotorLeft.setSpeed(mL_velocity);   
              simpleMotorRight.setSpeed(mR_velocity);
            #endif
          }
          else if(mLR_rotation>0){
            #if MAIN_ROBOT==1
              motor_left.motor_ctrl(-mLR_rotation_speed);
              motor_right.motor_ctrl(mLR_rotation_speed);
            #else
              simpleMotorLeft.setSpeed(-mLR_rotation_speed);   
              simpleMotorRight.setSpeed(mLR_rotation_speed);
            #endif
          }
          else if(mLR_rotation<0){
            #if MAIN_ROBOT==1
              motor_left.motor_ctrl(mLR_rotation_speed);
              motor_right.motor_ctrl(-mLR_rotation_speed);
            #else
              simpleMotorLeft.setSpeed(mLR_rotation_speed);   
              simpleMotorRight.setSpeed(-mLR_rotation_speed);
            #endif
          }
          #if IMU_SENSOR_ENABLED
          if((mLR_rotation<0&&IMU_yaw<=mLR_rotation)||(mLR_rotation>0&&IMU_yaw>=mLR_rotation)){
            #if MAIN_ROBOT==1
              motor_left.stop();
              motor_right.stop();
            #else
              simpleMotorLeft.stop();
              simpleMotorRight.stop();
            #endif
            mLR_rotation_speed=0;
            if(reset_IMU_local_home==false) IMU_local_home_bias=IMU_yaw;
            update_IMU_raw_home=true;
            #if DEBUG_MODE
              Serial.print("Yaw: ");
              Serial.println(IMU_yaw,6);
            #endif
            updateIMUData(true);
            sendSerialFeedback('R', nullptr, 0);
          } 
          #endif
        }
        else{
          #if MAIN_ROBOT==1
            motor_left.motor_ctrl(mL_speed*mL_speed_scale);
            motor_right.motor_ctrl(mR_speed*mR_speed_scale);
          #else
            simpleMotorLeft.setSpeed(mL_speed*mL_speed_scale);   
            simpleMotorRight.setSpeed(mR_speed*mR_speed_scale);
          #endif
        }
      #endif
    } 
    else stop_all();
  // Sync feedback loop timer
  if (current_time - last_feedback_time >= 1000 / feedback_rate) last_feedback_time = current_time;
  #if ARM_ACTUATORS_ENABLED || BUCKET_ACTUATOR_ENABLED
  // Reset Actuator PID's to prevent windup.
  if (current_time - last_reset_int_time >= 1000 / reset_int_rate) {
    last_reset_int_time = current_time;
    #if ARM_ACTUATORS_ENABLED
    act_left.resetPIDIntegral();
    act_right.resetPIDIntegral();
    #endif
    #if BUCKET_ACTUATOR_ENABLED
    act_bucket.resetPIDIntegral();
    #endif
  }
  #endif
  
  // System fully started turn on green led
  if(system_started==false){
    system_started=true;
    systemFault(false,"","", NONE, ON, NONE);
  }
    // Run fault function every loop to update leds, and handle critical errors
  systemFault(false, "","", NONE, NONE, NONE);
  #if IMU_SENSOR_ENABLED
  if(update_IMU_raw_home==true) updateIMUData(true);
  else updateIMUData(false);
  #endif
  #if SENSOR_OUTPUT == 1
    Serial.print("Yaw: ");
    Serial.println(IMU_yaw,6);
  #endif
}
}

// Read serial communication from autonomy computer and set system variables to output.
void processMessage(byte *data, int length) {
  current_time = millis();
  char type = data[0];
  #if DEBUG_MODE
    Serial.print("Received message of type: ");
    Serial.println(type);
    Serial.print("Length: ");
    Serial.println(length);
  #endif
  bool cmd_triggered = true;
  switch (type) {
    // Chnage actuator speeds/positions
    case 'A':
      {      
        #if BUCKET_ACTUATOR_ENABLED || ARM_ACTUATOR_ENABLED       
        // (A, actuators) Recvived Arm and Bucket actuator speeds and positions.
        aLR_tgt = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
        aB_tgt = (int16_t)((data[3] << 8) | data[4]);
        aL_speed = -(int8_t)data[5];
        aR_speed = aL_speed;
        aB_speed = -(int8_t)data[6];
        #if DEBUG_MODE
          Serial.print("Arm Position: ");
          Serial.println(aLR_tgt);
          Serial.print("Bucket Position: ");
          Serial.println(aB_tgt);
          Serial.print("Arm Velocity: ");
          Serial.println(aL_speed);
          Serial.print("Bucket Velocity: ");
          Serial.println(aB_speed);
        #endif
        last_actuator_cmd_time=current_time;
        #endif
        break;
      }
    // Change motor speeds
    case 'M':
      {    
        #if MOTORS_ENABLED
        // (M, motors) Recvived left and right motor speeds
        mR_speed = (int8_t)data[1];  
        mL_speed = (int8_t)data[2];
        #if DEBUG_MODE
          Serial.print("Left Speed: ");
          Serial.println(mL_speed);
          Serial.print("Right Speed: ");
          Serial.println(mR_speed);
        #endif
        last_motor_cmd_time=current_time;
        #endif
        break;
      }
    // Rotate robot
    case 'R': 
      { 
        #if IMU_SENSOR_ENABLED && MOTORS_ENABLED
        // (R, rotate) Recvives a rotation speed, angle, radius, and home reset, to allow arc or stationary turns       
        mLR_rotation_speed = (int8_t)data[1]; 
        if(mLR_rotation_speed>0&&mLR_rotation_speed>motor_max_vel) mLR_rotation_speed=motor_max_vel;
        else if(mLR_rotation_speed<0&&mLR_rotation_speed<-motor_max_vel) mLR_rotation_speed=-motor_max_vel;
        mLR_rotation = -(int8_t)data[2];
        mLR_arc_radius = (int8_t)data[3];
        reset_IMU_local_home = (int8_t)data[4];
        update_IMU_raw_home=false;
        if(mLR_arc_radius==0&&mLR_rotation_speed==0&&mLR_rotation==0&&reset_IMU_local_home==true) IMU_local_home_bias=IMU_yaw;
        #if DEBUG_MODE
          Serial.print("Motor rotation Speed: ");
          Serial.println(mLR_rotation_speed);
          Serial.print("Motor rotation arc radius: ");
          Serial.println(mLR_arc_radius);
          Serial.print("Motor rotation angle: ");
          Serial.println(mLR_rotation);
        #endif
        last_motor_cmd_time=current_time;
        #endif
        break;
      }
    // Servo state
    case 'S':    
      { 
        #if SERVO_MOTOR_ENABLED
        // (S, servo motor) Recvived servo latch state
        //servo_state = data[1];  
        servo.write((int8_t)data[1]);
        #if DEBUG_MODE
          Serial.print("Servo State: ");
          Serial.println(servo_state);
        #endif
        #endif
        break;
      }
    // E-STOP REBOOT
    case 'B':
    // USE E-STOP RESETS WITH EXTREAM CAUTION, THIS CAN BE DANGRUS TO THE WHOLE ROBOT.
    // This command should only be sent by a user whom knows the dangers and system workings, so (no automated button) and (no autonomy resets) and if a controller is used the (reset should require a combanation of 3+ buttons to do the reset)
      { 
        emergency_stop=false;
        break;
      }
    /*case 'L':
      {                   
        //led_r = data[1];  
        //led_y = data[2];
        //led_g = data[3];
        //led_b = data[4];
        #if DEBUG_MODE
          Serial.print("Red: ");
          Serial.print(led_r);
          Serial.print(" Yellow: ");
          Serial.print(led_y);
          Serial.print(" Green: ");
          Serial.print(led_g);
          Serial.print(" Blue: ");
          Serial.println(led_b);
        #endif
        break;
      }*/
    default:
      Serial.println("Unknown message type");
      cmd_triggered=false;
      break;
  }
  last_message_time=current_time;
  if(serial_connection_established==false&&cmd_triggered==true){
  serial_connection_established=true;
  systemFault(false,"","", NONE, NONE, ON);
  }
}

// Shutdown all eletronic systems in case of E-stop.
void stop_all() {
  #if ARM_ACTUATORS_ENABLED
  act_left.stop();
  act_right.stop();
  #endif
  #if BUCKET_ACTUATOR_ENABLED
  act_bucket.stop();
  motor_left.stop();
  #endif
  #if MOTORS_ENABLED
    #if MAIN_ROBOT==1
      motor_right.stop();
      motor_left.stop();
    #else
      simpleMotorLeft.stop();
      simpleMotorRight.stop();
    #endif
  #endif
}

// Change led state, and blink led if requested
#if ERROR_LEDS_ENABLED
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
#endif

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
    // If the system is E-STOPPED no normal commands are able to take place, however we keep listening to serial for a E-STOP reset command
    processSerialBuffer();
    if(emergency_stop==false) break;
    Serial.println("Critical System Fault (E-STOPPED): " + system_fault_msg + " -Reset arduino to continue.");
    #if ERROR_LEDS_ENABLED
      ledr_pin.write(1);
      ledy_pin.write(0);
      ledg_pin.write(0);
      ledb_pin.write(0);
    #endif
    }
    return;
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
  #if ERROR_LEDS_ENABLED
    updateLed(ledg_pin, led_g);
    updateLed(ledy_pin, led_y);
    updateLed(ledb_pin, led_b);
  #endif
}

#if IMU_SENSOR_ENABLED
// This logs the IMU rotation data over a timer of samples to find the sensors dift
void calibrateIMU() {
  // Start IMU sesnor and make sure it can be connected to
    IMU.initialize();
    if (!IMU.testConnection()) {
      systemFault(true,"IMU sensor not detected","", NONE, NONE, NONE);
      return;
    }
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

// This logs the IMU sensor over a number of sample to get an mean value, and then uses some math to figure out the angle of the IMU sensor
void calibrateIMUAngle() {
  // Record accelerometer and find average acceloration in each direction
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
  // Use trig to figure out the current pich and roll of the current sesnor
  float IMU_pitch_angle_bias = atan2(-ax_avg, sqrt(ay_avg * ay_avg + az_avg * az_avg));
  float IMU_roll_angle_bias = atan2(ay_avg, az_avg);
  // Apply roll and pitch as scale factor to correctly orientate the sensor at any orientation
  IMU_yaw_scale = cos(IMU_pitch_angle_bias) * cos(IMU_roll_angle_bias);
  #if SENSOR_OUTPUT == 1
  Serial.print("IMU_pitch_angle_bias: ");
  Serial.print(IMU_pitch_angle_bias);
  Serial.print(" IMU_roll_angle_bias: ");
  Serial.println(IMU_roll_angle_bias);
  Serial.print(" IMU_yaw_scale: ");
  Serial.println(IMU_yaw_scale);
  #endif
}

// This updates the IMU sesnor data and applys all of the IMU biases to figure out the best estimate for the sensor rotation
void updateIMUData(bool useHomeBias){ 
  // Record Gyroscope rotation data, and convert gz the horizional plan to degrees
  int16_t gx, gy, gz; 
  IMU.getRotation(&gx, &gy, &gz); 
  float rate_raw = gz / 131.0; 
  // Find dt constant for differentiation of rotation data
  float dt = (current_time - last_IMU_time) / 1000.0; 
  // Calulate and apply a low path filter to rotation data
  float alpha = dt / (IMU_filter_constant + dt); 
  IMU_filter_rate = alpha * rate_raw + (1 - alpha) * IMU_filter_rate;  
  // Update and apply any biases and update everything for next cycle
  if(useHomeBias==true) IMU_raw_home_bias=IMU_rate; 
  IMU_rate += (IMU_filter_rate - IMU_offset_bias) * dt; 
  IMU_yaw = -(((IMU_rate - IMU_raw_home_bias) + IMU_local_home_bias)/IMU_yaw_scale)*IMU_MANUAL_SCALER;
  last_IMU_time = current_time; 
}
#endif

// This uses the same format as Serial commands sent to the arduino but it sends them back to the jetson for live serial update
void sendSerialFeedback(char command, uint8_t* data, size_t dataLen) {
  // Start and end byte for feedback command
  const uint8_t startByte = 0x02;
  const uint8_t endByte   = 0x03; 
  // Create buffer with serial data
  uint8_t buf[64];   
  size_t idx = 0;
  // Add start byte, end byte, length byte, command byte, and data bytes into buffer,
  buf[idx++] = startByte; 
  buf[idx++] = dataLen + 1;
  buf[idx++] = command;
  for (size_t i = 0; i < dataLen; i++) { 
    buf[idx++] = data[i];
  }
  buf[idx++] = endByte;
  // Write serial buffer to Serial
  Serial.write(buf, idx);   
}

// This processes the serial buffer and if the buffer passes it's byte checks, it's sent to processMessage() to update data.
void processSerialBuffer() {
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
            if (expected_length <= 0 || expected_length > MY_SERIAL_BUFFER_SIZE) {
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
            } else if (serial_index >= MY_SERIAL_BUFFER_SIZE) {
                // overflow protection
                receiving_message = false;
            }
        }
    }
}
}
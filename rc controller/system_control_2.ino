#include "helpers.hpp"
// #include <Servo.h>

// Debug mode flag
bool debug_mode = false;

//////// ACTUATORS ////////
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
// TODO verify
#define POTL_PIN A1
#define POTR_PIN A0
#define POTB_PIN A3

// Actuator info
// Actuator stroke in mm
#define ALR_STROKE 191
#define AB_STROKE 140

// Actuator Calibration
#define AL_POT_MIN 47
#define AL_POT_MAX 893
#define AR_POT_MIN 0
#define AR_POT_MAX 840
#define AB_POT_MIN 30
#define AB_POT_MAX 782

float bucket_min = 20; // mm
float bucket_max = 110; // mm
float bucket_absolute_max = 115; // mm
float act_end_tolerance = 1; // mm

float act_max_vel = 25; //mm/s
float act_fix_err = 3.0; // mm
float act_max_err = 5.0; // mm

// Actuator targets
int aL_speed = 0;
int aR_speed = 0;
int aB_speed = 0;

float aL_pos = 0;
float aR_pos = 0;
float aB_pos = 0;

int aLR_tgt = -1;
int aB_tgt = -1;

//////// MOTORS ////////
const int DACL1_PIN = 2;
const int DACL2_PIN = 3;
const int DACR1_PIN = 4;
const int DACR2_PIN = 5;
const int EN_PIN = 32; // Common for both motors

int motor_max_vel = 30; //rpm

// Motor speeds
int mL_speed = 0;
int mR_speed = 0;

//TODO implement servo logic
// #define SERVO_PIN 22

// Servo
bool servo_state = false;

// LED's
// TODO implement
#define LEDR_PIN 24
#define LEDY_PIN 26
#define LEDG_PIN 28
#define LEDB_PIN 30

bool led_r = false;
bool led_y = false;
bool led_g = false;
bool led_b = false;

// Timing
int update_rate = 200; //hz
int update_actuator_feedback = 1000; //hz
int feedback_rate = 10; //hz
int reset_int_rate = 10; //hz
unsigned long last_update_time = 0;
unsigned long last_update_actuator_time = 0;
unsigned long last_feedback_time = 0;
unsigned long last_reset_int_time = 0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;
bool emergency_stop = false;
bool doomsday = false;

// Serial and state
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

// Set up servo
// Servo servo;

void processMessage(byte* data, int length);
void stop_all();
void fault();

void setup(){
    Serial.begin(115200);
    Serial.flush();
    // Wire.begin();
    // servo.attach(SERVO_PIN); //THIS LINE BREAKS THE MOTORS
    // servo.write(0);
    ledr_pin.write(1);
    ledy_pin.write(1);
    ledg_pin.write(0);
    ledb_pin.write(0);

    for (int i = 0; i < 10; i++) {
        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        aB_pos = act_bucket.update_pos();
    }
}

void loop() {
    current_time = millis();
    // if (Serial.available() > 0) {
    //     if (Serial.read() == 0x02) { // Start byte
    //         while (Serial.available() < 1) {} // Wait for type and length bytes
    //         int length = Serial.read();
    //         while (Serial.available() < length + 1) {} // Wait for the entire message
    //         byte data[length];
    //         Serial.readBytes(data, length);
    //         while (Serial.available() < 1) {} // Wait for end byte
    //         if (Serial.read() == 0x03) { // End byte
    //             emergency_stop = false;
    //             last_message_time = millis();
    //             processMessage(data, length);
    //         } else {
    //             Serial.println("End byte not found");
    //             emergency_stop = true;
    //         }
    //     }
    // }

        // --- Non-blocking serial receive ---
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
            } else if (serial_index < SERIAL_BUFFER_SIZE) {
                serial_buffer[serial_index++] = b;
                if (serial_index == expected_length + 1) { // +1 for end byte
                    if (serial_buffer[serial_index - 1] == 0x03) {
                        emergency_stop = false;
                        last_message_time = millis();
                        processMessage(serial_buffer, expected_length);
                    } else {
                        Serial.println("End byte not found");
                        emergency_stop = true;
                    }
                    receiving_message = false;
                }
            }
        }
    }

    if (current_time - last_message_time > estop_timeout) {
        emergency_stop = true;
    }

    if (current_time - last_update_actuator_time >= 1000 / update_actuator_feedback) {
        last_update_actuator_time = current_time;

        aL_pos = act_left.update_pos();
        aR_pos = act_right.update_pos();
        aB_pos = act_bucket.update_pos();
    }

    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;
        
        // Ensure bucket is in bounds
        // if (aB_pos > bucket_absolute_max) {
        //     fault("Bucket position out of bounds: " + String(aB_pos));
        // }
        // if (aB_pos < bucket_min || aB_pos > bucket_max) {
        //     stop_all();
        //     while (aB_pos < bucket_min) {
        //         act_bucket.vel_ctrl(5);
        //         aB_pos = act_bucket.update_pos();
        //         delay(5);
        //         Serial.println("Bucket past minimum. Fixing");
        //     }
        //     while (aB_pos > bucket_max) {
        //         act_bucket.vel_ctrl(-5);
        //         aB_pos = act_bucket.update_pos();
        //         delay(5);
        //         Serial.println("Bucket past maximum. Fixing");
        //     }
        //     act_bucket.stop();
        // }

        // Correct dual actuator misalignment
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
                if (lr_err > prev_err) {
                    fault("Actuator positions are diverging.");
                }
                Serial.println("Fixing actuators: ");
                ledy_pin.write(1);
            }
            act_left.stop();
            act_right.stop();
            ledy_pin.write(0);
        } else if (lr_err >= act_max_err) {
            fault("Actuator relative error too large: " + String(aL_pos) + " " + String(aR_pos));
            // Serial.print("wtf");
        }

        if (!emergency_stop) {
            led_r = false;
            led_g = true;

            if (aLR_tgt >= 0) {
                act_left.tgt_ctrl(aLR_tgt);
                act_right.tgt_ctrl(aLR_tgt);
            } else {
                float factor = (aL_pos - aR_pos) * vel_gain;
                act_left.vel_ctrl(aL_speed - factor);
                act_right.vel_ctrl(aR_speed + factor);
            }

            if (aB_tgt >= 0) {
                act_bucket.tgt_ctrl(aB_tgt);
            } else if((aB_speed >0 && aB_pos < bucket_max) || (aB_speed <0 && aB_pos > bucket_min)){
                act_bucket.vel_ctrl(aB_speed);
            }else{
                act_bucket.stop();
            }
            
        
            

            motor_left.motor_ctrl(mL_speed);
            motor_right.motor_ctrl(mR_speed);
        } else {
            led_r = true;
            led_g = false;
            stop_all();
        }

        ledr_pin.write(led_r);
        ledy_pin.write(led_y);
        ledg_pin.write(led_g);
        ledb_pin.write(led_b);
    }

    if (current_time - last_feedback_time >= 1000 / feedback_rate) {
        last_feedback_time = current_time;

        if (emergency_stop) {
            Serial.println("Estopped");
        }
        Serial.println("<F," + String(aL_pos) + "," + String(aR_pos) + "," + String(aB_pos)
            + "," + String(aL_speed) + "," + String(aR_speed) + ',' + String(aLR_tgt)
            + "," + String(mL_speed) + "," + String(mR_speed) + ">");
    }

    if (current_time - last_reset_int_time >= 1000 / reset_int_rate) {
        last_reset_int_time = current_time;

        act_left.resetPIDIntegral();
        act_right.resetPIDIntegral();
        act_bucket.resetPIDIntegral();
    }
}

void processMessage(byte* data, int length) {
    char type = data[0];
    if (debug_mode) {
        Serial.print("Received message of type: ");
        Serial.println(type);
        Serial.print("Length: ");
        Serial.println(length);
        Serial.print("Data: ");
        for (int i = 0; i < length; i++) {
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    switch (type) {
        case 'A': { // Actuator control
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
            break;
        }
        case 'M': { // Motor control
            mR_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
            mL_speed = (int8_t)data[2];
            if (debug_mode) {
                Serial.print("Left Speed: ");
                Serial.println(mL_speed);
                Serial.print("Right Speed: ");
                Serial.println(mR_speed);
            }
            break;
        }
        case 'S': { // Servo control
            servo_state = data[1];  // Adjusted index to skip the type byte
            if (debug_mode) {
                Serial.print("Servo State: ");
                Serial.println(servo_state);
            }
            break;
        }
        case 'L': { // LED control
            led_r = data[1];  // Adjusted index to skip the type byte
            led_y = data[2];
            led_g = data[3];
            led_b = data[4];
            if (debug_mode) {
                Serial.print("Red: ");
                Serial.println(led_r);
                Serial.print("Yellow: ");
                Serial.println(led_y);
                Serial.print("Green: ");
                Serial.println(led_g);
                Serial.print("Blue: ");
                Serial.println(led_b);
            }
            break;
        }
        default:
            Serial.println("Unknown message type");
            break;
    }
}

void stop_all() {
    act_left.stop();
    act_right.stop();
    act_bucket.stop();
    motor_left.stop();
    motor_right.stop();
}

void fault(String msg) {
    while (true) {
        stop_all();
        ledr_pin.write(1);
        delay(500);
        ledr_pin.write(0);
        delay(500);
        Serial.println("Critical Error: " + msg + " - Reset arduino to continue.");
    }
}

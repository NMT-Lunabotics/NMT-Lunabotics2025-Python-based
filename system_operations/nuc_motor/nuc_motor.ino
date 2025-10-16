// Debug mode flag
bool debug_mode = false;
bool calibrate_actuators_flag = true;
//TODO check pins

//////// MOTORS ////////
const float rpm_to_pwm_constant = 2.5;
const int motor1_pwm_pin = 3;  // Motor 1 speed (PWM)
const int motor1_dir_pin1 = 4;  // Motor 1 direction pin 1
const int motor1_dir_pin2 = 9;  // Motor 1 direction pin 2

const int motor2_pwm_pin = 6;  // Motor 2 speed (PWM)
const int motor2_dir_pin1 = 7;  // Motor 2 direction pin 1
const int motor2_dir_pin2 = 8;  // Motor 2 direction pin 2

// Motor speeds
int mL_speed = 0;
int mR_speed = 0;
int pwm_motor1 = 0;
int pwm_motor2 = 0;

// Timing
int update_rate = 100; //hz
unsigned long last_update_time = 0;
unsigned long current_time = 0;
const unsigned long estop_timeout = 1000; // 1 second timeout
unsigned long last_message_time = 0;
bool emergency_stop = false;
bool doomsday = false;

void processMessage(byte* data, int length);

void setup(){
    Serial.begin(115200);
    Serial.flush();
    pinMode(motor1_pwm_pin, OUTPUT);
    pinMode(motor1_dir_pin1, OUTPUT);
    pinMode(motor1_dir_pin2, OUTPUT);
  
    pinMode(motor2_pwm_pin, OUTPUT);
    pinMode(motor2_dir_pin1, OUTPUT);
    pinMode(motor2_dir_pin2, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
        if (Serial.read() == 0x02) { // Start byte
            while (Serial.available() < 1) {} // Wait for type and length bytes
            int length = Serial.read();
            while (Serial.available() < length + 1) {} // Wait for the entire message
            byte data[length];
            Serial.readBytes(data, length);
            while (Serial.available() < 1) {} // Wait for end byte
            if (Serial.read() == 0x03) { // End byte
                processMessage(data, length);
            } else {
                Serial.println("End byte not found");
            }
        }
    }

    current_time = millis();

    if (current_time - last_update_time >= 1000 / update_rate) {
        last_update_time = current_time;

          // Convert RPM to PWM using the constant
          pwm_motor1 = constrain(abs(mL_speed) * rpm_to_pwm_constant, 0, 255);
          pwm_motor2 = constrain(abs(mR_speed) * rpm_to_pwm_constant, 0, 255);

          // Set motor 1 direction
          if (mL_speed >= 0) {
            digitalWrite(motor1_dir_pin1, HIGH);
            digitalWrite(motor1_dir_pin2, LOW);
          } else {
            digitalWrite(motor1_dir_pin1, LOW);
            digitalWrite(motor1_dir_pin2, HIGH);
          }

          // Set motor 1 speed
          analogWrite(motor1_pwm_pin, pwm_motor1);

          // Set motor 2 direction
          if (mR_speed >= 0) {
            digitalWrite(motor2_dir_pin1, HIGH);
            digitalWrite(motor2_dir_pin2, LOW);
          } else {
            digitalWrite(motor2_dir_pin1, LOW);
            digitalWrite(motor2_dir_pin2, HIGH);
          }

          // Set motor 2 speed
          analogWrite(motor2_pwm_pin, pwm_motor2);
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
            // aLR_tgt = (int16_t)((data[1] << 8) | data[2]);  // Adjusted index to skip the type byte
            // aB_tgt = (int16_t)((data[3] << 8) | data[4]);
            // aL_speed = (int8_t)data[5];
            // aR_speed = aL_speed;
            // aB_speed = (int8_t)data[6];
            // if (debug_mode) {
            //     Serial.print("Arm Position: ");
            //     Serial.println(aLR_tgt);
            //     Serial.print("Bucket Position: ");
            //     Serial.println(aB_tgt);
            //     Serial.print("Arm Velocity: ");
            //     Serial.println(aL_speed);
            //     Serial.print("Bucket Velocity: ");
            //     Serial.println(aB_speed);
            // }
            break;
        }
        case 'M': { // Motor control
            mL_speed = (int8_t)data[1];  // Adjusted index to skip the type byte
            mR_speed = (int8_t)data[2];
            if (debug_mode) {
                Serial.print("Left Speed: ");
                Serial.println(mL_speed);
                Serial.print("Right Speed: ");
                Serial.println(mR_speed);
            }
            break;
        }
        case 'S': { // Servo control
            // servo_state = data[1];  // Adjusted index to skip the type byte
            // if (debug_mode) {
            //     Serial.print("Servo State: ");
            //     Serial.println(servo_state);
            // }
            break;
        }
        case 'L': { // LED control
            // led_r = data[1];  // Adjusted index to skip the type byte
            // led_y = data[2];
            // led_g = data[3];
            // led_b = data[4];
            // if (debug_mode) {
            //     Serial.print("Red: ");
            //     Serial.println(led_r);
            //     Serial.print("Yellow: ");
            //     Serial.println(led_y);
            //     Serial.print("Green: ");
            //     Serial.println(led_g);
            //     Serial.print("Blue: ");
            //     Serial.println(led_b);
            // }
            break;
        }
        default:
            Serial.println("Unknown message type");
            break;
    }
}
const int motor1_pwm_pin = 3;  // Motor 1 speed (PWM)
const int motor1_dir_pin1 = 4;  // Motor 1 direction pin 1
const int motor1_dir_pin2 = 9;  // Motor 1 direction pin 2

const int motor2_pwm_pin = 6;  // Motor 2 speed (PWM)
const int motor2_dir_pin1 = 7;  // Motor 2 direction pin 1
const int motor2_dir_pin2 = 8;  // Motor 2 direction pin 2

const float rpm_to_pwm_constant = 2.5;

void setup() {
  Serial.begin(2000000);  // Set baud rate to match Python script

  // Set motor control pins as output
  pinMode(motor1_pwm_pin, OUTPUT);
  pinMode(motor1_dir_pin1, OUTPUT);
  pinMode(motor1_dir_pin2, OUTPUT);

  pinMode(motor2_pwm_pin, OUTPUT);
  pinMode(motor2_dir_pin1, OUTPUT);
  pinMode(motor2_dir_pin2, OUTPUT);
}

void loop() {
          int pwm_motor1 = 1;
          int pwm_motor2 = 1;

          // Set motor 1 direction
          if (rpm_motor1 >= 0) {
            digitalWrite(motor1_dir_pin1, HIGH);
            digitalWrite(motor1_dir_pin2, LOW);
          } else {
            digitalWrite(motor1_dir_pin1, LOW);
            digitalWrite(motor1_dir_pin2, HIGH);
          }

          // Set motor 1 speed
          analogWrite(motor1_pwm_pin, pwm_motor1);

          // Set motor 2 direction
          if (rpm_motor2 >= 0) {
            digitalWrite(motor2_dir_pin1, HIGH);
            digitalWrite(motor2_dir_pin2, LOW);
          } else {
            digitalWrite(motor2_dir_pin1, LOW);
            digitalWrite(motor2_dir_pin2, HIGH);
          }

          // Set motor 2 speed
          analogWrite(motor2_pwm_pin, pwm_motor2);

          // Send success message to the ROS node
          // Serial.println("1");
        }
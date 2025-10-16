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
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('>');  // Read until '>'
    
    if (inputString.startsWith("<")) {  // Ensure message starts with '<'
      inputString = inputString.substring(1);  // Remove the starting '<'
      // Serial.println(inputString);
      if (inputString.startsWith("M")) {
        inputString = inputString.substring(2);  // Remove the starting 'M'
        int commaIndex = inputString.indexOf(',');
        if (commaIndex > 0) {
          // Parse the RPM values for motor 1 and motor 2
          int rpm_motor1 = inputString.substring(0, commaIndex).toInt();
          int rpm_motor2 = inputString.substring(commaIndex + 1).toInt();

          // Convert RPM to PWM using the constant
          int pwm_motor1 = constrain(abs(rpm_motor1) * rpm_to_pwm_constant, 0, 255);
          int pwm_motor2 = constrain(abs(rpm_motor2) * rpm_to_pwm_constant, 0, 255);

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
      }
    }
  }
}
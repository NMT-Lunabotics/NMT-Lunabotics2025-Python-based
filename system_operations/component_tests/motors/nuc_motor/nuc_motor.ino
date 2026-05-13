// Left motor
const int motor1_pwm_pin = 3;    // PWM pin
const int motor1_dir_pin1 = 4;   // Direction pin 1
const int motor1_dir_pin2 = 9;   // Direction pin 2

// Right motor
const int motor2_pwm_pin = 6;    // PWM pin
const int motor2_dir_pin1 = 7;   // Direction pin 1
const int motor2_dir_pin2 = 8;   // Direction pin 2

void setup() {
  // Set pins as outputs
  pinMode(motor1_pwm_pin, OUTPUT);
  pinMode(motor1_dir_pin1, OUTPUT);
  pinMode(motor1_dir_pin2, OUTPUT);

  pinMode(motor2_pwm_pin, OUTPUT);
  pinMode(motor2_dir_pin1, OUTPUT);
  pinMode(motor2_dir_pin2, OUTPUT);

  // Set forward direction
  digitalWrite(motor1_dir_pin1, HIGH);
  digitalWrite(motor1_dir_pin2, LOW);

  digitalWrite(motor2_dir_pin1, HIGH);
  digitalWrite(motor2_dir_pin2, LOW);
}

void loop() {
  // Stop motors
  analogWrite(motor1_pwm_pin, 0);
  analogWrite(motor2_pwm_pin, 0);
  delay(2000);

  // Half speed
  analogWrite(motor1_pwm_pin, 128);
  analogWrite(motor2_pwm_pin, 128);
  delay(2000);

  // Full speed
  analogWrite(motor1_pwm_pin, 255);
  analogWrite(motor2_pwm_pin, 255);
  delay(2000);
}

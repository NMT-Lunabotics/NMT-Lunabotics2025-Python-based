#include <IBusBM.h>

IBusBM ibusRc;

HardwareSerial& ibusRcSerial = Serial1;
HardwareSerial& debugSerial = Serial;

// H-Bridge wiring (L298N)
const int ENA = 9;   // Left motor enable (PWM)
const int IN1 = 22;  // Left motor input 1
const int IN2 = 23;  // Left motor input 2

const int ENB = 10;  // Right motor enable (PWM)
const int IN3 = 24;  // Right motor input 3
const int IN4 = 25;  // Right motor input 4

// IR sensors
const int leftIR  = 32;
const int rightIR = 33;

void setup() {
  debugSerial.begin(74880);
  ibusRc.begin(ibusRcSerial);

  // Motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR pins
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  // Stop motors at start
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Read channel and map to -100..100
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Proportional motor drive with deadzone
void driveMotor(int channelValue, int enPin, int inPin1, int inPin2) {
  if (channelValue < -30) {
    digitalWrite(inPin1, HIGH);
    digitalWrite(inPin2, LOW);
    int pwmVal = map(abs(channelValue), 30, 100, 0, 255);
    analogWrite(enPin, pwmVal);
  } else if (channelValue > 30) {
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, HIGH);
    int pwmVal = map(channelValue, 30, 100, 0, 255);
    analogWrite(enPin, pwmVal);
  } else {
    analogWrite(enPin, 0);
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
  }
}

// direct motor control (speed 0–255, dir: 1=forward, -1=back, 0=stop)
void setMotor(int speed, int dir, int enPin, int inPin1, int inPin2) {
  speed = constrain(speed, 0, 255);
  if (dir > 0) {
    // FORWARD
    digitalWrite(inPin1, HIGH);   // << swapped
    digitalWrite(inPin2, LOW);
    analogWrite(enPin, speed);
  } else if (dir < 0) {
    // BACKWARD
    digitalWrite(inPin1, LOW);    // << swapped
    digitalWrite(inPin2, HIGH);
    analogWrite(enPin, speed);
  } else {
    analogWrite(enPin, 0);
    digitalWrite(inPin1, LOW);
    digitalWrite(inPin2, LOW);
  }
}
void loop() {
  ibusRc.loop();

  int leftValue  = readChannel(1, -100, 100, 0);   // Ch3
  int rightValue = readChannel(2, -100, 100, 0);   // Ch2
  int ch6Value   = readChannel(5, -100, 100, 0);   // Ch6

  int leftIRValue  = digitalRead(leftIR);
  int rightIRValue = digitalRead(rightIR);

  if (ch6Value <= 0) {
    // --- Manual mode ---
    driveMotor(leftValue, ENA, IN1, IN2);
    driveMotor(rightValue, ENB, IN3, IN4);
  } else {
    // --- Autonomous obstacle avoid mode ---
    // Drive forward until obstacle
    setMotor(1, 1, ENA, IN1, IN2); // slow forward
    setMotor(1, 1, ENB, IN3, IN4);

    if (leftIRValue == HIGH || rightIRValue == HIGH) {
      // Obstacle detected -> backup
      setMotor(1, -1, ENA, IN1, IN2);
      setMotor(1, -1, ENB, IN3, IN4);
      delay(2000);

      // Turn right
      setMotor(0, 0, ENA, IN1, IN2);    // stop left motor
      setMotor(1, 1, ENB, IN3, IN4);  // right motor forward
      delay(1000);

      // Resume forward
      setMotor(5, 1, ENA, IN1, IN2);
      setMotor(5, 1, ENB, IN3, IN4);
    }
  }

  // Debug
  debugSerial.print("Ch3: ");
  debugSerial.print(leftValue);
  debugSerial.print(" | Ch2: ");
  debugSerial.print(rightValue);
  debugSerial.print(" | Ch6: ");
  debugSerial.print(ch6Value);
  debugSerial.print(" | Left IR: ");
  debugSerial.print(leftIRValue);
  debugSerial.print(" | Right IR: ");
  debugSerial.println(rightIRValue);

  delay(50);
}
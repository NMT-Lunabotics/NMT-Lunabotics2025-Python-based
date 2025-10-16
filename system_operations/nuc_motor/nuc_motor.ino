// Left motor
const int LEFT_DAC1 = 4;  // A1 pin
const int LEFT_DAC2 = 6;  // B1 pin
const int LEFT_PWM  = 9;  // EN1 pin

// Right motor
const int RIGHT_DAC1 = 7; // A2 pin
const int RIGHT_DAC2 = 8; // B2 pin
const int RIGHT_PWM  = 3; // EN2 pin

void setup() {
  pinMode(LEFT_DAC1, OUTPUT);
  pinMode(LEFT_DAC2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);

  pinMode(RIGHT_DAC1, OUTPUT);
  pinMode(RIGHT_DAC2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  // Forward direction
  digitalWrite(LEFT_DAC1, HIGH);
  digitalWrite(LEFT_DAC2, LOW);
  digitalWrite(RIGHT_DAC1, HIGH);
  digitalWrite(RIGHT_DAC2, LOW);
}

void loop() {
  // Run both motors very slowly
  analogWrite(LEFT_PWM, 10);   // 0–255 PWM
  analogWrite(RIGHT_PWM, 10);
}

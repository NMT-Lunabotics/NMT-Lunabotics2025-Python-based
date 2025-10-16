// Left motor
const int A1 = 4;
const int B1 = 6;
const int PWM1 = 9;

// Right motor
const int A2 = 7;
const int B2 = 8;
const int PWM2 = 3;

void setup() {
  pinMode(A1, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(PWM1, OUTPUT);

  pinMode(A2, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  // Forward direction
  digitalWrite(A1, HIGH);
  digitalWrite(B1, LOW);
  digitalWrite(A2, HIGH);
  digitalWrite(B2, LOW);
}

void loop() {
  // Run both motors very slowly
  analogWrite(PWM1, 10);   // range = 0–255
  analogWrite(PWM2, 10);
}

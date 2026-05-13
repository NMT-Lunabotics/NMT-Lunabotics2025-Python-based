
#define DRV21_PWM_PIN 8

void setup() {
  Serial.begin(115200);
  pinMode(DRV21_PWM_PIN, OUTPUT);
}

void loop() {
    digitalWrite(DRV21_PWM_PIN, 0);
}
#define SCL_PIN 34  // Direction
#define SDA_PIN 36  // Speed (PWM)
#define ACT1_POT A0 //Actuator


//-----OLD ACTUATORS-----
#define ACT1_POT_MIN 32
#define ACT1_POT_MAX 946

#define ACT2_POT_MIN 30
#define ACT2_POT_MAX 943

void setup() {
  Serial.begin(115200);
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SCL_PIN, LOW);
  analogWrite(SDA_PIN, 0);
}

int val = 0;


void setMotor(int speed) {
  // speed: -255 → 0 → 255
  if (speed > 0) {
    digitalWrite(SCL_PIN, HIGH);   // forward
    analogWrite(SDA_PIN, speed);   // 0-255 PWM
  } else if (speed < 0) {
    digitalWrite(SCL_PIN, LOW);    // backward
    analogWrite(SDA_PIN, speed);  // PWM magnitude
  } else {
    digitalWrite(SCL_PIN, LOW);    // stop
    analogWrite(SDA_PIN, 0);      
  }
}

void loop() {
  //val = analogRead(ACT1_POT); 
  //Serial.println(val);          


  //setMotor(255);    // Forward half speed
  //delay(2000);
  
  setMotor(-255);   // Full reverse
  delay(2000);
  
  //setMotor(0);      // Stop
  //delay(500);
}
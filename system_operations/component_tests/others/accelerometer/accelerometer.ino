#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
Servo myservo;

// Calulate yaw drift to get less yaw drifting
#define BIAS_SAMPLES 25
float biasOffset=0;
static int gyroSampleCount = 0;
static float gyroBiasSum = 0;
// Record yaw when when yaw is not being used to reduce dirfting yaw
float biasHome = 0;

float gyroRate = 0;
float gyroYaw=0;
unsigned long lastTime;
bool robotRotating = false;


MPU6050 mpu;  
   

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if(mpu.testConnection()) Serial.println("MPU6050 connected successfully");
  else Serial.println("MPU6050 connection failed");
  lastTime = millis();
  myservo.attach(2);
  myservo.write(0);
  delay(1000);
}

int currentAngle = 0;  // track servo position

// Call this function repeatedly in loop to move servo
// speed: -5 to 5 degrees per call (negative = backward, positive = forward)
void moveServo(int speed) {
  currentAngle += speed;           // update position
  if(currentAngle > 180) currentAngle = 180;  // clamp max
  if(currentAngle < 0) currentAngle = 0;      // clamp min
  myservo.write(currentAngle);     // move servo a small step
}

void updatePotentiometerData(bool zeroHomeBias=true, bool calibrateBias = true){
  int16_t gx, gy, gz; 
  mpu.getRotation(&gx, &gy, &gz);
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float rate = gz / 131.0;
  if(calibrateBias==true && gyroSampleCount<BIAS_SAMPLES){
    gyroBiasSum += rate;
    gyroSampleCount++;
    if (gyroSampleCount >= BIAS_SAMPLES) biasOffset=gyroBiasSum/BIAS_SAMPLES;
  }
  else if(zeroHomeBias==true) biasHome=gyroRate;
  //if(robotRotating==false) 
  gyroRate += (rate-biasOffset) * dt;
  //else 
  //gyroRate += rate * dt;
  gyroYaw=gyroRate-biasHome;
}  

bool testRotate=true;

void rotateRobot(float angle=-90){
  robotRotating=true;
  moveServo(1);
  if(gyroYaw<=angle) {
    robotRotating=false;
    testRotate=false;
  }
}


void loop() {
  if(robotRotating==true) updatePotentiometerData(false);
  else updatePotentiometerData(true);
  if(testRotate==true && biasOffset!=0) rotateRobot();

  Serial.print("Yaw: ");
  Serial.println(gyroYaw);
  delay(20);
}

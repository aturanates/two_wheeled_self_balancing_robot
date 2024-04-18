#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;


double Kp = 3.234;  
double Ki = 15.12;  
double Kd = 0.19;  


double setpoint = 0;  
double angle = 0;  
double prevAngle = 0;  
double error = 0;  
double errorSum = 0;  
double errorDiff = 0;  
double motorSpeed = 0;  


const int motorPin1 = 3;  
const int motorPin2 = 5;  

void setup() {

  Serial.begin(9600);
  

  Wire.begin();
  mpu.initialize();
  

  mpu.calibrateGyro();
  
 
  setpoint = getIMUAngle();
  

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {

  angle = getIMUAngle();
  
 
  error = setpoint - angle;
  errorSum += error;
  errorDiff = error - prevAngle;
  
  motorSpeed = (Kp * error) + (Ki * errorSum) + (Kd * errorDiff);
  
  if (motorSpeed > 255) {
    motorSpeed = 255;
  }
  else if (motorSpeed < -255) {
    motorSpeed = -255;
  }
  
  if (motorSpeed > 0) {
    analogWrite(motorPin1, motorSpeed);
    analogWrite(motorPin2, 0);
  }
  else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, -motorSpeed);
  }
  
  prevAngle = error;
}

double getIMUAngle() {
  int16_t accelerometerX = mpu.getAccelerationX();
  int16_t accelerometerY = mpu.getAccelerationY();
  int16_t accelerometerZ = mpu.getAccelerationZ();
  
  double radians = atan2(accelerometerY, accelerometerZ);
  double degrees = radians * (180.0 / M_PI);
  
  return degrees;
}

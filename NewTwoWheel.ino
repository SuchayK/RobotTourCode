#include <AFMotor.h>  
#include <PID_v1.h>       
#include <MPU6050_tockn.h>
#include <Wire.h>  

MPU6050 mpu6050(Wire);

AF_DCMotor right(2);
AF_DCMotor left(3);

const float wheeldiameter = 62;
const float Kp = 3;
int encoderPin = 19;
int encoderPin2 = 18;

volatile unsigned long totalPulses = 0;
volatile long totalAngle = 0;

volatile unsigned long totalPulses2 = 0;
volatile long totalAngle2 = 0;

float inang = 0;
float Kt = 1.05;

float counter = 0;

// int Kp = 3;
// int Ki = 2;
// int Kd = 1;

// double setPoint, Input, Output;

// PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup() {
  stopMotors();
  Wire.begin();  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);

  // pinMode(encoderPin, INPUT);
  // pinMode(encoderPin2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), interruptFunction2, RISING);

  Serial.begin(9600);

  inang = mpu6050.getAngleZ();

  // move(35);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // tL();
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // tL();
  // move(25);
  // move(25);
  // tL();
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);
  // move(25);

  move(35);
  move(25);
  move(25);
  move(25);
  move(25);
  move(25);
  move(25);
  move(25);
  tL();
  move(25);
  move(25);
  move(25);
  move(25);
  move(25);
  move(25);
  tL();
  move(25);
  move(25);
  tL();
  move(25);
  move(25);
  move(25);
  move(25);
  tR();
  move(25);
  move(25);
  tL();
  tL();
  move(25);
  move(25);
  tR();
  move(25);
  move(25);

  

}


void loop() {
  // put your main code here, to run repeatedly:
  //updatempu();
}

void updatempu() {

  mpu6050.update();
  //Serial.println(mpu6050.getAngleZ());

}

void interruptFunction() {
  totalPulses = totalPulses + 1;
  
}

void interruptFunction2() {
  totalPulses2 = totalPulses2 + 1;

}
int CMtoSteps(float cm) {

  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.1415) / 10; // Calculate wheel circumference in cm
  float cm_step = circumference / 20;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result

}

float stepsToCM(int steps){
  float result;
  float circumference = (wheeldiameter * 3.1419) / 10;
  float cm_step = circumference / 20; 

  return cm_step * steps;

}

void move(float distance){
  left.setSpeed(102);
  right.setSpeed(110);

  int pulses = CMtoSteps(distance) - 3; //6 for 50
  int n = 0;
  double ang = mpu6050.getAngleZ();
  //experimental 
  inang = ang;
  while (totalPulses < pulses) {
    left.run(FORWARD);
    right.run(FORWARD);
    updatempu();
    //right.setSpeed(110 + n);
  }

  stopMotors();
  delay(600);
  totalPulses = 0;
  totalPulses2 = 0;
  updatempu();

  left.setSpeed(130);
  right.setSpeed(130);

  while (mpu6050.getAngleZ() > inang + 1 || mpu6050.getAngleZ() < inang - 1){

    if (mpu6050.getAngleZ() > inang){
      left.run(FORWARD);
      right.run(BACKWARD);
      updatempu();
    }

    if (mpu6050.getAngleZ() < inang){
      left.run(BACKWARD);
      right.run(FORWARD);
      updatempu();
    }

  }

  stopMotors();
  delay(1000);
  totalPulses = 0;
  totalPulses2 = 0;

}

void tL(){

  left.setSpeed(130);
  right.setSpeed(130);

  inang = 78 + inang + counter;
  counter = counter + 0.5;

  while(inang > mpu6050.getAngleZ()){

    left.run(BACKWARD);
    right.run(FORWARD);
    updatempu();

  }

  stopMotors();
  delay(1000);
  
  totalPulses = 0;
  totalPulses2 = 0;

}

void tR(){

  left.setSpeed(130);
  right.setSpeed(130);

  inang = -78 + inang;

  while(inang < mpu6050.getAngleZ()){

    left.run(FORWARD);
    right.run(BACKWARD);
    updatempu();

  }
  stopMotors();
  delay(1000);
  totalPulses = 0;
  totalPulses2 = 0;

}

void stopMotors(){

  left.setSpeed(0);
  right.setSpeed(0);
  left.run(RELEASE);
  right.run(RELEASE);

}

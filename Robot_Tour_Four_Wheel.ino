#include <AFMotor.h>  
#include <PID_v1.h>       
#include <MPU6050_tockn.h>
#include <Wire.h>  

MPU6050 mpu6050(Wire);

AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

const float wheeldiameter = 55.2; // 55.2
const float Kp = 3;
int encoderPin = 19;
int encoderPin2 = 18;

volatile unsigned long totalPulses = 0;
volatile long totalAngle = 0;

volatile unsigned long totalPulses2 = 0;
volatile long totalAngle2 = 0;

volatile int anglePerPulse = 0.314;

volatile unsigned long lastTime = 0;
volatile unsigned long currentTime = 0;
volatile double deltaT = 0;

volatile double angularVelocity = 0;
volatile double angularVelocity2 = 0;
volatile unsigned long timeAverageAngularVelocity = 0;
volatile unsigned long timeAverageAngularVelocity2 = 0;
volatile int sumPulses = 0;
volatile int sumPulses2 = 0;
volatile int sampleRate = 20;

float inang = 0;

// int Kp = 3;
// int Ki = 2;
// int Kd = 1;

// double setPoint, Input, Output;

// PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup() {

  Wire.begin();  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);

  // pinMode(encoderPin, INPUT);
  // pinMode(encoderPin2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPin), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), interruptFunction2, RISING);

  Serial.begin(9600);

  inang = mpu6050.getAngleZ();

  // shitMove(50);
  // shitMove(50);
  // shitMove(50);
  // shitMove(50);
  // turnLeft();
  // turnLeft();
  // shitMove(50);
  // shitMove(50);
  // shitMove(50);
  // shitMove(50);

  shitMove(35);
  turnLeft();
  shitMove(50);
  turnLeft();
  turnLeft();
  shitMove(50);
  turnLeft();
  shitMove(50);
  shitMove(50);
  turnLeft();
  shitMove(50);
  turnRight();
  shitMove(50);
  turnLeft();
  turnLeft();
  shitMove(50);
  turnLeft();
  shitMove(50);
  turnLeft();
  shitMove(50);
  turnRight();
  shitMove(50);
  shitMove(50);
  turnRight();
  shitMove(50);
  shitMove(50);
  shitMove(35);
  turnRight();
  shitMove(50);
  turnLeft();
  shitMove(50);
  turnLeft();
  shitMove(50);
  turnLeft();
  turnLeft();
  shitMove(50);
  turnRight();
  shitMove(35);

  // turnLeft();
  // turnRight();
  // turnLeft();
  // turnRight();

  // shitMove(50);
}

void loop() {
  
  /*
  while (!(mpu6050.getAngleZ() > 175 && mpu6050.getAngleZ() < 180)){
    turnLeft();
  }
  */


}

void updatempu() {

  mpu6050.update();
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());

}

void interruptFunction() {

  currentTime = millis();

  deltaT = currentTime - lastTime;
  lastTime = currentTime;

  totalPulses = totalPulses + 1;
  totalAngle = totalPulses * anglePerPulse;

  sumPulses = sumPulses + 1;

  timeAverageAngularVelocity = timeAverageAngularVelocity + deltaT;

  if (sumPulses >= sampleRate){

    angularVelocity = 500 * ((double) (sumPulses * anglePerPulse)) / ((double) timeAverageAngularVelocity);
    //Serial.println((String) "Total angle:" + totalAngle + " Angular Velocity:" + angularVelocity);
    sumPulses = 0;
    timeAverageAngularVelocity = 0;

  }
  
}

void interruptFunction2() {

  currentTime = millis();

  deltaT = currentTime - lastTime;
  lastTime = currentTime;

  totalPulses2 = totalPulses2 + 1;
  totalAngle2 = totalPulses2 * anglePerPulse;

  sumPulses2 = sumPulses2 + 1;

  timeAverageAngularVelocity2 = timeAverageAngularVelocity2 + deltaT;

  if (sumPulses2 >= sampleRate){

    angularVelocity2 = 500 * ((double) (sumPulses2 * anglePerPulse)) / ((double) timeAverageAngularVelocity2);
    //Serial.println((String) "Total angle:" + totalAngle2 + " Angular Velocity:" + angularVelocity2);
    sumPulses2 = 0;
    timeAverageAngularVelocity2 = 0;

  }

}
int CMtoSteps(float cm) {

  int result;  
  float circumference = (wheeldiameter * 3.1415) / 10; 
  float cm_step = circumference / 20;  
  
  float f_result = cm / cm_step; 
  result = (int) f_result; 
  
  return result; 

}

float stepsToCM(int steps){
  float result;
  float circumference = (wheeldiameter * 3.1419) / 10;
  float cm_step = circumference / 20; 

  return cm_step * steps;

}

void move(float distance){
  motor1.setSpeed(130); //130
  motor2.setSpeed(130); //130
  motor3.setSpeed(131); //146
  motor4.setSpeed(131); //135
  
  while(totalPulses2 < CMtoSteps(distance)){
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    updatempu();
    // Serial.println((totalPulses + totalPulses2) / 2);
    Serial.println(totalPulses2);
  }
  stopMotors();
  delay(2000);
  totalPulses = 0;
  totalPulses2 = 0;

  motor1.setSpeed(50); 
  motor2.setSpeed(50);
  motor3.setSpeed(-50);
  motor4.setSpeed(-50);

  while (inang - 5 > mpu6050.getAngleZ() || inang + 5 > mpu6050.getAngleZ()){
    
    if (inang > mpu6050.getAngleZ()) {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      updatempu();
    } 
    
    if (inang < mpu6050.getAngleZ()) {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
      updatempu();
    }

    delay(50);
    stopMotors();
    delay(500);

  }

  stopMotors();
  delay(2000);
  totalPulses = 0;


}

void moveBack(float distance) {

  motor1.setSpeed(130); 
  motor2.setSpeed(130);
  motor3.setSpeed(146);
  motor4.setSpeed(130);
  
  while(totalPulses < CMtoSteps(distance) - 1){
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    updatempu();
    Serial.println(totalPulses);
  }
  stopMotors();
  delay(2000);
  totalPulses = 0;

  motor1.setSpeed(50); 
  motor2.setSpeed(50);
  motor3.setSpeed(-50);
  motor4.setSpeed(-50);

  while (inang - 1 > mpu6050.getAngleZ() || inang + 1 > mpu6050.getAngleZ()){
    
    if (inang > mpu6050.getAngleZ()) {
      motor1.run(FORWARD);
      motor2.run(FORWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      updatempu();
    } 
    
    if (inang < mpu6050.getAngleZ()) {
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(FORWARD);
      motor4.run(FORWARD);
      updatempu();
    }

  }

  stopMotors();
  delay(2000);
  totalPulses = 0;

}

void moveForward(float distance, int duration) {
  float m2s =  0;
  float m4s = 0;
  int init_pulses = totalPulses;
  int init_pulses2 = totalPulses2;
  int init_time = millis();
  int dPulses = CMtoSteps(distance);
  motor2.setSpeed(m2s);
  motor4.setSpeed(m4s);

  duration = duration * 500;

  while(millis()<init_time+duration){
    if(totalPulses<dPulses+init_pulses){
      motor2.run(FORWARD);
      float curV = stepsToCM(totalPulses-init_pulses) / (millis() - init_time);
      float reqV = (distance - stepsToCM(totalPulses-init_pulses)) / (duration - (millis() - init_time));
      if(curV > reqV){
        m2s = m2s-Kp;
        motor2.setSpeed(m2s);
      }else if(curV < reqV){
        if(m2s+Kp<220){
          m2s = m2s+Kp;
          motor2.setSpeed(m2s);
        }
      }
    }else{
      motor2.setSpeed(0);
    }

    if(totalPulses2<dPulses+init_pulses2){
      motor4.run(FORWARD);
      float curV = stepsToCM(totalPulses2-init_pulses2) / (millis() - init_time);
      float reqV = (distance - stepsToCM(totalPulses2-init_pulses2)) / (duration - (millis() - init_time));
      if(curV > reqV){
        m4s = m4s-Kp;
        motor4.setSpeed(m4s);
      }else if(curV < reqV){
        m4s = m4s+Kp;
        motor4.setSpeed(m4s);
      }
    }else{
      motor4.setSpeed(0);
    }
  }


}

void turnLeft() {
  motor1.setSpeed(110); //120
  motor2.setSpeed(110); //120 
  motor3.setSpeed(110); //125
  motor4.setSpeed(110); //120

  inang = 78 + inang;

  while (inang > mpu6050.getAngleZ()){ //-15z
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD); 
    motor4.run(BACKWARD);
    updatempu();
  }
  inang+=12;
  stopMotors();
  totalPulses = 0;
  totalPulses2 = 0;
  delay(2000);

}

void turnLefty(int x) {
  motor1.setSpeed(120); 
  motor2.setSpeed(120);
  motor3.setSpeed(-125);
  motor4.setSpeed(-120);

  while (totalPulses < 21){
    motor1.run(FORWARD);
    motor3.run(BACKWARD);
    motor2.run(FORWARD);
    motor4.run(BACKWARD);
  }

  stopMotors();
  totalPulses = 0;
  delay(4000);

}

void turnRighty(int x) {
  motor1.setSpeed(-120); 
  motor2.setSpeed(-120);
  motor3.setSpeed(125);
  motor4.setSpeed(120);

  while (totalPulses < 21){
    motor1.run(BACKWARD);
    motor3.run(FORWARD);
    motor2.run(BACKWARD);
    motor4.run(FORWARD);
  }

  stopMotors();
  totalPulses = 0;
  delay(4000);

}

void turnRight() {
  motor1.setSpeed(110); 
  motor2.setSpeed(110);
  motor3.setSpeed(110);
  motor4.setSpeed(110);

  inang = -78 + inang;

  while (inang < mpu6050.getAngleZ()){
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    updatempu();
  }
  inang -= 12;
  stopMotors();
  totalPulses = 0;
  totalPulses2 = 0;
  delay(2000);
}

void turnLeft() {
  motor1.setSpeed(110); 
  motor2.setSpeed(110);
  motor3.setSpeed(110);
  motor4.setSpeed(110);

  inang = 78 + inang;

  while (inang < mpu6050.getAngleZ()){
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    updatempu();
  }
  inang += 12;
  stopMotors();
  totalPulses = 0;
  totalPulses2 = 0;
  delay(2000);
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  motor4.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void move(float distance){

  motor1.setSpeed(130); 
  motor2.setSpeed(130);
  motor3.setSpeed(146);
  motor4.setSpeed(130);
  
  while(totalPulses < CMtoSteps(distance) - 1){
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    updatempu();
    Serial.println(totalPulses);
  }
  stopMotors();
  delay(2000);
  totalPulses = 0;

  motor1.setSpeed(100); 
  motor2.setSpeed(100);
  motor3.setSpeed(-100);
  motor4.setSpeed(-100);

  // while (inang - 5 > mpu6050.getAngleZ() || inang + 5 > mpu6050.getAngleZ()){
    
  //   if (inang > mpu6050.getAngleZ()) {
  //     motor1.run(FORWARD);
  //     motor2.run(FORWARD);
  //     motor3.run(BACKWARD);
  //     motor4.run(BACKWARD);
  //     updatempu();
  //   } 
    
  //   if (inang < mpu6050.getAngleZ()) {
  //     motor1.run(BACKWARD);
  //     motor2.run(BACKWARD);
  //     motor3.run(FORWARD);
  //     motor4.run(FORWARD);
  //     updatempu();
  //   }

  //   delay(50);
  //   stopMotors();
  //   delay(500);

  // }

  // stopMotors();
  // delay(2000);
  // totalPulses = 0;


}

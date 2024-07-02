#include <AFMotor.h>  
#include <PID_v1.h>      
#include <MPU6050_tockn.h>
#include <Wire.h>  


MPU6050 mpu6050(Wire);

AF_DCMotor motor2(2);
AF_DCMotor motor4(3);


const float wheeldiameter = 66.4;
const float Kp = 0.16;
const float Kc = 0.5;
const float Ke = 0.5;
int encoderPin = 19;
int encoderPin2 = 18;
volatile int totalPulses = 0;
double ref_ang= 0.0;
volatile int totalPulses2 = 0;
int sampleRate = 20;

// int Kp = 3;
// int Ki = 2;
// int Kd = 1;


//double turnSetPoint, Inp, Outp;


//PID turnPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);


void setup() {

  Wire.begin();  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  // Serial.begin(9600);

  //pinMode(encoderPin, INPUT_PULLUP);
  //pinMode(encoderPin2, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(encoderPin), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), interruptFunction2, RISING);
  delay(2000);
  // ref_ang = mpu6050.getAngleZ();
  // delay(2000);
  // instruct();
  moveBackward(25);
}

void loop() {

  // Serial.println(mpu6050.getAngleZ());
  // updatempu();

}
void instruct(){
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(20);
  //turnLeft();
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(20);
  //turnLeft();
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(15);
  //turnLeft();
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(25);
  moveForward(15);
}
void updatempu() {
  mpu6050.update();
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());
}
void moveForward(float distance){ //basic ass moving (no encoders)
  double init_time = millis();
  double init_ang = mpu6050.getAngleZ();
  motor2.setSpeed(155); // 155
  motor4.setSpeed(180); // 180
  totalPulses = totalPulses2 = 0;
  while(totalPulses2 < CMtoSteps(distance)){
    motor2.run(FORWARD);
    motor4.run(FORWARD);
    updatempu();
  }
  stopMotors();
  delay(2000);

  while(mpu6050.getAngleZ() < init_ang - 1 || mpu6050.getAngleZ() > init_ang + 1){

    if (mpu6050.getAngleZ() < init_ang){

      motor4.setSpeed(180);
      motor4.run(FORWARD);
      updatempu();

    } 
    
    if (mpu6050.getAngleZ() > init_ang){

      motor2.setSpeed(140);
      motor2.run(FORWARD);
      updatempu();

    }

  }

  stopMotors();
  delay(2000);

}

void newMoveForward(float distance){
  
}

void moveBackward(float distance){ //basic ass moving (no encoders)
  double init_time = millis();
  double init_ang = mpu6050.getAngleZ();
  motor2.setSpeed(130); // 140
  motor4.setSpeed(155); // 190
  totalPulses = totalPulses2 = 0;
  while(totalPulses2 < CMtoSteps(distance)){
    motor2.run(REVERSE);
    motor4.run(REVERSE);
    updatempu();
  }
  stopMotors();
  delay(2000);

  while(mpu6050.getAngleZ() < init_ang - 1 || mpu6050.getAngleZ() > init_ang + 1){

    if (mpu6050.getAngleZ() < init_ang){

      motor4.setSpeed(190);
      motor4.run(FORWARD);
      updatempu();

    } 
    
    if (mpu6050.getAngleZ() > init_ang){

      motor2.setSpeed(140);
      motor2.run(FORWARD);
      updatempu();

    }

  }

  stopMotors();
  delay(2000);

}

void interruptFunction() {
  totalPulses++;
}
void interruptFunction2() {
  totalPulses2++; 
}

int CMtoSteps(float cm) {
  int result;  // Final calculation result
  float circumference = (wheeldiameter * 3.1419) / 10; // Calculate wheel circumference in cm
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

void moveFEorward(float distance, int duration){

  float m2s = 120;
  float m4s = 125;
  float init_ang = mpu6050.getAngleZ();
  int init_pulses = totalPulses;
  int init_pulses2 = totalPulses2;
  float init_time = millis();
  int dPulses = CMtoSteps(distance);
  int cPulses;
  int cPulses2;
  float error; 
  while(totalPulses2*Ke < init_pulses2*Ke + dPulses){
    motor2.setSpeed(m2s);
    motor4.setSpeed(m4s);
    motor4.run(FORWARD);
    motor2.run(FORWARD);
    cPulses = totalPulses - init_pulses;
    cPulses2 = totalPulses2 - init_pulses2;
    error = dPulses-(cPulses+cPulses2)/2.0; 
    m4s =map(error*Kp, 0, dPulses*Kp, 100, 140);        
    m2s =map(error*Kp, 0, dPulses*Kp, 100, 140);
    if(cPulses<cPulses2){
      m4s-= (cPulses2-cPulses)*Kc;
    }else{
      m4s+= (cPulses-cPulses2)*Kc;
    }
    //ariv openai plugin
    if(init_ang-5<mpu6050.getAngleZ()){
      motor2.setSpeed(115);
      while(mpu6050.getAngleZ() < init_ang){
        motor2.run(FORWARD);
        updatempu();
      }
    }
    updatempu();
  }
  stopMotors();
  delay(500);
  
}
void oldmoveForward(float distance, int duration) {
  duration = duration * 1000 -500;
  float m2s = 150;
  float m4s = 150;
  float init_ang = mpu6050.getAngleZ();
  int init_pulses = totalPulses;
  int init_pulses2 = totalPulses2;
  int init_time = millis();
  int dPulses = CMtoSteps(distance);
  motor2.setSpeed(m2s);
  motor4.setSpeed(m4s);

  while (millis() < init_time + duration) {
    if (totalPulses < dPulses + init_pulses) {
      motor2.run(FORWARD);
      float curV = stepsToCM(totalPulses - init_pulses) / (millis() - init_time);
      float reqV = (distance - stepsToCM(totalPulses - init_pulses)) / (duration - (millis() - init_time));
      if (curV > reqV) {
        m2s = m2s - Kp;
        motor2.setSpeed(m2s);
      } else if (curV < reqV) {
        m2s = m2s + Kp;
        motor2.setSpeed(m2s);
      }
    } else {
      stopMotors();
    }

    if (totalPulses2 < dPulses + init_pulses2) {
      motor4.run(FORWARD);
      float curV2 = stepsToCM(totalPulses2 - init_pulses2) / (millis() - init_time);
      float reqV2 = (distance - stepsToCM(totalPulses2 - init_pulses2)) / (duration - (millis() - init_time));
      if (curV2 > reqV2) {
        m4s = m4s - Kp;
        motor4.setSpeed(m4s);
      } else if (curV2 < reqV2) {
        m4s = m4s + Kp;
        motor4.setSpeed(m4s);
      }
    } else {
      stopMotors();
    }

    if (mpu6050.getAngleZ()  > init_ang) {
      m4s -= Kc;
      m2s += Kc;
    }
    if (mpu6050.getAngleZ()  < init_ang) {
      m4s += Kc; // Added missing assignment operator
      m2s -= Kc;
    }

    updatempu();
  }

  while(mpu6050.getAngleZ() < init_ang - 1 || mpu6050.getAngleZ() > init_ang + 1){

    if (mpu6050.getAngleZ() < init_ang){

      motor4.setSpeed(120);
      motor4.run(FORWARD);
      updatempu();

    } else {

      motor2.setSpeed(120);
      motor2.run(FORWARD);
      updatempu();

    }

  }
  stopMotors();
  delay(500);
}
void oldTurnLeft() {
  int init_time=millis();
  duration*=1000;
  updatempu();
  motor2.setSpeed(0);
  motor4.setSpeed(140);
  while(millis()<init_time+duration){
    if(mpu6050.getAngleZ() < ref_ang+85){
      motor4.run(FORWARD);
      updatempu();
    }else{
      motor4.setSpeed(0);
    }
  }
  stopMotors();
  
void newturnLeft(){

  updatempu();
  float init_angle = mpu6050.getAngleZ() + 83;
  motor2.setSpeed(0);
  motor4.setSpeed(190);
  while(mpu6050.getAngleZ() < init_angle){
    motor4.run(FORWARD);
    updatempu();
  }
  stopMotors();
  delay(1000);

  mpu6050.begin();

  while(mpu6050.getAngleZ() > 93 || mpu6050.getAngleZ() < 87){

    if (mpu6050.getAngleZ() < 87) {

      motor2.setSpeed(0);
      motor4.setSpeed(190);

      motor4.run(FORWARD);
      updatempu();

    }

    if (mpu6050.getAngleZ() > 93) {

      motor2.setSpeed(120);
      motor4.setSpeed(0);
      
      motor2.run(FORWARD);
      updatempu();

    }

  }

}
void oldTurnRight() {
  // int init_time=millis();
  // duration*=1000;
  // updatempu();
  // motor4.setSpeed(0);
  // motor2.setSpeed(140);
  // while(millis()<init_time+duration){
  //   if(mpu6050.getAngleZ() > ref_ang-88){
  //     motor2.run(FORWARD);
  //     updatempu();
  //   }else{
  //     motor2.setSpeed(0);
  //   }
  // } 
  // stopMotors();
  ref_ang-=90;
  updatempu();
  float init_angle = mpu6050.getAngleZ() - 65;
  motor2.setSpeed(130); // 140
  motor4.setSpeed(0);
  while(mpu6050.getAngleZ() > init_angle){
    motor2.run(FORWARD);
    updatempu();
  }
  stopMotors();
  delay(1000);
}


void stopMotors() {
  motor2.setSpeed(0);
  motor4.setSpeed(0);
}
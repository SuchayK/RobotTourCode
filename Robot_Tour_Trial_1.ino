#include <AFMotor.h>  
#include <PID_v1.h>      
#include <MPU6050_tockn.h>
#include <Wire.h>  


MPU6050 mpu6050(Wire);


AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);


const float wheeldiameter = 66.4;
const float Kp = 0.16;
const float Kc = 0.1;
int encoderPin = 19;
int encoderPin2 = 18;


volatile unsigned long totalPulses = 0;
volatile long totalAngle = 0;

volatile unsigned long ref_ang= 0;
volatile unsigned long totalPulses2 = 0;
volatile long totalAngle2 = 0;


const int anglePerPulse = 0.314;


volatile unsigned long lastTime = 0;
volatile unsigned long currentTime = 0;
volatile double deltaT = 0;

volatile unsigned long lastTime2 = 0;
volatile unsigned long currentTime2 = 0;
volatile double deltaT2 = 0;

volatile double angularVelocity = 0;
volatile double angularVelocity2 = 0;
volatile unsigned long timeAverageAngularVelocity = 0;
volatile unsigned long timeAverageAngularVelocity2 = 0;
volatile int sumPulses = 0;
volatile int sumPulses2 = 0;
volatile int sampleRate = 20;

unsigned long prevTime;
double Input, Output, Setpoint;
double Output2;
double errSum1, errSum2, lastErr1, lastErr2;
double kp = 0.15;
double ki = 0.1;
double kd = 0.1;

// int Kp = 3;
// int Ki = 2;
// int Kd = 1;


// double setPoint, Input, Output;


// PID myPID(&Input, &Output, &setPoint, Kp, Ki, Kd, DIRECT);


void setup() {

  delay(5000);
  Wire.begin();  
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  Serial.begin(9600);

  // pinMode(encoderPin, INPUT);
  // pinMode(encoderPin2, INPUT);

  motor2.run(BRAKE);
  motor4.run(BRAKE);


  attachInterrupt(digitalPinToInterrupt(encoderPin), interruptFunction, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), interruptFunction2, RISING);
  updatempu();
  
  ref_ang = mpu6050.getAngleZ();

  // moveForward(15,3);
  // turnLeft();
  // moveForward(20,4);
  // moveForward(20,4);
  // moveForward(20,4);
  // moveForward(10,2);
  // turnRight();
  // moveForward(15,2);
  // moveForward(15,2);
  // turnRight();
  // moveForward(15,3);
  // turnLeft();
  // moveForward(20,3);
  // moveForward(20,3);
  // moveForward(20,3);
  // moveForward(20,3);
  // turnLeft();
  // moveForward(20,3);

  moveBackward(15, 3);


  lastTime = millis();
  // updatempu();
 
}

void loop() {



  // Serial.println(getAccurateVoltage());
  // delay(1000);
  
  /*
  while (!(mpu6050.getAngleZ() > 175 && mpu6050.getAngleZ() < 180)){
    turnLeft();
  }
  */

  // Serial.print("Angle: ");
  // Serial.println(mpu6050.getAngleZ());
  // updatempu();



}

void Compute(float distance)
{
   while (true){
    /*How long since we last calculated*/
    unsigned long now = millis();
    double timeChange = (double)(now - prevTime) / 1000;

    int init_pulses = totalPulses;
    int dPulses = stepsToCM(init_pulses);

    int init_pulses2 = totalPulses2;
    int dPulses2 = stepsToCM(init_pulses2);

    /*Compute all the working error variables*/
    double error1 = distance - dPulses;
    errSum1 += (error1 * timeChange);
    double dErr1 = (error1 - lastErr1) / timeChange;

    double error2 = distance - dPulses2;
    errSum2 += (error2 - lastErr2) / timeChange;
    double dErr2 = (error2 - lastErr1) / timeChange;

    /*Compute PID Output*/
    Output = kp * error1 + ki * errSum1 + kd * dErr1;

    // Output2 = kp * error2 + ki * errSum2 + kd * dErr2;

    motor2.setSpeed(Output * 5);
    motor4.setSpeed(Output * 5 + 30);

    motor2.run(FORWARD);
    motor4.run(FORWARD);

    Serial.print("Output: ");
    Serial.println(Output * 5);

    // Serial.print("Output 2: ");
    // Serial.println(Output2);

    lastErr1 = error1;
    lastErr2 = error2;
    prevTime = now;

    

    if (Output * 5 < 5){
      break;
    }
   }
   stopMotors();
}

void move(float distance, int duration) {

  Compute(distance);

}

void updatempu() {
  mpu6050.update();
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());
}

int getAccurateVoltage() {
  int i;
  for (i=0; i<10; i++) {
    getVoltage();
  }
  return getVoltage();
}

// Read the voltage of the battery the Arduino is currently running on (in millivolts)
int getVoltage(void) {
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // For mega boards
    const long InternalReferenceVoltage = 1115L;  // Adjust this value to your boards specific internal BG voltage x1000
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0<<MUX5) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #else // For 168/328 boards
    const long InternalReferenceVoltage = 1056L;
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #endif
  //delay(50); // Let mux settle a little to get a more stable A/D conversion
  ADCSRA |= _BV( ADSC ); // Start a conversion 
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) ); // Wait for it to complete
  int results = (((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L; // Scale the value; calculates for straight line value
  return results*10; // convert from centivolts to millivolts
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


void moveForward(float distance, int duration) {
  duration = duration * 1000;
  float m2s = 133.3;
  float m4s = 133.3;
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

  while(mpu6050.getAngleZ() < init_ang - 3 || mpu6050.getAngleZ() > init_ang + 3){

    if (mpu6050.getAngleZ() < init_ang){

      motor4.setSpeed(140);
      motor4.run(FORWARD);
      updatempu();

    } 

    if (mpu6050.getAngleZ() > init_ang) {

      motor2.setSpeed(140);
      motor2.run(FORWARD);
      updatempu();

    }

  }

  stopMotors();
  delay(500);
}

void moveBackward(float distance, int duration) {
  duration = duration * 1000;
  float m2s = 133.3;
  float m4s = 133.3;
  float init_ang = mpu6050.getAngleZ();
  int init_pulses = totalPulses;
  int init_pulses2 = totalPulses2;
  int init_time = millis();
  int dPulses = CMtoSteps(distance);
  motor2.setSpeed(m2s);
  motor4.setSpeed(m4s);

  while (millis() < init_time + duration) {
    if (totalPulses < dPulses + init_pulses) {
      motor2.run(BACKWARD);
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
      motor4.run(BACKWARD);
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

  while(mpu6050.getAngleZ() < init_ang - 3 || mpu6050.getAngleZ() > init_ang + 3){

    if (mpu6050.getAngleZ() < init_ang){

      motor4.setSpeed(140);
      motor4.run(FORWARD);
      updatempu();

    }  
    
    if (mpu6050.getAngleZ() > init_ang) {

      motor2.setSpeed(140);
      motor2.run(FORWARD);
      updatempu();

    }

  }

  stopMotors();
  delay(2000);
}

void turnLeft() {
  // int init_time=millis();
  // duration*=1000;
  // updatempu();
  // motor2.setSpeed(0);
  // motor4.setSpeed(140);
  // while(millis()<init_time+duration){
  //   if(mpu6050.getAngleZ() < ref_ang+85){
  //     motor4.run(FORWARD);
  //     updatempu();
  //   }else{
  //     motor4.setSpeed(0);
  //   }
  // }

  // stopMotors();
  // ref_ang+=90;

  int time = millis();
  updatempu();
  float init_angle = mpu6050.getAngleZ() + 83.5;
  motor2.setSpeed(0);
  motor4.setSpeed(140);
  while(mpu6050.getAngleZ() < init_angle){
    motor4.run(FORWARD);
    updatempu();
  }
  stopMotors();
  delay(1000);

}

void turnRight() {
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
  // ref_ang-=90;

  int time = millis();
  updatempu();
  float init_angle = mpu6050.getAngleZ();
  float new_angle = mpu6050.getAngleZ();
  motor2.setSpeed(140);
  motor4.setSpeed(0);
  while(init_angle - new_angle < 85){
    motor2.run(FORWARD);
    updatempu();
    new_angle = mpu6050.getAngleZ();
  }
  stopMotors();
  delay(1000);

}

long readVcc() {
 
  long result;
    // Read 1.1V reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result = ADCL;
    result |= ADCH<<8;
    result = 1126400L / result; // Back-calculate AVcc in mV
    return result;

}

// void forward() {

//   // === Read accelerometer (on the MPU6050) data === //
//   readAcceleration();
//   // Calculating Roll and Pitch from the accelerometer data
//   accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
//   accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
//   // === Read gyroscope (on the MPU6050) data === //
//   previousTime = currentTime;
//   currentTime = micros();
//   elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
//   readGyro();
//   // Correct the outputs with the calculated error values
//   GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
//   GyroY -= GyroErrorY;
//   GyroZ -= GyroErrorZ;
//   // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
//   gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
//   gyroAngleY += GyroY * elapsedTime;
//   yaw += GyroZ * elapsedTime;
//   //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
//   roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
//   pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
//   angle = pitch; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
//   //for me, turning right reduces angle. Turning left increases angle.

// }




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




// void turnAngle(int angle) {
//   float initialYaw = mpu.getAngleY();
//   float targetYaw = initialYaw + angle;
 
//   while (mpu.getAngleY() < targetYaw) {
//     turnLeft();
//   }


 
//   stopMotors();
// }
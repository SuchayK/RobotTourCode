#include <Encoder.h>

const int motor1PWM = 3;
const int motor1Dir = 4;
const int motor2PWM = 5;
const int motor2Dir = 6;
const int motor3PWM = 9;
const int motor3Dir = 10;
const int motor4PWM = 11;
const int motor4Dir = 12;

const int encoder1A = 2;
const int encoder1B = 7;
const int encoder2A = 8;
const int encoder2B = 13;
const int encoder3A = 18;
const int encoder3B = 19;
const int encoder4A = 20;
const int encoder4B = 21;

Encoder encoder1(encoder1A, encoder1B);
Encoder encoder2(encoder2A, encoder2B);
Encoder encoder3(encoder3A, encoder3B);
Encoder encoder4(encoder4A, encoder4B);

double Kp = 2.0;
double Ki = 0.5;
double Kd = 1.0;

double setpoint1 = 1000;
double setpoint2 = 1000;
double setpoint3 = 1000;
double setpoint4 = 1000;

double input1, input2, input3, input4;
double output1, output2, output3, output4;
double error1, error2, error3, error4;
double lastError1, lastError2, lastError3, lastError4;
double integral1, integral2, integral3, integral4;

void setup() {
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Dir, OUTPUT);
  pinMode(motor3PWM, OUTPUT);
  pinMode(motor3Dir, OUTPUT);
  pinMode(motor4PWM, OUTPUT);
  pinMode(motor4Dir, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  input1 = encoder1.read();
  input2 = encoder2.read();
  input3 = encoder3.read();
  input4 = encoder4.read();

  error1 = setpoint1 - input1;
  error2 = setpoint2 - input2;
  error3 = setpoint3 - input3;
  error4 = setpoint4 - input4;

  integral1 += error1;
  integral2 += error2;
  integral3 += error3;
  integral4 += error4;

  double derivative1 = error1 - lastError1;
  double derivative2 = error2 - lastError2;
  double derivative3 = error3 - lastError3;
  double derivative4 = error4 - lastError4;

  output1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
  output2 = Kp * error2 + Ki * integral2 + Kd * derivative2;
  output3 = Kp * error3 + Ki * integral3 + Kd * derivative3;
  output4 = Kp * error4 + Ki * integral4 + Kd * derivative4;

  double maxError = max(max(error1, error2), max(error3, error4));

  if (maxError == error1) {
    output2 += (error1 - error2) * Kp;
    output3 += (error1 - error3) * Kp;
    output4 += (error1 - error4) * Kp;
  } else if (maxError == error2) {
    output1 += (error2 - error1) * Kp;
    output3 += (error2 - error3) * Kp;
    output4 += (error2 - error4) * Kp;
  } else if (maxError == error3) {
    output1 += (error3 - error1) * Kp;
    output2 += (error3 - error2) * Kp;
    output4 += (error3 - error4) * Kp;
  } else {
    output1 += (error4 - error1) * Kp;
    output2 += (error4 - error2) * Kp;
    output3 += (error4 - error3) * Kp;
  }

  setMotorSpeed(motor1PWM, motor1Dir, output1);
  setMotorSpeed(motor2PWM, motor2Dir, output2);
  setMotorSpeed(motor3PWM, motor3Dir, output3);
  setMotorSpeed(motor4PWM, motor4Dir, output4);

  lastError1 = error1;
  lastError2 = error2;
  lastError3 = error3;
  lastError4 = error4;

  Serial.print("Encoder 1: ");
  Serial.print(input1);
  Serial.print("\tOutput 1: ");
  Serial.print(output1);
  Serial.print("\tEncoder 2: ");
  Serial.print(input2);
  Serial.print("\tOutput 2: ");
  Serial.print(output2);
  Serial.print("\tEncoder 3: ");
  Serial.print(input3);
  Serial.print("\tOutput 3: ");
  Serial.print(output3);
  Serial.print("\tEncoder 4: ");
  Serial.print(input4);
  Serial.print("\tOutput 4: ");
  Serial.println(output4);

  delay(50);
}

void setMotorSpeed(int pwmPin, int dirPin, double speed) {
  if (speed > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, constrain(speed, 0, 255));
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, constrain(-speed, 0, 255));
  }
}

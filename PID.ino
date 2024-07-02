#include <Encoder.h>
#include <PID_v1.h>

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

double Kp = 2.0, Ki = 0.5, Kd = 1.0;

double setpoint1 = 1000, setpoint2 = 1000, setpoint3 = 1000, setpoint4 = 1000;
double input1, input2, input3, input4;
double output1, output2, output3, output4;

PID pid1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);
PID pid3(&input3, &output3, &setpoint3, Kp, Ki, Kd, DIRECT);
PID pid4(&input4, &output4, &setpoint4, Kp, Ki, Kd, DIRECT);

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
  
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);
  pid3.SetMode(AUTOMATIC);
  pid4.SetMode(AUTOMATIC);
}

void loop() {
  input1 = encoder1.read();
  input2 = encoder2.read();
  input3 = encoder3.read();
  input4 = encoder4.read();

  pid1.Compute();
  pid2.Compute();
  pid3.Compute();
  pid4.Compute();

  double maxError = max(max(abs(input1 - setpoint1), abs(input2 - setpoint2)), max(abs(input3 - setpoint3), abs(input4 - setpoint4)));

  if (abs(input1 - setpoint1) == maxError) {
    output2 += (input1 - input2) * Kp;
    output3 += (input1 - input3) * Kp;
    output4 += (input1 - input4) * Kp;
  } else if (abs(input2 - setpoint2) == maxError) {
    output1 += (input2 - input1) * Kp;
    output3 += (input2 - input3) * Kp;
    output4 += (input2 - input4) * Kp;
  } else if (abs(input3 - setpoint3) == maxError) {
    output1 += (input3 - input1) * Kp;
    output2 += (input3 - input2) * Kp;
    output4 += (input3 - input4) * Kp;
  } else {
    output1 += (input4 - input1) * Kp;
    output2 += (input4 - input2) * Kp;
    output3 += (input4 - input3) * Kp;
  }

  setMotorSpeed(motor1PWM, motor1Dir, output1);
  setMotorSpeed(motor2PWM, motor2Dir, output2);
  setMotorSpeed(motor3PWM, motor3Dir, output3);
  setMotorSpeed(motor4PWM, motor4Dir, output4);

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

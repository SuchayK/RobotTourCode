#include <AFMotor.h>  

AF_DCMotor motor1(4);
AF_DCMotor motor3(3);  

void setup() {
  // put your setup code here, to run once:
  motor1.setSpeed(255); 
  motor1.run(FORWARD);
  motor3.setSpeed(255); 
  motor3.run(FORWARD);  
  delay(10000);
  motor1.run(RELEASE);
  motor3.run(RELEASE);
}

void loop() {
  // put your main code here, to run repeatedly:

}

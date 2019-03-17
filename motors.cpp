#include "motors.h"

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;



volatile unsigned int leftEncoderCount = 0;
unsigned long totalLeftEncoderCount = 0;
void leftEncoderInterrupt() {
  leftEncoderCount++;
}

volatile unsigned int rightEncoderCount = 0;
unsigned long totalRightEncoderCount = 0;

void rightEncoderInterrupt() {
  rightEncoderCount++;
}

void motors_init() {
  pinMode(MOT_LEFT_1, OUTPUT);
  pinMode(MOT_LEFT_2, OUTPUT);
  pinMode(MOT_RIGHT_1, OUTPUT);
  pinMode(MOT_RIGHT_2, OUTPUT);
  pinMode(MOT_PWM_LEFT, OUTPUT);
  pinMode(MOT_PWM_RIGHT, OUTPUT);
  digitalWrite(MOT_LEFT_1, LOW);
  digitalWrite(MOT_LEFT_2, LOW);
  digitalWrite(MOT_RIGHT_1, LOW);
  digitalWrite(MOT_RIGHT_2, LOW);
  digitalWrite(MOT_PWM_LEFT, LOW);
  digitalWrite(MOT_PWM_RIGHT, LOW);


  attachInterrupt(digitalPinToInterrupt(MOT_ENCODER_LEFT), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOT_ENCODER_RIGHT), rightEncoderInterrupt, CHANGE);


}

void motors_recordSpeed() {//verify if 168 is correct to obtain mm/s
  leftMotorSpeed  = (168 * leftEncoderCount)  / ENCODER_SPEED_SAMPLE_INTERVAL;
  rightMotorSpeed = (168 * rightEncoderCount) / ENCODER_SPEED_SAMPLE_INTERVAL;
  totalLeftEncoderCount += leftEncoderCount;
  totalRightEncoderCount += rightEncoderCount;
  /*Serial.print(leftEncoderCount);
  Serial.print(" ");
  Serial.println(rightEncoderCount);*/
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
}




void motors_setPower(int leftSpeed, int rightSpeed) {
  digitalWrite(MOT_RIGHT_1, rightSpeed >= 0);
  digitalWrite(MOT_RIGHT_2, rightSpeed < 0);
  digitalWrite(MOT_LEFT_1, leftSpeed >= 0);
  digitalWrite(MOT_LEFT_2, leftSpeed < 0);
  analogWrite(MOT_PWM_LEFT, abs(leftSpeed));
  analogWrite(MOT_PWM_RIGHT, abs(rightSpeed));
}

void motors_break() {
  digitalWrite(MOT_LEFT_1, LOW);
  digitalWrite(MOT_LEFT_2, LOW);
  digitalWrite(MOT_RIGHT_1, LOW);
  digitalWrite(MOT_RIGHT_2, LOW);
  digitalWrite(MOT_PWM_LEFT, LOW);
  digitalWrite(MOT_PWM_RIGHT, LOW);
}



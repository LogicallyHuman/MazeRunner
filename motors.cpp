#include "motors.h"

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

volatile unsigned int leftEncoderCount = 0;
unsigned long totalLeftEncoderCount = 0;
void leftEncoderInterrupt() {
    leftEncoderCount += 2;
}

volatile unsigned int rightEncoderCount = 0;
unsigned long totalRightEncoderCount = 0;

void rightEncoderInterrupt() {
    rightEncoderCount += 2;
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

void motors_recordSpeed() {  //
    leftMotorSpeed = 10 * leftEncoderCount;
    rightMotorSpeed = 10 * rightEncoderCount;
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
    digitalWrite(MOT_LEFT_1, HIGH);
    digitalWrite(MOT_LEFT_2, HIGH);
    digitalWrite(MOT_RIGHT_1, HIGH);
    digitalWrite(MOT_RIGHT_2, HIGH);
    digitalWrite(MOT_PWM_LEFT, HIGH);
    digitalWrite(MOT_PWM_RIGHT, HIGH);
}

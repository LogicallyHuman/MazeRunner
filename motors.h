#ifndef MOTORS_H
#define MOTORS_H


#include <Arduino.h>

#define MAX_MOTOR_POWER 100
#define ENCODER_SPEED_SAMPLE_INTERVAL 30 //ms

#define MOT_LEFT_1     5
#define MOT_LEFT_2     4
#define MOT_RIGHT_1    8
#define MOT_RIGHT_2    7
#define MOT_PWM_LEFT   6
#define MOT_PWM_RIGHT  9

#define MOT_ENCODER_LEFT 3
#define MOT_ENCODER_RIGHT 2

void motors_recordSpeed();

extern int leftMotorSpeed;
extern int rightMotorSpeed;

extern volatile unsigned int leftEncoderCount;
extern unsigned long totalLeftEncoderCount;
void leftEncoderInterrupt();

extern volatile unsigned int rightEncoderCount;
extern unsigned long totalRightEncoderCount;
void rightEncoderInterrupt(); 

void motors_init();

void motors_setPower(int leftSpeed, int rightSpeed);//Sets power delivered to motors

void motors_break();


#endif

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "sensors.h"

#define MAX_MOTOR_POWER 200             //Limit motor power for safety reasons.

#define Ki  0.04  //PID Values for keeping robot at a constant linear velocity. Encoder based.
#define Kp  0.07
#define Kd  0

#define Kig  0  //PID Values for keeping robot straight.  Gyroscope based.
#define Kpg  0.1
#define Kdg  0.08


#define Ki_rot   0.04  //PID Values for keeping robot turning at a constant angular velocity. Encoder based.
#define Kp_rot  0.07
#define Kd_rot  0

#define Ki_diff   0  //PID Values for keeping robot turing on its central axis. Encoder based.
#define Kp_diff  0.1
#define Kd_diff  0.08

enum states{FORWARD, TURNING};

#define TURNING_SPEED 6000 //Maximum angular velocity reached. TODO: Make the velocity setable by rotate function.
#define TURNING_ACCEL 300  //TODO: Implementation of accel. won't work for deacceleration, it needs to be based on angle position.

#define ACCEL 4000  //Acceleration and deacceleration for linear movement.

#define INTEGRAL_LIMIT 500 //TODO: Implement correct integral windup correction.

extern unsigned long forwardSpeedIntegral;
extern unsigned long forwardDifferenceIntegral;
extern unsigned long rotatingDifferenceIntegral;
extern unsigned long rotatingSpeedIntegral;


extern int prevForwardSpeedError;
extern int prevForwardDifferenceError;
extern int prevRotatingSpeedError;
extern int prevRotatingDifferenceError;

extern char robotState;

extern int longTermSpeedTarget;
extern int shortTermSpeedTarget;

extern int longTermRotateSpeedTarget;
extern int shortTermRotateSpeedTarget;

extern long ticksTarget;

extern char movementState;

void updatePID();
void resetAngle();
void forward(int robotSpeed);//Moves robot forward in a straight line at a constant speed
void forwardFor(int robotSpeed, int distance);
void forwardUntil(int robotSpeed, int distanceToWall);
void turnFor(int angle);

#endif


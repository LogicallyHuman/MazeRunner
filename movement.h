#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "sensors.h"
#include "leds.h"

#define MAX_MOTOR_POWER 200             //Limit motor power for safety reasons.

#define Ki  0.02  //PID Values for keeping robot at a constant linear velocity. Encoder based.
#define Kp  0.04
#define Kd  0

#define Kig  0  //PID Values for keeping robot straight.  Gyroscope based.
#define Kpg  0.1
#define Kdg  0.5


#define Ki_rot   0.02  //PID Values for keeping robot turning at a constant angular velocity. Encoder based.
#define Kp_rot  0.04
#define Kd_rot  0

#define Ki_diff   0  //PID Values for keeping robot turing on its central axis. Encoder based.
#define Kp_diff  0.03
#define Kd_diff  0.03

enum states{FORWARD, TURNING};
enum rotatingStates{ROT_ACCEL_STATE, ROT_CONST_STATE, ROT_DEACCEL_STATE};


#define TURNING_START_SPEED 150L
#define TURNING_SPEED 300L //Maximum angular velocity reached. TODO: Make the velocity setable by rotate function.
#define TURNING_ACCEL 500L  //Distance to travel accelerating and distance to travel deaccelerating.
//HIGHER TURNING_ACCEL IMPLIES SLOWER ACCELERATION. LOWER TURNING_ACCEL FASTER ACCELERATION
//
//   ANG VEL
//    /|\ 
//     |      _________________
//     |     /.               .\ 
//     |    / .               . \ 
//     |   /  .               .  \  
//     |  /   .               .   \ 
//     | /    .               .    \ 
//START|/     .               .     \ 
//     |._____._______________._____._______->  ANGULAR DISTANCE TRAVELED
//      .     .               .     .
//      .ACCEL.               .ACCEL.
//    start                        goal
#define ACCEL 100  //Acceleration and deacceleration for linear movement.

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

void updatePID(int * readings);
void resetAngle();
void forward(int robotSpeed);//Moves robot forward in a straight line at a constant speed
void forwardFor(int robotSpeed, int distance);
void forwardUntil(int robotSpeed, int distanceToWall);
void turnFor(int angle);

#endif


#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "sensors.h"

#define MAX_MOTOR_POWER 200
#define MAX_MOTOR_TURNING_POWER 150

#define Ki  0.04
#define Kp  0.07
#define Kd  0

#define Kig  0
#define Kpg  0.1
#define Kdg  0.08


#define Ki_rot   0.04
#define Kp_rot  0.07
#define Kd_rot  0

#define Ki_diff   0
#define Kp_diff  0.1
#define Kd_diff  0.08


#define STRAIGHT 1
#define TURNING 2

#define TURNING_SPEED 6000
#define TURNING_ACCEL 300 

#define ACCEL 4000


#define INTEGRAL_LIMIT 500

extern unsigned long integral;
extern unsigned long angleIntegral;
extern int prevError;
extern int prevAngleError;
extern long rotateIntegral;
extern int angleTarget;

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


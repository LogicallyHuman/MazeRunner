#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

#define Ki  0.04
#define Kp  0.07
#define Kd  0

#define Kig  0
#define Kpg  0.3
#define Kdg  0.2

#define Ki_rot   0.1
#define Kpg_rot  0.2
#define Kdg_rot  0


#define STOP 0
#define FORWARD 1
#define ROTATING 2
#define FORWARD_AND_STOP 3

#define ROTATE_SPEED 90 //degrees/sec
#define ROTATE_ACCEL 900 //degrees/sec/sec

#define ACCEL 2000


#define INTEGRAL_LIMIT 500

extern long integral;
extern long angleIntegral;
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


void updatePID();
void forward(int robotSpeed);//Moves robot forward in a straight line at a constant speed
void forwardFor(int robotSpeed, int distance);
void turn(int rotateSpeed, int angle);

#endif


#include "movement.h"
#include "gyro.h"
#include "motors.h"

int prevForwardSpeedError;
int prevForwardDifferenceError;
int prevRotatingSpeedError;
int prevRotatingDifferenceError;

long forwardSpeedIntegral = 0;
long forwardDifferenceIntegral = 0;
long rotatingDifferenceIntegral = 0;
long rotatingSpeedIntegral = 0;

char movementState = MOTORS_FORWARD;
char rotatingState;

int longTermForwardSpeedTarget = 0;
int shortTermForwardSpeedTarget = 0;
int shortTermRotatingSpeedTarget = 0;

int angleOffset = 0;

int idealSumOfSideDistance;

void resetAngle() {  //TODO: Move to gyro.cpp
    angleOffset = gyro_getAngle();
}

long rotateTicksTarget;  //TODO: Implement functionality.
long rotateTicksTraveled = 0;

#define LEFT_SIDE 0
#define RIGHT_SIDE 1
bool side = RIGHT_SIDE;

int desiredAngle = 0;
#define CW 0
#define CCW 1

char rotateDirection;

char prevCondition = true;
long refoundWallMillis = 0;

char juan = 0;

void updatePID(volatile int* readings, volatile char ignoreSensors) {  //THIS FUNCTION NEEDS TO BE CALLED PERIODICALLY. TODO: Maybe use a timer to call it.
    motors_recordSpeed();                            //Records motor speed, to leftMotorSpeed and rightMotorSpeed

    
            if(juan == 100){
                juan = 0;
                side = !side;
            }

    if (movementState == MOTORS_FORWARD) {
        if (longTermForwardSpeedTarget > 0) {                                //If desired speed > 0, adjust short term value based on acceleration
            if (shortTermForwardSpeedTarget < longTermForwardSpeedTarget) {  //Update shortTermForwardSpeedTarget based on current longTermForwardSpeedTarget and ACCEL
                shortTermForwardSpeedTarget += ((long)ACCEL);
                if (shortTermForwardSpeedTarget > longTermForwardSpeedTarget) {
                    shortTermForwardSpeedTarget = longTermForwardSpeedTarget;
                }
            } else if (shortTermForwardSpeedTarget > longTermForwardSpeedTarget) {
                shortTermForwardSpeedTarget -= ((long)ACCEL);
                if (shortTermForwardSpeedTarget < longTermForwardSpeedTarget) {
                    shortTermForwardSpeedTarget = longTermForwardSpeedTarget;
                }
            }
        } else {  //If desired speed is 0, break motors.
            shortTermForwardSpeedTarget = 0;
            motors_break();
        }

        if (shortTermForwardSpeedTarget > 0) {  //If motors didn't break, or current short term desired speed is > 0

            int modifiedShortTermForwardSpeedTarget;
            int sideDifference;
            int static prevSideDifference = 0;
            int static prevLSensor = 0;
            int static prevRSensor = 0;
            char condition = (readings[0] < 160 && readings[4] < 160 && (readings[0] - prevLSensor < 20) && (readings[4] - prevRSensor < 20));
            if (!prevCondition && condition) {
                refoundWallMillis = millis();
            } else if (!condition && prevCondition) {
                refoundWallMillis = 0;
            }
            prevCondition = condition;
            if (millis() > refoundWallMillis + 170 && condition) {
                sideDifference = (readings[0] - readings[4]);
                //if (readings[0] + readings[4] > idealSumOfSideDistance + 1000 || abs(readings[0] - readings[4]) > 5000) {
                //    modifiedShortTermForwardSpeedTarget = max(shortTermForwardSpeedTarget - 8 * (readings[0] + readings[4] - idealSumOfSideDistance) - 3 * abs(readings[0] - readings[4]), 100);
                //} else {
                    modifiedShortTermForwardSpeedTarget = shortTermForwardSpeedTarget;
                //}
            } else {
                sideDifference = 0;
                modifiedShortTermForwardSpeedTarget = shortTermForwardSpeedTarget;
            }

            prevLSensor = readings[0];
            prevRSensor = readings[4];
            //Speed control PID loop
            int error = modifiedShortTermForwardSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;
            if (abs(forwardSpeedIntegral + error) < 255 / Ki)
                forwardSpeedIntegral += error;
            int power = Kp * error + Kd * (error - prevForwardSpeedError) + Ki * forwardSpeedIntegral;
            prevForwardSpeedError = error;

            //Angle correction PID loop

            int static angleError = 0;

            angleError = 0.1 * (angleError - gyro_getAngle()) + 0.9 * (sideDifference);
            //angleError = sideDifference;

            //if (abs(forwardDifferenceIntegral + angleError) < 255 / Kig)
            //forwardDifferenceIntegral += angleError;
            int anglePower = Kpg * angleError + Kdg * (angleError - prevForwardDifferenceError) + 0.01 * forwardDifferenceIntegral;
            prevForwardDifferenceError = angleError;

            //Calculate motor powers.
            int lpower = power - anglePower;
            int rpower = power + anglePower;

            if (lpower > MAX_MOTOR_POWER)  //TODO: Move power limit to motors.cpp
                lpower = MAX_MOTOR_POWER;
            else if (lpower < -MAX_MOTOR_POWER)
                lpower = -MAX_MOTOR_POWER;

            if (rpower > MAX_MOTOR_POWER)
                rpower = MAX_MOTOR_POWER;
            else if (rpower < -MAX_MOTOR_POWER)
                rpower = -MAX_MOTOR_POWER;

            motors_setPower(lpower, rpower);  //Update motor powers.
        }

    } else if (movementState == MOTORS_TURNING) {

        if (shortTermForwardSpeedTarget > 0) {  //If motors didn't break, or current short term desired speed is > 0

            int sideDifference;
            int static prevSideDifference = 0;
            int static prevLSensor = 0;
            int static prevRSensor = 0;

            //Speed control PID loop
            int error = 600 - (leftMotorSpeed + rightMotorSpeed) / 2;
            if (abs(forwardSpeedIntegral + error) < 255 / Ki)
                forwardSpeedIntegral += error;
            int power = Kp * error + Kd * (error - prevForwardSpeedError) + Ki * forwardSpeedIntegral;
            prevForwardSpeedError = error;

            //Angle correction PID loop
            int angleError = 0;
            static int prevSide;

            if(side == RIGHT_SIDE) angleError = (leftMotorSpeed - rightMotorSpeed) - 400;  //Tunear der MOT_DIF + -250
            if(side == LEFT_SIDE) angleError = (leftMotorSpeed - rightMotorSpeed) + 400;
            if(side != prevSide)forwardDifferenceIntegral = 0;
            prevSide = side;
            //angleError = (leftMotorSpeed - rightMotorSpeed);
            forwardDifferenceIntegral += angleError;
            int anglePower = 0 * angleError + Kd_rot * (angleError - prevForwardDifferenceError) + 0.01 * forwardDifferenceIntegral;
            Serial.print(leftMotorSpeed);
            Serial.print(" ");
            Serial.println(rightMotorSpeed);
            prevForwardDifferenceError = angleError;

            //Calculate motor powers.
            int lpower;
            int rpower;

            //if(side == RIGHT_SIDE){
          //      lpower = power - anglePower;
           //     rpower = power + anglePower;
            //}
            //if(side == LEFT_SIDE) {
                lpower = power - anglePower;
                rpower = power + anglePower;
//            }

            if (lpower > MAX_MOTOR_POWER)  //TODO: Move power limit to motors.cpp
                lpower = MAX_MOTOR_POWER;
            else if (lpower < -MAX_MOTOR_POWER)
                lpower = -MAX_MOTOR_POWER;

            if (rpower > MAX_MOTOR_POWER)
                rpower = MAX_MOTOR_POWER;
            else if (rpower < -MAX_MOTOR_POWER)
                rpower = -MAX_MOTOR_POWER;

            motors_setPower(lpower, rpower);  //Update motor powers.
        }

    } else {
        movementState = MOTORS_FORWARD;
    }
}

//Set movement state to MOTORS_FORWARD. Tries to mantain a constant forward speed.
//Accelerates linearly at ACCEL rate until robotSpeed speed is reached.
void forward(int robotSpeed) {
    longTermForwardSpeedTarget = robotSpeed;
    forwardDifferenceIntegral = 0;
    movementState = MOTORS_FORWARD;
}

//Rotates on axis. TODO: Implement correct acceletation and deacceleration. And stop when angle is reached.
void turnFor(int angle) {
    movementState = MOTORS_TURNING;
    rotatingState = ROT_ACCEL_STATE;
    desiredAngle = angle * 10;
    gyro_reset();
    rotatingDifferenceIntegral = 0;
    rotatingSpeedIntegral = 0;
    rotateTicksTarget = abs(angle * 20);  //Magic degrees to (lmotorticks + rmotorticls)/2 proportionality constant.
    if (angle > 0) {
        rotateDirection = CCW;
    } else {
        rotateDirection = CW;
    }
    totalLeftEncoderCount = 0;
    totalRightEncoderCount = 0;
    rotateTicksTraveled = 0;
}

void turnFor2() {
    forwardDifferenceIntegral = 0;
    movementState = MOTORS_TURNING;
    gyro_reset();
    rotatingDifferenceIntegral = 0;
    rotatingSpeedIntegral = 0;
    rotateTicksTarget = 40000;
    rotateTicksTraveled = 0;
}

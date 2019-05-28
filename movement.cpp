#include "gyro.h"
#include "movement.h"
#include "motors.h"

#define FORWARD 1
#define ROTATING 2

int prevForwardSpeedError;
int prevForwardDifferenceError;
int prevRotatingSpeedError;
int prevRotatingDifferenceError;

unsigned long forwardSpeedIntegral = 0;
unsigned long forwardDifferenceIntegral = 0;
unsigned long rotatingDifferenceIntegral = 0;
unsigned long rotatingSpeedIntegral = 0;

char movementState = FORWARD;

int longTermForwardSpeedTarget = 0;
int shortTermForwardSpeedTarget = 0;
int shortTermRotatingSpeedTarget = 0;

int angleOffset = 0;

void resetAngle() {  //TODO: Move to gyro.cpp
    angleOffset = gyro_getAngle();
}

int angleTarget;  //TODO: Implement functionality.

void updatePID() {         //THIS FUNCTION NEEDS TO BE CALLED PERIODICALLY. TODO: Maybe use a timer to call it.
    motors_recordSpeed();  //Records motor speed, to leftMotorSpeed and rightMotorSpeed

    if (movementState == FORWARD) {
        if (longTermForwardSpeedTarget > 0) {                                //If desired speed > 0, adjust short term value based on acceleration
            if (shortTermForwardSpeedTarget < longTermForwardSpeedTarget) {  //Update shortTermForwardSpeedTarget based on current longTermForwardSpeedTarget and ACCEL
                shortTermForwardSpeedTarget += ((long)ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
                if (shortTermForwardSpeedTarget > longTermForwardSpeedTarget) {
                    shortTermForwardSpeedTarget = longTermForwardSpeedTarget;
                }
            } else if (shortTermForwardSpeedTarget > longTermForwardSpeedTarget) {
                shortTermForwardSpeedTarget -= ((long)ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
                if (shortTermForwardSpeedTarget < longTermForwardSpeedTarget) {
                    shortTermForwardSpeedTarget = longTermForwardSpeedTarget;
                }
            }
        } else {  //If desired speed is 0, break motors.
            shortTermForwardSpeedTarget = 0;
            motors_break();
        }

        if (shortTermForwardSpeedTarget > 0) { //If motors didn't break, or current short term desired speed is > 0

            //Speed control PID loop
            int error = shortTermForwardSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;
            if (abs(forwardSpeedIntegral + error) < 255 / Ki)
                forwardSpeedIntegral += error;
            int power = Kp * error + Kd * (error - prevForwardSpeedError) + Ki * forwardSpeedIntegral;
            prevForwardSpeedError = error;


            //Ange correction PID loop
            int angleError = (-gyro_getAngle() + angleOffset);
            if (abs(forwardDifferenceIntegral + angleError) < 255 / Kig)
                forwardDifferenceIntegral += angleError;
            int anglePower = Kpg * angleError + Kdg * (angleError - prevForwardDifferenceError) + Kig * forwardDifferenceIntegral;
            prevForwardDifferenceError = angleError;

            //Calculate motor powers.
            int lpower = power - anglePower;
            int rpower = power + anglePower;

            if (lpower > MAX_MOTOR_POWER) //TODO: Move power limit to motors.cpp
                lpower = MAX_MOTOR_POWER;
            else if (lpower < -MAX_MOTOR_POWER)
                lpower = -MAX_MOTOR_POWER;

            if (rpower > MAX_MOTOR_POWER)
                rpower = MAX_MOTOR_POWER;
            else if (rpower < -MAX_MOTOR_POWER)
                rpower = -MAX_MOTOR_POWER;

            motors_setPower(lpower, rpower);//Update motor powers.
        }

    } else if (movementState == TURNING) {

        if (shortTermRotatingSpeedTarget < TURNING_SPEED) { //Update short trem rotation speed based on TURNING_ACCEL
            shortTermRotatingSpeedTarget += ((long)TURNING_ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
            if (shortTermRotatingSpeedTarget > TURNING_SPEED) {
                shortTermRotatingSpeedTarget = TURNING_SPEED;
            }
        } else if (shortTermRotatingSpeedTarget > TURNING_SPEED) {
            shortTermRotatingSpeedTarget -= ((long)TURNING_ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
            if (shortTermRotatingSpeedTarget < TURNING_SPEED) {
                shortTermRotatingSpeedTarget = TURNING_SPEED;
            }
        }

        //Total motor velocity PID loop
        int angularSpeedError = shortTermRotatingSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;
        if (abs(rotatingSpeedIntegral + angularSpeedError) < 255 / Ki_rot)
            rotatingSpeedIntegral += angularSpeedError;
        int totalPower = Kp_rot * angularSpeedError + Kd_rot * (angularSpeedError - prevRotatingSpeedError) + Ki_rot * rotatingSpeedIntegral;
        prevRotatingSpeedError = angularSpeedError;


        //Same motor speed correction PID loop
        int motorDifferenceError = leftMotorSpeed - rightMotorSpeed;
        if (abs(rotatingDifferenceIntegral + motorDifferenceError) < 255 / Kig)
            rotatingDifferenceIntegral += motorDifferenceError;
        int differencePower = Kp_diff * motorDifferenceError + Kd_diff * (motorDifferenceError - prevRotatingDifferenceError) + Ki_diff * rotatingDifferenceIntegral;
        prevRotatingDifferenceError = motorDifferenceError;

        //Calculate motor powers
        int lpower = -totalPower + differencePower;
        int rpower =  totalPower - differencePower;

        if (lpower > MAX_MOTOR_POWER) //TODO: Move to motors.cpp
            lpower = MAX_MOTOR_POWER;
        else if (lpower < -MAX_MOTOR_POWER)
            lpower = -MAX_MOTOR_POWER;

        if (rpower > MAX_MOTOR_POWER)
            rpower = MAX_MOTOR_POWER;
        else if (rpower < -MAX_MOTOR_POWER)
            rpower = -MAX_MOTOR_POWER;

        motors_setPower(lpower, rpower);//Set motor powers.
    }
}

//Set movement state to FORWARD. Tries to mantain a constant forward speed.
//Accelerates linearly at ACCEL rate until robotSpeed speed is reached.
void forward(int robotSpeed) {
    longTermForwardSpeedTarget = robotSpeed;
    movementState = FORWARD;
}

//Rotates on axis. TODO: Implement correct acceletation and deacceleration. And stop when angle is reached.
void turnFor(int angle) {
    movementState = TURNING;
    resetAngle();
    rotatingDifferenceIntegral = 0;
    rotatingSpeedIntegral = 0;
    angleTarget = angle * 10;
    totalLeftEncoderCount = 0;
    totalRightEncoderCount = 0;
}


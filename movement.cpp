#include "movement.h"
#include "gyro.h"
#include "motors.h"

int prevForwardSpeedError;
int prevForwardDifferenceError;
int prevRotatingSpeedError;
int prevRotatingDifferenceError;

unsigned long forwardSpeedIntegral = 0;
unsigned long forwardDifferenceIntegral = 0;
unsigned long rotatingDifferenceIntegral = 0;
unsigned long rotatingSpeedIntegral = 0;

char movementState = FORWARD;
char rotatingState;

int longTermForwardSpeedTarget = 0;
int shortTermForwardSpeedTarget = 0;
int shortTermRotatingSpeedTarget = 0;

int angleOffset = 0;

void resetAngle() {  //TODO: Move to gyro.cpp
    angleOffset = gyro_getAngle();
}

long rotateTicksTarget;  //TODO: Implement functionality.
long rotateTicksTraveled = 0;

int desiredAngle = 0;

void updatePID(int * readings) {         //THIS FUNCTION NEEDS TO BE CALLED PERIODICALLY. TODO: Maybe use a timer to call it.
    motors_recordSpeed();  //Records motor speed, to leftMotorSpeed and rightMotorSpeed

    if (movementState == FORWARD) {
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

            //Speed control PID loop
            int error = shortTermForwardSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;
            if (abs(forwardSpeedIntegral + error) < 255 / Ki)
                forwardSpeedIntegral += error;
            int power = Kp * error + Kd * (error - prevForwardSpeedError) + Ki * forwardSpeedIntegral;
            prevForwardSpeedError = error;

            //Angle correction PID loop
            int sideDifference;
            int static angleError = 0;
            if(readings[0] < 250 && readings[4] < 250){
                sideDifference = readings[0] - readings[4];
                digitalWrite(LED1, 1);
            }
            else{
                sideDifference = 0;
                digitalWrite(LED1, 0);

            }

            angleError = 0.05*(angleError - gyro_getAngle()) + 0.95*sideDifference;


            if (abs(forwardDifferenceIntegral + angleError) < 255 / Kig)
                forwardDifferenceIntegral += angleError;
            int anglePower = Kpg * angleError + Kdg * (angleError - prevForwardDifferenceError) + Kig * forwardDifferenceIntegral;
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

    } else if (movementState == TURNING) {
        rotateTicksTraveled += (leftMotorSpeed + rightMotorSpeed) / 2;

        if (rotateTicksTraveled < rotateTicksTarget) {
            if (rotateTicksTraveled < TURNING_ACCEL) {
                shortTermRotatingSpeedTarget = (rotateTicksTraveled * (TURNING_SPEED)) / TURNING_ACCEL;
            } else if (rotateTicksTarget - rotateTicksTraveled < TURNING_ACCEL) {
                shortTermRotatingSpeedTarget = TURNING_SPEED + ((rotateTicksTarget - rotateTicksTraveled - TURNING_ACCEL) * (TURNING_SPEED)) / TURNING_ACCEL;
            } else {
                shortTermRotatingSpeedTarget = TURNING_SPEED;
            }
            if(shortTermRotatingSpeedTarget < TURNING_START_SPEED)
                shortTermRotatingSpeedTarget = TURNING_START_SPEED;

            /*
            #define TURNING_ACCEL2         400
            #define TURNING_SPEED2         100
            #define TURNING_START_SPEED2   100
            
            int rotationError = desiredAngle - gyro_getAngle();
            int static prevRotationError;
            long static rotationIntegralError;
            char static startedPID = 0;

            if (gyro_getAngle() < TURNING_ACCEL2 && !startedPID){
                shortTermRotatingSpeedTarget = (rotateTicksTraveled*(TURNING_SPEED2))/TURNING_ACCEL2;
                shortTermRotatingSpeedTarget += TURNING_START_SPEED;
            }
            else if(desiredAngle - gyro_getAngle() < TURNING_ACCEL2 || startedPID){
                rotationIntegralError += rotationError;
                shortTermRotatingSpeedTarget = 3*rotationError + 0.05*(rotationError - prevRotationError) + 0.01*rotationIntegralError;
                if(shortTermRotatingSpeedTarget > TURNING_SPEED2 + TURNING_START_SPEED2){
                    shortTermRotatingSpeedTarget = TURNING_SPEED2 + TURNING_START_SPEED2;
                }
                prevRotationError = rotationError;
                startedPID = 1;
            }
            else{
                shortTermRotatingSpeedTarget = TURNING_SPEED2;
                shortTermRotatingSpeedTarget += TURNING_START_SPEED2;
            }*/

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
            int rpower = totalPower - differencePower;

            if (lpower > MAX_MOTOR_POWER)  //TODO: Move to motors.cpp
                lpower = MAX_MOTOR_POWER;
            else if (lpower < -MAX_MOTOR_POWER)
                lpower = -MAX_MOTOR_POWER;

            if (rpower > MAX_MOTOR_POWER)
                rpower = MAX_MOTOR_POWER;
            else if (rpower < -MAX_MOTOR_POWER)
                rpower = -MAX_MOTOR_POWER;

            motors_setPower(lpower, rpower);  //Set motor powers.
        } else {
            movementState = FORWARD;
        }
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
    rotatingState = ROT_ACCEL_STATE;
    desiredAngle = angle * 10;
    gyro_reset();
    rotatingDifferenceIntegral = 0;
    rotatingSpeedIntegral = 0;
    rotateTicksTarget = angle * 20;  //Magic degrees to (lmotorticks + rmotorticls)/2 proportionality constant.
    totalLeftEncoderCount = 0;
    totalRightEncoderCount = 0;
    rotateTicksTraveled = 0;
}

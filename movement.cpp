#include "movement.h"
#include "motors.h"
#include "gyro.h"

#define FORWARD 1
#define ROTATING 2

int prevError;
int prevAngleError;
int prevAngularSpeedError;
int prevMotorDifferenceError;

unsigned long integral = 0;
unsigned long angleIntegral = 0;
unsigned long motorDifferenceIntegral = 0;
unsigned long angularSpeedIntegral = 0;



char movementState = FORWARD;



int longTermSpeedTarget = 0;
int shortTermSpeedTarget = 0;
int shortTermAngularSpeedTarget = 0;

int angleOffset = 0;

void resetAngle(){
  angleOffset = gyro_getAngle();
  angleIntegral = 0;
  integral = 0;
}


int angleTarget;

int RticksCount = 0;
int LticksCount = 0;


void updatePID() {
  motors_recordSpeed();

  if(movementState == FORWARD){
  if(longTermSpeedTarget > 0){
    if (shortTermSpeedTarget < longTermSpeedTarget) { //Update shortTermSpeedTarget based on current longTermSpeedTarget and ACCEL
      shortTermSpeedTarget += ((long)ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
      if (shortTermSpeedTarget > longTermSpeedTarget) {
        shortTermSpeedTarget = longTermSpeedTarget;
      }
    }
    else if (shortTermSpeedTarget > longTermSpeedTarget) {
      shortTermSpeedTarget -= ((long)ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
      if (shortTermSpeedTarget < longTermSpeedTarget) {
        shortTermSpeedTarget = longTermSpeedTarget;
      }
    }
  }
  else {
    shortTermSpeedTarget = 0;
    motors_break();
  }
  if(shortTermSpeedTarget > 0){
      int error = shortTermSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;

      if (abs(integral + error) < 255 / Ki)
        integral += error;
      int power = Kp * error + Kd * (error - prevError) + Ki * integral;
      prevError = error;

      int angleError = (-gyro_getAngle() + angleOffset);


      if (abs(angleIntegral + angleError) < 255 / Kig)
        angleIntegral += angleError;
      int anglePower = Kpg * angleError + Kdg * (angleError - prevAngleError) + Kig * angleIntegral;
      prevAngleError = angleError;
      Serial.print(power);
      Serial.print(" ");
      Serial.println(error);

      int lpower = power - anglePower;
      int rpower = power + anglePower;

      if (lpower > MAX_MOTOR_POWER)lpower = MAX_MOTOR_POWER;
      else if (lpower < -MAX_MOTOR_POWER)lpower = -MAX_MOTOR_POWER;

      if (rpower > MAX_MOTOR_POWER)rpower = MAX_MOTOR_POWER;
      else if (rpower < -MAX_MOTOR_POWER)rpower = -MAX_MOTOR_POWER;

      motors_setPower(lpower, rpower);

    }
  }
  else if(movementState == TURNING){



      if (shortTermAngularSpeedTarget < TURNING_SPEED) {
        shortTermAngularSpeedTarget += ((long)TURNING_ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
        if (shortTermAngularSpeedTarget > TURNING_SPEED) {
          shortTermAngularSpeedTarget = TURNING_SPEED;
        }
      }
      else if (shortTermAngularSpeedTarget > TURNING_SPEED) {
        shortTermAngularSpeedTarget -= ((long)TURNING_ACCEL * ENCODER_SPEED_SAMPLE_INTERVAL) / 1000;
        if (shortTermAngularSpeedTarget < TURNING_SPEED) {
          shortTermAngularSpeedTarget = TURNING_SPEED;
        }
      }
    
      int angularSpeedError = shortTermAngularSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;

      if (abs(angularSpeedIntegral + angularSpeedError) < 255 / Ki_rot)
        angularSpeedIntegral += angularSpeedError;
      int totalPower = Kp_rot * angularSpeedError + Kd_rot * (angularSpeedError - prevAngularSpeedError) + Ki_rot * angularSpeedIntegral;
      prevAngularSpeedError = angularSpeedError;

      int motorDifferenceError = leftMotorSpeed - rightMotorSpeed;

      if (abs(motorDifferenceIntegral + motorDifferenceError) < 255 / Kig)
        motorDifferenceIntegral += motorDifferenceError;
      int differencePower = Kp_diff * motorDifferenceError + Kd_diff * (motorDifferenceError - prevMotorDifferenceError) + Ki_diff * motorDifferenceIntegral;
      prevMotorDifferenceError = motorDifferenceError;
 

      int lpower = -totalPower + differencePower;
      int rpower = totalPower - differencePower;

      if (lpower > MAX_MOTOR_POWER)lpower = MAX_MOTOR_POWER;
      else if (lpower < -MAX_MOTOR_POWER)lpower = -MAX_MOTOR_POWER;

      if (rpower > MAX_MOTOR_POWER)rpower = MAX_MOTOR_POWER;
      else if (rpower < -MAX_MOTOR_POWER)rpower = -MAX_MOTOR_POWER;

      motors_setPower(lpower, rpower);
  }
}

void forward(int robotSpeed) {
  longTermSpeedTarget = robotSpeed;
  movementState = FORWARD;
}

void turnFor(int angle){
  movementState = TURNING;
  resetAngle();
  motorDifferenceIntegral = 0;
  angularSpeedIntegral = 0;
  angleTarget = angle*10;
  totalLeftEncoderCount = 0;
  totalRightEncoderCount = 0;
}

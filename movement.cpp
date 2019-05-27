#include "movement.h"
#include "motors.h"
#include "gyro.h"

#define FORWARD 1
#define ROTATING 2

int prevError;
int prevAngleError;
int prevRotationError;
long integral = 0;
long angleIntegral = 0;
long rotateIntegral = 0;
char movementState = FORWARD;



int longTermSpeedTarget = 0;
int shortTermSpeedTarget = 0;

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

     // Serial.println(angleError);

      if (abs(angleIntegral + angleError) < 255 / Kig)
        angleIntegral += angleError;
      int anglePower = Kpg * angleError + Kdg * (angleError - prevAngleError) + Kig * angleIntegral;
      prevAngleError = angleError;

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
      
      int error =  (angleTarget*0.31) - (totalLeftEncoderCount + totalRightEncoderCount)/2;

      if (abs(rotateIntegral + error) < 255 / Ki_rot)
        rotateIntegral += error;
      int power = Kpg_rot * error + Kdg_rot * (error - prevRotationError) + Ki_rot * rotateIntegral;
      prevRotationError = error;

      int lpower = power;
      int rpower = -power;

      if (lpower > MAX_MOTOR_POWER)lpower = MAX_MOTOR_POWER;
      else if (lpower < -MAX_MOTOR_POWER)lpower = -MAX_MOTOR_POWER;

      if (rpower > MAX_MOTOR_POWER)rpower = MAX_MOTOR_POWER;
      else if (rpower < -MAX_MOTOR_POWER)rpower = -MAX_MOTOR_POWER;

      motors_setPower(lpower, rpower);
      Serial.println(error);
      if(error < 0){
        
        motors_break();
        movementState = FORWARD;
      }
  }
}

void forward(int robotSpeed) {
  longTermSpeedTarget = robotSpeed;
  movementState = FORWARD;
}

void turnFor(int angle){
  movementState = TURNING;
  resetAngle();
  rotateIntegral = 0;
  angleTarget = angle*10;
  totalLeftEncoderCount = 0;
  totalRightEncoderCount = 0;
}

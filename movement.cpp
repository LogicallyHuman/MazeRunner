#include "movement.h"
#include "motors.h"
#include "gyro.h"

int prevError;
int prevAngleError;
long integral = 0;
long angleIntegral = 0;
long rotateIntegral = 0;
char robotState;

int longTermSpeedTarget = 0;
int shortTermSpeedTarget = 0;

int angleOffset = 0;

void resetAngle(){
  angleOffset = 10*gyro.getAngleZ();
}

long ticksTarget;

void updatePID() {
  motors_recordSpeed();
  Serial.println(gyro_getAngle());
  if(robotState == FORWARD_AND_STOP){
      if(ticksTarget < (totalLeftEncoderCount + totalRightEncoderCount)/2){
          robotState = STOP;
          Serial.println("Destination reached");
          Serial.println();
      }
  }
  if (robotState == STOP){
    motors_break();
  }
  else if (robotState == FORWARD || robotState == FORWARD_AND_STOP) {
    //ACCEL mm/s / s
   /* Serial.print(longTermSpeedTarget);
    Serial.print(" ");
    Serial.print(shortTermSpeedTarget);
    Serial.print(" ");
    Serial.println((leftMotorSpeed + rightMotorSpeed) / 2);*/


    if (shortTermSpeedTarget < longTermSpeedTarget) {
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

    if (shortTermSpeedTarget == 0) {
      motors_break();
    }
    else {


      int error = shortTermSpeedTarget - (leftMotorSpeed + rightMotorSpeed) / 2;

      if (abs(integral + error) < 255 / Ki)
        integral += error;
      int power = Kp * error + Kd * (error - prevError) + Ki * integral;
      prevError = error;

      int angleError = (-gyro_getAngle() - angleOffset);

      if (abs(angleIntegral + angleError) < 255 / Ki)
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
}

void forward(int robotSpeed) {
  longTermSpeedTarget = robotSpeed;
  robotState = FORWARD;
}

void forwardFor(int robotSpeed, int distance){
  longTermSpeedTarget = robotSpeed;
  ticksTarget = ((totalLeftEncoderCount + totalRightEncoderCount)/2) + distance*5.94;
  robotState = FORWARD_AND_STOP;
  Serial.println("Starting Forward for");
  Serial.print("Current position: ");
  Serial.println((totalLeftEncoderCount + totalRightEncoderCount)/2);
  Serial.print("Destination: ");
  Serial.println(ticksTarget);  
  Serial.println();
}

void turn(int rotateSpeed, int angle){
  //longTermRotateSpeedTarget = angleSpeed;
  robotState = ROTATING;
}


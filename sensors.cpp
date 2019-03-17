
#include "sensors.h"

VL53L0X sensorLeft;
VL53L0X sensorLeftDiag;
VL53L0X sensorFront;
VL53L0X sensorRightDiag;
VL53L0X sensorRight;

void sensors_init(){

  byte temp = 0;
  
  Wire.beginTransmission(PCF_WRITE_ADDR);  //RESTART ALL SENSORS
  Wire.write(0x00);
  Wire.endTransmission();  
  delay(10);
  Wire.beginTransmission(PCF_WRITE_ADDR);
  Wire.write(0xff);
  Wire.endTransmission(); 
  delay(20);   

  temp |= PCF_SENSOR_LEFT_DIAG;
  Wire.beginTransmission(PCF_WRITE_ADDR);  //CHANGE LEFT DIAG SENSOR ADDRESSS
  Wire.write(temp);
  Wire.endTransmission();
  delay(20);
  sensorLeftDiag.setAddress(0x2A);

  temp |= PCF_SENSOR_FRONT;
  Wire.beginTransmission(PCF_WRITE_ADDR);  //CHANGE LEFT DIAG SENSOR ADDRESSS
  Wire.write(temp);
  Wire.endTransmission();
  delay(20);
  sensorFront.setAddress(0x2B);  

  temp |= PCF_SENSOR_RIGHT_DIAG;
  Wire.beginTransmission(PCF_WRITE_ADDR);  //CHANGE LEFT DIAG SENSOR ADDRESSS
  Wire.write(temp);
  Wire.endTransmission();
  delay(20);
  sensorRightDiag.setAddress(0x2C);  

  temp |= PCF_SENSOR_RIGHT;
  Wire.beginTransmission(PCF_WRITE_ADDR);  //CHANGE LEFT DIAG SENSOR ADDRESSS
  Wire.write(temp);
  Wire.endTransmission();
  delay(20);
  sensorRight.setAddress(0x2D);  

  temp |= PCF_SENSOR_LEFT;
  Wire.beginTransmission(PCF_WRITE_ADDR);  //CHANGE LEFT DIAG SENSOR ADDRESSS
  Wire.write(temp);
  Wire.endTransmission();
    
  sensorLeft.init();
  sensorLeftDiag.init();
  sensorFront.init();
  sensorRightDiag.init();
  sensorRight.init();
  
  sensorLeft.setTimeout(TIMEOUT);
  sensorLeftDiag.setTimeout(TIMEOUT);
  sensorFront.setTimeout(TIMEOUT);
  sensorRightDiag.setTimeout(TIMEOUT);
  sensorRight.setTimeout(TIMEOUT);
/*
  sensorLeft.startContinuous();
  sensorLeftDiag.startContinuous();
  sensorFront.startContinuous();
  sensorRightDiag.startContinuous();
  sensorRight.startContinuous();
*/
  sensorLeft.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorLeftDiag.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorFront.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorRightDiag.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorRight.setMeasurementTimingBudget(TIMING_BUDGET);  

  sensorLeft.setSignalRateLimit(SIGNAL_RATE_LIMIT);
  sensorLeftDiag.setSignalRateLimit(SIGNAL_RATE_LIMIT);
  sensorFront.setSignalRateLimit(SIGNAL_RATE_LIMIT);
  sensorRightDiag.setSignalRateLimit(SIGNAL_RATE_LIMIT);
  sensorRight.setSignalRateLimit(SIGNAL_RATE_LIMIT);
  
}
#define LEFT       0x1
#define LEFT_DIAG  0x2
#define FRONT      0x4
#define RIGHT_DIAG 0x8
#define RIGHT      0x10

void sensors_readAll(int * readings){ //66ms

  sensorLeft.startRangeSingleMillimeters();
  sensorLeftDiag.startRangeSingleMillimeters();
  sensorFront.startRangeSingleMillimeters();
  sensorRightDiag.startRangeSingleMillimeters();
  sensorRight.startRangeSingleMillimeters();  
  
  readings[0] = sensorLeft.fetchRangeSingleMillimeters();
  readings[1] = sensorLeftDiag.fetchRangeSingleMillimeters();
  readings[2] = sensorFront.fetchRangeSingleMillimeters();
  readings[3] = sensorRightDiag.fetchRangeSingleMillimeters();
  readings[4] = sensorRight.fetchRangeSingleMillimeters();
}



void sensors_startReadAll(){ //66ms

  sensorLeft.startRangeSingleMillimeters();
  sensorLeftDiag.startRangeSingleMillimeters();
  sensorFront.startRangeSingleMillimeters();
  sensorRightDiag.startRangeSingleMillimeters();
  sensorRight.startRangeSingleMillimeters();  

}


void sensors_fetchReadAll(int * readings){ //66ms

  readings[0] = sensorLeft.fetchRangeSingleMillimeters();
  readings[1] = sensorLeftDiag.fetchRangeSingleMillimeters();
  readings[2] = sensorFront.fetchRangeSingleMillimeters();
  readings[3] = sensorRightDiag.fetchRangeSingleMillimeters();
  readings[4] = sensorRight.fetchRangeSingleMillimeters();
}

int sensors_readLeft(){
  return sensorLeft.readRangeSingleMillimeters2();
}
int sensors_readLeftDiag(){
  return sensorLeftDiag.readRangeSingleMillimeters2();
}
int sensors_readFront(){
  return sensorFront.readRangeSingleMillimeters2();
}
int sensors_readRightDiag(){
  return sensorRightDiag.readRangeSingleMillimeters2();
}
int sensors_readRight(){
  return sensorRight.readRangeSingleMillimeters2();
}

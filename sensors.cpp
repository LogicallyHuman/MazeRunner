
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

  sensorLeft.startContinuous();
  sensorLeftDiag.startContinuous();
  sensorFront.startContinuous();
  sensorRightDiag.startContinuous();
  sensorRight.startContinuous();

  sensorLeft.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorLeftDiag.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorFront.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorRightDiag.setMeasurementTimingBudget(TIMING_BUDGET);
  sensorRight.setMeasurementTimingBudget(TIMING_BUDGET);  

  sensorLeft.setSignalRateLimit(SIGNAL_RATE_LIMIT_SIDES);
  sensorLeftDiag.setSignalRateLimit(SIGNAL_RATE_LIMIT_DIAGS);
  sensorFront.setSignalRateLimit(SIGNAL_RATE_LIMIT_FRONT);
  sensorRightDiag.setSignalRateLimit(SIGNAL_RATE_LIMIT_DIAGS);
  sensorRight.setSignalRateLimit(SIGNAL_RATE_LIMIT_SIDES);


  
}

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
/*
  sensorLeft.startRangeSingleMillimeters();
  sensorLeftDiag.startRangeSingleMillimeters();
  sensorFront.startRangeSingleMillimeters();
  sensorRightDiag.startRangeSingleMillimeters();
  sensorRight.startRangeSingleMillimeters();  
*/
}


void sensors_fetchReadAll(int * readings){ //66ms
/*
  int d = sensorLeft.fetchRangeSingleMillimeters();
  if(d > 0) readings[0] = d; 

  d = sensorLeftDiag.fetchRangeSingleMillimeters();
  if(d > 0) readings[1] = d; 

  d = sensorFront.fetchRangeSingleMillimeters();
  if(d > 0) readings[2] = d; 

  d = sensorRightDiag.fetchRangeSingleMillimeters();
  if(d > 0) readings[3] = d; 

  d = sensorRight.fetchRangeSingleMillimeters();
  if(d > 0) readings[4] = d; 

  */
  int d = sensorLeft.readRangeContinuousMillimeters();
  if(d > 0) readings[0] = d; 

  d = sensorLeftDiag.readRangeContinuousMillimeters();
  if(d > 0) readings[1] = d; 

  d = sensorFront.readRangeContinuousMillimeters();
  if(d > 0) readings[2] = d; 

  d = sensorRightDiag.readRangeContinuousMillimeters();
  if(d > 0) readings[3] = d; 

  d = sensorRight.readRangeContinuousMillimeters();
  if(d > 0) readings[4] = d; 
 
}

int sensors_readLeft(){
  return sensorLeft.readRangeSingleMillimeters();
}
int sensors_readLeftDiag(){
  return sensorLeftDiag.readRangeSingleMillimeters();
}
int sensors_readFront(){
  return sensorFront.readRangeSingleMillimeters();
}
int sensors_readRightDiag(){
  return sensorRightDiag.readRangeSingleMillimeters();
}
int sensors_readRight(){
  return sensorRight.readRangeSingleMillimeters();
}


#define FRONT_LOW_THRESHOLD   350
#define FRONT_HIGH_THRESHOLD  360

#define DIAG_LOW_THRESHOLD  180
#define DIAG_HIGH_THRESHOLD 190

#define SIDE_LOW_THRESHOLD 300
#define SIDE_HIGH_THRESHOLD 320

#define WALL_LEFT 0b1
#define WALL_LEFT_DIAG 0b10
#define WALL_FRONT 0b100
#define WALL_RIGHT_DIAG 0b1000
#define WALL_RIGHT 0b10000



char sensors_interpretReadings(int * readings){
        static char interpretations = 0;

        if(readings[0] > SIDE_HIGH_THRESHOLD)
            interpretations |= 0b00001;
        if(readings[0] < SIDE_LOW_THRESHOLD)
            interpretations &= 0b11110;         

        if(readings[1] > DIAG_HIGH_THRESHOLD)
            interpretations |= 0b00010;
        if(readings[1] < DIAG_LOW_THRESHOLD)
            interpretations &= 0b11101;  

        if(readings[2] > FRONT_HIGH_THRESHOLD)
            interpretations |= 0b00100;
        if(readings[2] < FRONT_LOW_THRESHOLD)
            interpretations &= 0b11011;  

        if(readings[3] > SIDE_HIGH_THRESHOLD)
            interpretations |= 0b01000;
        if(readings[3] < SIDE_LOW_THRESHOLD)
            interpretations &= 0b10111;

        if(readings[4] > SIDE_HIGH_THRESHOLD)
            interpretations |= 0b10000;
        if(readings[4] < SIDE_LOW_THRESHOLD)
            interpretations &= 0b01111;

        return interpretations; 
    
}

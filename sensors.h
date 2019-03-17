#ifndef SENSORS_H
#define SENSORS_H


#define SIGNAL_RATE_LIMIT 0.8
#define TIMEOUT 500
#define TIMING_BUDGET 20000 //us

#include <Arduino.h>
#include "VL53L0X.h"
#include <Wire.h>

extern VL53L0X sensorLeft;
extern VL53L0X sensorLeftDiag;
extern VL53L0X sensorFront;
extern VL53L0X sensorRightDiag;
extern VL53L0X sensorRight;


#define PCF_WRITE_ADDR   B0111000
#define PCF_SENSOR_LEFT       B00001000
#define PCF_SENSOR_LEFT_DIAG  B00000100
#define PCF_SENSOR_FRONT      B10000000
#define PCF_SENSOR_RIGHT_DIAG B01000000
#define PCF_SENSOR_RIGHT      B00100000



void sensors_init();

int sensors_readLeft();
int sensors_readLeftDiag();
int sensors_readFront();
int sensors_readRightDiag();
int sensors_readRight();
void sensors_readAll(int * readings);
void sensors_startReadAll();
void sensors_fetchReadAll(int * readings);
#endif


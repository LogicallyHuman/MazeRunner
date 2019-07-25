#ifndef SENSORS_H
#define SENSORS_H

#define LEFT       0x1
#define LEFT_DIAG  0x2
#define FRONT      0x4
#define RIGHT_DIAG 0x8
#define RIGHT      0x10


#define SIGNAL_RATE_LIMIT_DIAGS 0.2
#define SIGNAL_RATE_LIMIT_SIDES 0.5
#define SIGNAL_RATE_LIMIT_FRONT 0.5
#define TIMEOUT 30
#define TIMING_BUDGET 20000 //us

#include <Arduino.h>
#include "VL53L0X.h"
#include <Wire.h>

extern VL53L0X sensorLeft;
extern VL53L0X sensorLeftDiag;
extern VL53L0X sensorFront;
extern VL53L0X sensorRightDiag;
extern VL53L0X sensorRight;

enum turnTypes{NO_TURN, U_TURN, LEFT_TURN, RIGHT_TURN, LEFT_RIGHT_TURN, LEFT_FRONT_TURN, RIGHT_FRONT_TURN, ALL_TURN, SLOW_DOWN};


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
void sensors_fetchReadAll(volatile int * readings);
char sensors_interpretReadings(volatile int * readings);
#endif


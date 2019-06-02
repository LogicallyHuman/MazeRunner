#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include "MPU6050_tockn.h"
#include <EEPROM.h>

extern MPU6050 gyro;


void gyro_init();
int gyro_getAngle();
void gyro_reset();
void gyro_readOffsetsFromEEPROM();
void gyro_calcOffsetsAndSave();

#endif

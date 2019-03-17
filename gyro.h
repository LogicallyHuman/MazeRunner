#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>
#include "MPU6050_tockn.h"

extern MPU6050 gyro;


void gyro_init();
int gyro_getAngle();

#endif

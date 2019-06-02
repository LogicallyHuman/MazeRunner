#include "gyro.h"

MPU6050 gyro(Wire);

int gyroOffset = 0;

void gyro_init() {
    gyro.begin();
    //gyro.setGyroOffsets(-4.14, -0.88, -0.36);
}
/*
X : -9.29
Y : -1.42
Z : -0.82
*/
int gyro_getAngle() {
    return gyro.getAngleZ() * 10 - gyroOffset;
}

void gyro_reset() {
    gyroOffset = gyro.getAngleZ() * 10;
}

void gyro_readOffsetsFromEEPROM() {
    float x, y, z;  //Read gyro offsets from EEPROM
    EEPROM.get(0, x);
    EEPROM.get(4, y);
    EEPROM.get(8, z);
    gyro.setGyroOffsets(x, y, z);
}

void gyro_calcOffsetsAndSave() {
    gyro.calcGyroOffsets(false);
    EEPROM.put(0, gyro.getGyroXoffset());
    EEPROM.put(4, gyro.getGyroYoffset());
    EEPROM.put(8, gyro.getGyroZoffset());
}
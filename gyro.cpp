#include "gyro.h"


MPU6050 gyro(Wire);


void gyro_init(){
  gyro.begin();
  gyro.setGyroOffsets(-3.42, -0.96, -0.34);
}

int gyro_getAngle(){
  return gyro.getAngleZ()*10;  
}

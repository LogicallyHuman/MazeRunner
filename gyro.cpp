#include "gyro.h"


MPU6050 gyro(Wire);



void gyro_init(){
  gyro.begin();
  gyro.setGyroOffsets(-4.14, -0.88, -0.36);
}

int gyro_getAngle(){
  return gyro.getAngleZ()*10;  
}

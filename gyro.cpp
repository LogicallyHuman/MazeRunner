#include "gyro.h"


MPU6050 gyro(Wire);



void gyro_init(){
  gyro.begin();
  //gyro.setGyroOffsets(-4.2, -0.88, -0.34);
  gyro.calcGyroOffsets(true);
}

int gyro_getAngle(){
  return gyro.getAngleZ()*10;  
}

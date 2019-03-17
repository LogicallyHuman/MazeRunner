
#include <SimpleKalmanFilter.h>

#include "motors.h"
#include "leds.h"
#include "sensors.h"
#include "movement.h"
#include "gyro.h"

#define BTN1   A1
#define BTN2   13
#define BTN3   10

long lastTurnTime = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  motors_init();
  leds_init();
  sensors_init();
  gyro_init();
  pinMode(BTN1, INPUT);

  digitalWrite(LED1, HIGH);
  while (digitalRead(BTN1)) {}

  delay(500);
  //motors_setSpeed(100, 0);
 digitalWrite(LED1, LOW); 

}

unsigned long lastSpeedReadTime = 0;

char state = 1;

int desiredAngle = 0;

void loop() {
  gyro.update();
  if (millis() > lastSpeedReadTime + 30) {
      updatePID();
      lastSpeedReadTime = millis();
  }
  if(state == 1){
    forwardFor(300, 100);
    state = 2;
  }
  else if(state == 2){
    if(robotState == STOP)state = 3;
  }
  else if(state == 3){
    forwardFor(70, 100);
    state = 4;
  }
}


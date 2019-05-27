

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
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT);

  digitalWrite(LED1, HIGH);
  while (digitalRead(BTN1)) {}
  resetAngle();
  delay(500);
  //motors_setSpeed(100, 0);
  digitalWrite(LED1, LOW);
  forward(1500);
  sensors_startReadAll();
  delay(50);
}

unsigned long lastSpeedReadTime = 0;


#define FORWARD 1
#define TURNING 2

int desiredAngle = 0;

int sensorReadings[5];

void updateSensorReadings(){
  sensors_fetchReadAll(sensorReadings);
  sensors_startReadAll();
}



char startedTurning = false;
char shouldTurn = true;
long wallReachTime = 0;

#define STOP 0
#define FORWARDING 1
#define WAITING_TO_TURN 2
#define ROBOT_TURNING 3

char state = FORWARDING;

void loop() {
  static long prevRefresh = 0;
  char sensorUpdated = false;
      digitalWrite(LED1, !digitalRead(LED1));

  gyro.update();
  if(prevRefresh + 30 < millis()){
    if(state == FORWARDING)updateSensorReadings();
    updatePID();
    prevRefresh = millis();
    sensorUpdated = true;
  }

  if(state == FORWARDING){
    if(sensorUpdated){
      sensorUpdated = false;
      if(sensorReadings[2] < 300){
        wallReachTime = millis();
        forward(0);
        state = WAITING_TO_TURN;
      }
    }
  }

  if(state == WAITING_TO_TURN){
    if(wallReachTime + 500 < millis()){
      turnFor(180);
      state = ROBOT_TURNING;
    }
  }

  if(state == ROBOT_TURNING){
    if(movementState == FORWARD){
      state = STOP; 
    }

  }

  if(state == STOP){
      motors_break();
  }

  if(!digitalRead(BTN1)){
    resetAngle();
    //delay(500);
    state = FORWARDING;
    forward(1500);
  }


}
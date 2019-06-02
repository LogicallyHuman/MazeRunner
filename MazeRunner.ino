#include "gyro.h"
#include "leds.h"
#include "motors.h"
#include "movement.h"
#include "sensors.h"
#include <EEPROM.h>

#define BTN1 A1  //TODO: Check buttons. Move to spereate file.
#define BTN2 13
#define BTN3 10

enum setupStates{READY, WAITING_TO_START};

void setup() {
    Wire.begin();          //Starts i2c functionality
    Serial.begin(115200);  //Starts serial functionality
    motors_init();  //Starts robot control functionality
    leds_init();
    sensors_init();
    gyro_init();
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);

    char setupState = READY;
    char start = 0;

    while (!start) {
        switch(setupState){
            case READY:
            if(millis()%100 < 50){
                leds_set(0b0110);
            }
            else{
                leds_set(0b0000);
            }
            if(!digitalRead(BTN2))
                setupState = WAITING_TO_START;
            if(!digitalRead(BTN1)){
                leds_set(0b1001);
                gyro_calcOffsetsAndSave();
            }
            break;

            case WAITING_TO_START:
            sensors_startReadAll();
            delay(500);
            gyro_reset();
            delay(50);
            leds_set(0b0000);
            start = 1;

        }
    }
    forward(700); 
}

unsigned long lastSpeedReadTime = 0;



int sensorReadings[5];

void updateSensorReadings() {
    sensors_fetchReadAll(sensorReadings);
    sensors_startReadAll();
}

char startedTurning = false;
char shouldTurn = true;
long wallReachTime = 0;
long finishedTurningTime = 0;




//enum runStates {FORWARDING, WAITING_TO_RUN, TURNING, WAITING_TO_FORWARD};

//char runState = FORWARDING;

void loop() {
    static long prevRefresh = 0;
    char sensorUpdated = false;
    gyro.update();
    if (prevRefresh + 30 < millis()) {
        updateSensorReadings();
        updatePID(sensorReadings);
        prevRefresh = millis();
        sensorUpdated = true;
    }


    int r = sensors_interpretReadings(sensorReadings);
    //digitalWrite(LED1, r&LEFT_DIAG);
    //digitalWrite(LED2, r&FRONT);    
    //digitalWrite(LED3, r&RIGHT_DIAG); 
}
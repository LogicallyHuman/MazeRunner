/*
    EEPROM:
        0-3: Gyro X Offset
        4-7: Gyro Y Offset
        8-11: Gyro Z Offset
        12-13: Sum of side distances
 */


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

char rotateWith2 = 0;

void setup() {
    Wire.begin();          //Starts i2c functionality
    Serial.begin(115200);  //Starts serial functionality
    motors_init();  //Starts robot control functionality
    leds_init();
    sensors_init();
    gyro_init();
    sensors_startReadAll();
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
            if(!digitalRead(BTN3)){
                rotateWith2 = true;
                setupState = WAITING_TO_START;
            }
            if(!digitalRead(BTN1)){
                leds_set(0b1001);
                gyro_calcOffsetsAndSave();
                int tempSensorReadings[5];
                sensors_fetchReadAll(tempSensorReadings);
                EEPROM.put(12, tempSensorReadings[4] + tempSensorReadings[0]);
            }
            break;

            case WAITING_TO_START:
            sensors_startReadAll();
            delay(500);
            gyro_reset();
            delay(50);
            leds_set(0b0000);
            EEPROM.get(12, idealSumOfSideDistance);
            start = 1;

        }
    }
    forward(800); 
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




enum runStates {FORWARDING, WAITING_TO_TRUN_SLOW, WAITING_TO_TURN_STOP, TURNING, WAITING_TO_FORWARD, FORWARD_BLIND, WAITING_TO_TURN2};

char runState = FORWARDING;



void loop() {
    static long prevRefresh = 0;
    char sensorUpdated = false;
    static char interpretatedReadings;
    static int distanceCount = 0;
    static long millisCount = 0;
    gyro.update();
    static char lastTurn = 0;
    static int turnToDo = 0;

    if (prevRefresh + 30 < millis()) {
        updateSensorReadings();
        updatePID(sensorReadings, runState == FORWARD_BLIND);
        prevRefresh = millis();
        sensorUpdated = true;
        interpretatedReadings = sensors_interpretReadings(sensorReadings);
        distanceCount += (leftMotorSpeed + rightMotorSpeed)/2;
        //juan += 1;
    }

    //forward(500);
    
    

    
    //if(runState == FORWARDING){
        //turnFor2();
        if(interpretatedReadings == LEFT_TURN){
            turnFor2();
            leds_set(0b1000);
        }
        else if(interpretatedReadings == RIGHT_TURN){
            turnFor2();
            leds_set(0b0001);
        }
        else if(interpretatedReadings == ALL_TURN){
            //forward(500);
            leds_set(0b1111);
        }
        else if(interpretatedReadings == U_TURN){
            leds_set(0b0110);
        }
        else if(interpretatedReadings == LEFT_FRONT_TURN){
            turnFor2();
            leds_set(0b1100);
        }
        else if(interpretatedReadings == RIGHT_FRONT_TURN){
            turnFor2();
            leds_set(0b0011);
        }
        else if(interpretatedReadings == LEFT_RIGHT_TURN){
            turnFor2();
            leds_set(0b1001);
        }
        else{
            //forward(500);
            leds_set(0b0000);
        }
        if(movementState == MOTORS_FORWARD){
            //leds_set(0b1000);
        }
        else{
            //leds_set(0b0000);
        }

    //}
    /*
    else if(runState == WAITING_TO_TRUN_SLOW){
        if(interpretatedReadings == NO_TURN){
            runState == FORWARDING;
        }
        if(distanceCount > 600){
            forward(0);
            millisCount = millis();
            runState = WAITING_TO_TURN_STOP;
        }
        leds_set(0b1000);
    }
    else if(runState == WAITING_TO_TURN_STOP){
        if(millisCount + 1000 < millis()){
            runState = TURNING;
            if(turnToDo == LEFT_TURN){
                turnFor(90);
                lastTurn = LEFT_TURN;
            }
            else if(turnToDo == RIGHT_TURN){
                turnFor(-90);
                lastTurn = RIGHT_TURN;
            }
        }
        leds_set(0b0000);
    }
    else if(runState == WAITING_TO_TURN2){
        if(interpretatedReadings == NO_TURN){
            runState == FORWARDING;
        }
        if(distanceCount > 300){
            turnFor2();
            runState = TURNING;
        }
        leds_set(0b1000);
    }
    else if(runState == TURNING){
        if(movementState == MOTORS_FORWARD){
            runState = FORWARD_BLIND;
            gyro_reset();
            forward(0);
            millisCount = millis();
        }
        leds_set(0b0100);
    }
    else if(runState == WAITING_TO_FORWARD){
        if(millisCount + 1000 < millis()){
            runState = FORWARD_BLIND;
            forward(200);
            distanceCount = 0;
        }
        leds_set(0b0100);
    }
    else if(runState == FORWARD_BLIND){
        if(lastTurn == LEFT_TURN){
            if(interpretatedReadings&LEFT){
                runState = FORWARDING;
            }
        }
        if(lastTurn == RIGHT_TURN){
            if(interpretatedReadings&RIGHT){
                runState = FORWARDING;
            }
        }
        leds_set(0b0010);
    }
    //digitalWrite(LED1, r&LEFT_DIAG);
    //digitalWrite(LED2, r&FRONT);    
    //digitalWrite(LED3, r&RIGHT_DIAG); 
    */
}
#include "leds.h"
#include <Arduino.h>
void leds_init(){
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
}


void leds_set(char states){
    digitalWrite(LED1, states&0b1000);
    digitalWrite(LED2, states&0b0100);
    digitalWrite(LED3, states&0b0010);
    digitalWrite(LED4, states&0b0001);
}
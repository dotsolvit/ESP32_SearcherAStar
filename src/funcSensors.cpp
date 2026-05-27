//Functions Sensors: ultrasonic and infrared

#include <Arduino.h>

#include "config.hpp"
#include "funcSensors.hpp"

//Distance covered counters:
extern int distancePulseCounterLeft;
extern int distancePulseCounterRight;

//Sonic initialization
void initEcho() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}

//Display distance using ultrasonic sensor
int distanceEcho() {
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  int cm = pulseIn(ECHO_PIN, HIGH, 5000 ) / 58;  //86см примерно=5000 мкс
  if(cm < ULTRASONIC_MIN_DISTANCE or cm > ULTRASONIC_MAX_DISTANCE) cm=0;
  return cm;
}
/*
//Display distance
int IR_Distance(void){
  //analogRead= 0–4095 that read voltages from 0V to 3.3V
  float floatValue = float( analogRead(PIN_IR) ); //Read value from analog pin 
  int currentDist = int(pow( floatValue, COEFFICIENT_B) * COEFFICIENT_A);
  if(currentDist < IR_MIN_DISTANCE or currentDist > IR_MAX_DISTANCE) currentDist=0;
  return currentDist;
}
*/

//distance counter - odometr
int odometer(void){
  float float_dist_counter= float(distancePulseCounterLeft + distancePulseCounterRight)/2.0 * DISTANCE_PER_IMPULSE;
  return int(float_dist_counter);
}

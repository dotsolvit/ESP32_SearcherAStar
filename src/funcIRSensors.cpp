//Functions IR Sensors
#include <Wire.h>

#include "config.hpp"
#include "funcIRSensors.hpp"

//Distance covered counters:
extern int distancePulseCounterLeft;
extern int distancePulseCounterRight;


//Display distance
int IR_Distance(void){
  //analogRead= 0–4095 that read voltages from 0V to 3.3V
  float floatValue = float( analogRead(PIN_IR) ); //Read value from analog pin 
  int currentDist = int(pow( floatValue, COEFFICIENT_B) * COEFFICIENT_A);
  if(currentDist < 20 or currentDist > 150) currentDist=0;
  return currentDist;
}

//distance counter - odometr
int odometer(void){
  float float_dist_counter= float(distancePulseCounterLeft + distancePulseCounterRight)/2.0 * DISTANCE_PER_IMPULSE;
  return int(float_dist_counter);
}

//Functions for motors and servo

#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "funcMotors.hpp"
#include "funcMPU6050.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"

//Scanner angle:    
extern int scannerAngle;

//Motor initialization
void initializationMotors(void){
    // Attach pins to channels
    // Old way (v2.x)
    ledcSetup( MOTOR_L_A_Channel, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // (channel, frequency, resolution)
    ledcAttachPin(MOTOR_L_A_Pin, MOTOR_L_A_Channel);  // (pin, channel)
    ledcSetup( MOTOR_L_B_Channel, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // 
    ledcAttachPin(MOTOR_L_B_Pin, MOTOR_L_B_Channel);  // (pin, channel)
    ledcSetup( MOTOR_R_A_Channel, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // 
    ledcAttachPin(MOTOR_R_A_Pin, MOTOR_R_A_Channel);  // (pin, channel)
    ledcSetup( MOTOR_R_B_Channel, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // 
    ledcAttachPin(MOTOR_R_B_Pin, MOTOR_R_B_Channel);  // (pin, channel)

    ledcWrite(MOTOR_L_A_Channel, 0);
    ledcWrite(MOTOR_L_B_Channel, 0);
    ledcWrite(MOTOR_R_A_Channel, 0);
    ledcWrite(MOTOR_R_B_Channel, 0);
}

//Init Buzzer:
void initBuzzer() {
    tone(BUZ_PIN, 500);
    vTaskDelay(100 / portTICK_PERIOD_MS);       
    noTone(BUZ_PIN);
    vTaskDelay(100 / portTICK_PERIOD_MS);       
    tone(BUZ_PIN, 1000);
    vTaskDelay(100 / portTICK_PERIOD_MS);         
    noTone(BUZ_PIN);
}

//Tank Forward
void TankForward(int motor_speed){
  ledcWrite(MOTOR_L_A_Channel, motor_speed-SPEED_ADJUSTMENT);
  ledcWrite(MOTOR_L_B_Channel, 0);
  ledcWrite(MOTOR_R_A_Channel, motor_speed+SPEED_ADJUSTMENT);
  ledcWrite(MOTOR_R_B_Channel, 0);
}

//TankStop
void TankStop(void){
  ledcWrite(MOTOR_L_A_Channel, 0);
  ledcWrite(MOTOR_L_B_Channel, 0);
  ledcWrite(MOTOR_R_A_Channel, 0);
  ledcWrite(MOTOR_R_B_Channel, 0);
}


//Turn the tank to a new course angle
void TankRorateOnAngle(int new_angle){
  int cur_angle = getAngleX();
  int start_diff = differenceInAngles(new_angle,cur_angle,0);
  int onRight = ( start_diff > 0 ) ? 1 : -1;
  //test
  Serial.print("start_diff ="); Serial.print(start_diff);
  Serial.print("onRight ="); Serial.print(onRight);
  //
  if(abs(start_diff) > 15){
    while (true){
        cur_angle= getAngleX();
        int diff=differenceInAngles(new_angle,cur_angle, 0);
        //Show diff: 
        displayMessage(3, "Diff= ", diff, " ");
        //
        if( abs(diff) <= 10 ) break;
        if( diff > 0){
          TankRotateRight(40, SPEED_NORMAL); //50
        }
        else{    
          if(differenceInAngles(new_angle,cur_angle,onRight) >= -10) break;
          Serial.print("diff=");Serial.print(diff);Serial.println(" TankRotateLeft(50)");
          TankRotateLeft(40, SPEED_NORMAL);  //50
        }
    }
  }
  while (true){
      cur_angle= getAngleX();
      int diff=differenceInAngles(new_angle,cur_angle, 0);
      //Show diff: 
      displayMessage(3, "Diff= ", diff, " ");
      //
      if( abs(diff) <= 1 ) break;
      if( diff > 0){
        TankRotateRight(20, SPEED_SLOW_TURN);   //20 //25  //30
      }
      else{
        Serial.print("diff=");Serial.print(diff);Serial.println(" TankRotateLeft(20)");
        TankRotateLeft(20, SPEED_SLOW_TURN);   //20 //25   //30   
      }
  }
  return ;
}

void TankRotateLeft(int pause, int speed){
  ledcWrite(MOTOR_L_A_Channel, 0);
  ledcWrite(MOTOR_L_B_Channel, speed);
  ledcWrite(MOTOR_R_A_Channel, speed);
  ledcWrite(MOTOR_R_B_Channel, 0);
  vTaskDelay(pause / portTICK_PERIOD_MS); // delay(pause);
  TankStop();
}

void TankRotateRight(int pause, int speed){
  ledcWrite(MOTOR_L_A_Channel, speed);
  ledcWrite(MOTOR_L_B_Channel, 0);
  ledcWrite(MOTOR_R_A_Channel, 0);
  ledcWrite(MOTOR_R_B_Channel, speed);
  vTaskDelay(pause / portTICK_PERIOD_MS); // delay(pause);
  TankStop();
}

//SERVO
//Servo intialization
void initServo(void){
  // Setting up the PWM channel (Channel, Frequency, Resolution)
  ledcSetup(SERVO_Channel, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  // Attaching the channel to the ESP32 pin
  ledcAttachPin(PIN_SERVO, SERVO_Channel);
  scannerAngle = 0; //Initial scanner angle
  setServo(scannerAngle); //set to 0 (servo 90 degrees)   
}

//Set Servo from - 90 (0) +90 degrees
void setServo(int angl){
  angl=constrain(angl, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
  float min=0.065536 * SERVO_PWM_FREQ * SERVO_MIN_PULSE;
  float max=0.065536 * SERVO_PWM_FREQ * SERVO_MAX_PULSE;
  int pos = map(angl, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE, int(max), int(min)); //set servo pos
  ledcWrite(SERVO_Channel, pos);
}

//Tank Signal
void TankBuz(int signal){
  struct {
    int firstTone, firstDelay, secondTone, secondDelay;
  } buz_signal;
 
  switch (signal) {
    case SIGNAL_GO:
      buz_signal={200, 50, 500, 50};
      break;
    case SIGNAL_NOPATH:
      buz_signal={500, 500, 100, 10};
      break;
    case SIGNAL_OBSTACLE:
      buz_signal={500, 500, 500, 500};
      break;     
    default: 
      buz_signal={100, 10, 100, 10}; 
  }
  tone(BUZ_PIN, buz_signal.firstTone);
  vTaskDelay(buz_signal.firstDelay / portTICK_PERIOD_MS);
  tone(BUZ_PIN, buz_signal.secondTone);
  vTaskDelay(buz_signal.secondDelay / portTICK_PERIOD_MS);
  noTone(BUZ_PIN);
}
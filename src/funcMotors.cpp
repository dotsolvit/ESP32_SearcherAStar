//Functions for motors and servo
// Add Lib: ESP32Servo by Kevin Harrington 3.2.0

#include <Wire.h>
#include <ESP32Servo.h>

#include "config.hpp"
#include "funcMotors.hpp"

Servo servoMotor;  // Створюємо об'єкт сервопривода

//Motor initialization
void initializationMotors(void){
    // Attach pins to channels
    // Old way (v2.x)
    ledcSetup(0, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // (channel, frequency, resolution)
    ledcAttachPin(MOTOR_L_A_Pin, 0);  // (pin, channel)
    ledcSetup(1, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // (channel, frequency, resolution)
    ledcAttachPin(MOTOR_L_B_Pin, 1);  // (pin, channel)
    ledcSetup(2, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // (channel, frequency, resolution)
    ledcAttachPin(MOTOR_R_A_Pin, 2);  // (pin, channel)
    ledcSetup(3, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION); // (channel, frequency, resolution)
    ledcAttachPin(MOTOR_R_B_Pin, 3);  // (pin, channel) 

    //new way (v3.x)
    //ledcAttach(MOTOR_L_A_Pin, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION);
    //ledcAttach(MOTOR_L_B_Pin, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION);
    //ledcAttach(MOTOR_R_A_Pin, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION);
    //ledcAttach(MOTOR_R_B_Pin, MOTORS_PWM_FREQ, MOTORS_PWM_RESOLUTION);

    // Old way (v2.x)
    ledcWrite(MOTOR_L_A_Channel, 0);
    ledcWrite(MOTOR_L_B_Channel, 0);
    ledcWrite(MOTOR_R_A_Channel, 0);
    ledcWrite(MOTOR_R_B_Channel, 0);
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

//SERVO
//Servo intialization
void initServo(void){
  servoMotor.setPeriodHertz(50); // // Standard 50hz servo
  servoMotor.attach(PIN_SERVO, SERVO_MIN_PULSE, SERVO_MAX_PULSE);  // Attach the servo with min/max pulse widths
  setServo(0); //set to 0 (servo 90 degrees)   
}

//Set Servo from - 86 (0) +86 degrees
void setServo(int angl){
  angl=constrain(angl, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE);
  int pos = map(angl, -SERVO_MAX_ANGLE, SERVO_MAX_ANGLE, 178, 0); //set servo pos
  servoMotor.write(pos);
}
//Functions for motors

//MOTORS
//Motor initialization
void initializationMotors();

//Init Buzzer:
void initBuzzer() ;

//Tank Forward
void TankForward(int motor_speed);
//TankStop
void TankStop(void);

//SERVO
//Servo intialization
void initServo(void);
//Set Servo from - 86 (0) +86 degrees
void setServo(int angl);

//Tank Signal
void TankBuz(int signal);

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

//Turn the tank to a new course angle
void TankRorateOnAngle(int new_angle);

void TankRotateLeft(int pause);
void TankRotateRight(int pause);

//SERVO
//Servo intialization
void initServo(void);
//Set Servo from - 90 (0) +90 degrees
void setServo(int angl);

//Tank Signal
void TankBuz(int signal);
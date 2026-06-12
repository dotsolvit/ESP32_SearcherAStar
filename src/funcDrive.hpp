//funcDrive.hpp

//Init EEPROM:
void initEEPROM() ;

//Init Real Coordinates:
void initRealCoords();

//Init Obstacle Set:
int initializationObstacleSet();

//Initialization Route Set:
int initializationRouteSet();

//Init Stage:
void initStage();

//Init Buzzer:
void initBuzzer();

//Init MPU6050:
void initMPU6050();

//Execute main drive logic
void cycleDrive(void);
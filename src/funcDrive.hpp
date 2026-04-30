//funcDrive.hpp

//Init Real Coordinates:
void initRealCoords();

//Init Obstacle Set:
int initializationObstacleSet();

//Init Stage:
void initStage();

//Init MPU6050:
void initMPU6050();

// виконуємо основну логіку керування (execute main drive logic)
void cycleDrive(void);
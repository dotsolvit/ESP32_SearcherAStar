//Functions MPU6050
//Lib: MPU6050 by Electronic Cats
//Used pins: SDA - GPIO21, SCL - GPIO22
//           VCC - 3.3V, GND - GND

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "funcMPU6050.hpp" //Functions MPU6050

//Наявність модуля MPU6050
bool isMPU6050Present;

//For MPU6050 DMP6:
MPU6050 mpu;
/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint8_t FIFOBuffer[64]; // FIFO storage buffer
/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
float euler[3];         // [psi, theta, phi]    Euler angle container

//MPU6050 DMP6 initialization
void initializationMPU6050(){
  Wire.begin();
  isMPU6050Present=mpu.testConnection();
  if(isMPU6050Present) {
    Serial.println("MPU6050 is connected");
  } else {
    Serial.println("MPU6050 connection failed");
    return;
  } 
  Serial.print("MPU6050 Initialization"); 
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    DMPReady = true;
  } 
  else {
    Serial.print(F("DMP Initialization failed")); //Print the error code
  }
}

// normalization of the angles in degrees to the range [0, 360]
int normalize(int angle) {
  int res=angle % 360;
  if (res < 0) res += 360;
  return res;
}

//Get rotation angle in degrees:
int getAngleX(void){
  if(!isMPU6050Present) return 0; 
  //rotation angle:
  float angle; //angle in radians
  int angleInDegrees;
  for(int i=0; i<3; i++){
    angle=0.0;
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetEuler(euler, &q);
      angle = euler[0]; //rotation angle X in radians
    }
    if(!(angle == 0.0)) break;
    delay(10);
  }
  //To degrees
  angle = angle * 180.0 / M_PI;
  angleInDegrees = int(angle);
  return normalize(angleInDegrees);
}




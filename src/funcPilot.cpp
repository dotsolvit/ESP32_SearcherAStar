//Function Pilot

#include <Arduino.h>
#include "config.hpp" 
#include "funcPilot.hpp"
#include "funcMotors.hpp" 
#include "funcMPU6050.hpp"
#include "funcTransfCoordsAngles.hpp"

//Path array (in main.cpp):
extern Coord pathSet[MAX_PATH_LENGH];
extern Par pathSetPar;
//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;
//Real Coordinates
extern realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель

//pathIndexForGo
extern int pathIndexForGo;

extern int currentAngle, displayed_currentAngle; //Текущий угол по Х

//Pilot initialization
int pilotInit() {
    if(pathSetPar.setSize==0 or pathSetPar.setSize==1){
        Serial.println("The Path is empty. No GO!");
        TankBuz(SIGNAL_NOPATH);
        return -1; // Return an error code
    }
    pathIndexForGo=pathSetPar.setSize-1; //Index for path array, from which the robot will start moving
    return 0; // Return success code
}

//Turn to the current point of the path
void pilotTurn() {
    int new_angle=calcAngleToNewPointPath(realCoordsCurrent, pathSet[pathIndexForGo-1]); //Calculating the angle to a new waypoint
    currentAngle = getAngleX();
    if(currentAngle != new_angle){
        Serial.print("Rotate from ");Serial.print(currentAngle); Serial.print(" to "); Serial.println(new_angle);
        TankRorateOnAngle(new_angle);
        currentAngle = getAngleX();
    }
    else Serial.println("The angle is the same");
}
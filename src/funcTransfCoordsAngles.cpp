//funcTransfCoordsAngles.cpp
//Functions for transforming coordinates, angles, etc.

#include <Arduino.h>
#include "config.hpp"       
#include "funcTransfCoordsAngles.hpp"
#include "funcArray.hpp"
#include "funcPilot.hpp"

//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;
//Route array (in main.cpp):
extern Coord routeSet[MAX_ROUTE_LENGH];
extern Par routeSetPar;

int prevDistanceCoveredRoute = 0; // Previous distance covered

//Transform real coordinates to grid coordinates
Coord TransformRealToGridCoords(realCoord &coord_in ){
  Coord coord_out;
  coord_out.y=int(coord_in.y/STEP_GRID);
  coord_out.x=int(coord_in.x/STEP_GRID);
  return coord_out;
}

//Transform grid coordinates to real coordinates
realCoord TransformGridToRealCoords(Coord &coord_in ){
  realCoord coord_out;
  coord_out.y=coord_in.y*STEP_GRID + int( STEP_GRID/2 );
  coord_out.x=coord_in.x*STEP_GRID + int( STEP_GRID/2 );
  return coord_out;
}

//Calculation of new current coordinates
realCoord calcRealCoords(realCoord &coord_in, int angle_in, int distance){
  realCoord coord_out = coord_in;
  float angle_rad = float(angle_in) * M_PI /180;
  float dY = - sin(angle_rad) * distance; //in cm, minus because our angle is clockwise
  float dX = cos(angle_rad) * distance; //in cm
  coord_out.y += int(dY);
  coord_out.x += int(dX);
  return coord_out;
}

//Calculation of the angle to the new path point
int calcAngleToNewPointPath(realCoord &coord_in, Coord &new_coord_in){
  int angle_out;
  realCoord new_coord = TransformGridToRealCoords(new_coord_in);
  //Finding the difference in coordinates:
  float dY=float(new_coord.y - coord_in.y);
  float dX=float(new_coord.x - coord_in.x);
  //Calculation of the angle (in radians):
  float angleRad = - atan2(dY, dX);//minus because our angle is clockwise
  //Conversion to degrees and normalization:
  float angleDeg = angleRad * 180 / M_PI;
  angle_out = normalizeAngle(int(angleDeg));
  return angle_out;
}

// normalization of the angles in degrees to the range [0, 360]
int normalizeAngle(int angle) {
  int res=angle % 360;
  if (res < 0) res += 360;
  return res;
}

//difference in angles  (perameter= -1 CCW; 1 CW; 0 min)
int differenceInAngles(int a1, int a0, int parameter){
  int cw,cww;
  a1=normalizeAngle(a1);
  a0=normalizeAngle(a0);
  //
  cw =a1-a0+((a1 >= a0) ? 0.0 : 360);
  cww=a1-a0+((a1 <= a0) ? 0.0 : -360);
  if(parameter==-1) return cww; //CWW
  else if(parameter==1) return cw;  //CW
  return( (abs(cw) < abs(cww)) ? cw : cww); //min
}

//Calculating the distance between nodes
int distanceBetweenPoints(realCoord &coord_in, Coord &new_coord_in){
  realCoord new_coord = TransformGridToRealCoords(new_coord_in);
  long dy=new_coord.y - coord_in.y;
  dy=dy*dy;
  long dx=new_coord.x - coord_in.x;
  dx=dx*dx;
  dy=dy+dx;
  double value = sqrt(dy);
  return int(value);
}

///////////////////////
//Processing measurement data to find and install obstacles.
int seekAndSetObstacle(realCoord realCoordsCurrent, int currentAngle, int scannerOffset, int scannerAngle, int distanceM){
  //Calculating real coordinates of the scanner
  realCoord scannerRealCoords = calcRealCoords(realCoordsCurrent, currentAngle, scannerOffset);
  //Calculating real coordinates of the detected obstacle
  realCoord obstacleRealCoords = calcRealCoords(scannerRealCoords, currentAngle + scannerAngle, distanceM);
  //Transform real coordinates of the detected obstacle to grid coordinates
  Coord obstacleCoords = TransformRealToGridCoords(obstacleRealCoords);
  //We check whether such an obstacle exists in the obstacle array.
  if(indexFindPointCoords(obstacleSet,obstacleSetPar, obstacleCoords)==-1) { //If the obstacle is new
    if(AddCoords(obstacleSet,obstacleSetPar, obstacleCoords) == -1) {
      String message = "Failed to add obstacle: (" + String(obstacleCoords.y) + ", " + String(obstacleCoords.x) + ")";
      addToJournal(message.c_str()); // Add message to journal
      return -1; //Failed to add new obstacle
    }
  }
  return 0;
}

//Processing measurement data to find obstacles.
int seekObstacle(realCoord realCoordsCurrent, int currentAngle, int currentDistanceCovered, int scannerOffset, int scannerAngle, int distanceM){
  //Calculating real coordinates of the scanner
  realCoord scannerRealCoords = calcRealCoords(realCoordsCurrent, currentAngle, currentDistanceCovered + scannerOffset);
  //Calculating real coordinates of the detected obstacle
  realCoord obstacleRealCoords = calcRealCoords(scannerRealCoords, currentAngle + scannerAngle, distanceM);
    //Calculating real coordinates of the detected obstacle minus step
  realCoord obstacleRealCoordsMinus = calcRealCoords(scannerRealCoords, currentAngle + scannerAngle, distanceM-STEP_GRID);
  //Transform real coordinates of the detected obstacle to grid coordinates
  Coord obstacleCoords = TransformRealToGridCoords(obstacleRealCoords);
  //Transform real coordinates of the detected obstacle to grid coordinates for minus step
  Coord obstacleCoordsMinus = TransformRealToGridCoords(obstacleRealCoordsMinus);
  //We check whether such an obstacle exists in the obstacle array.
  if(indexFindPointCoords(obstacleSet,obstacleSetPar, obstacleCoords)==-1) { //If the obstacle is new
    //Check if there is a fixed obstacle one step closer
    if(indexFindPointCoords(obstacleSet,obstacleSetPar, obstacleCoordsMinus) == -1) {
      String message = "New obstacle (grid): (" + String(obstacleCoords.y) + ", " + String(obstacleCoords.x) + ")";
      addToJournal(message.c_str()); // Add message to journal
      return 1; //New obstacle detected
    }
  }
  return 0;
}

//Reset previous distance covered for the route
void resetPreviousDistanceCoveredRoute() {
  prevDistanceCoveredRoute = 0;
}

//Add a point to the route
void addPointToRoute(realCoord realCoordsCurrent) {
  //Transform real coordinates to grid coordinates
  Coord routeCoords = TransformRealToGridCoords(realCoordsCurrent);
  if(indexFindPointCoords(routeSet, routeSetPar, routeCoords)==-1) { //If the route point is new
    AddCoords(routeSet, routeSetPar, routeCoords);
    //Not yet checked for the possibility of adding
  }
}

//Add a point to the route in movement
void addPointToRouteInMovement(realCoord realCoordsCurrent, int currentAngle, int currentDistanceCovered) {
  int step= STEP_GRID;
  if(isDiagonal(currentAngle)) step=DIAGONAL_STEP_GRID;

  if (currentDistanceCovered - prevDistanceCoveredRoute >= step)
  {
    prevDistanceCoveredRoute = currentDistanceCovered; // Update previous distance covered
    //Calculating real coordinates of the current point
    realCoord newRealCoords = calcRealCoords(realCoordsCurrent, currentAngle, currentDistanceCovered);
    //Add a point to the route
    addPointToRoute(newRealCoords);
  }
}

//Is the current angle diagonal
int isDiagonal(int currentAngle) {
  int normalizedAngle = normalizeAngle(currentAngle);
  return ((normalizedAngle % 90 >= 40) && (normalizedAngle % 90 <= 50)); // Diagonal if not a multiple of 90 degrees
}

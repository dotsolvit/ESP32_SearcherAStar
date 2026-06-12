//funcTransfCoordsAngles.hpp
//Functions for transforming coordinates, angles, etc.

//Transform real coordinates to grid coordinates
Coord TransformRealToGridCoords(realCoord &coord_in );

//Transform grid coordinates to real coordinates
realCoord TransformGridToRealCoords(Coord &coord_in );  

//Calculation of new current coordinates
realCoord calcRealCoords(realCoord &coord_in, int angle_in, int distance_mm);

//Calculation of the angle to the new path point
int calcAngleToNewPointPath(realCoord &coord_in, Coord &new_coord_in);

// normalization of the angles in degrees to the range [0, 360]
int normalizeAngle(int angle);

//difference in angles  (perameter= -1 CCW; 1 CW; 0 min)
int differenceInAngles(int a1, int a0, int parameter);

//Calculating the distance between nodes
int distanceBetweenPoints(realCoord &coord_in, Coord &new_coord_in);

//Processing measurement data to find and install obstacles.
int seekAndSetObstacle(realCoord realCoordsCurrent, int currentAngle, int scannerOffset, int scannerAngle, int distanceM);

//Processing measurement data to find obstacles.
int seekObstacle(realCoord realCoordsCurrent, int currentAngle, int currentDistanceCovered, int scannerOffset, int scannerAngle, int distanceM);

//Reset previous distance covered for the route
void resetPreviousDistanceCoveredRoute();

//Add a point to the route
void addPointToRoute(realCoord realCoordsCurrent);

//Add a point to the route in movement
void addPointToRouteInMovement(realCoord realCoordsCurrent, int currentAngle, int currentDistanceCovered);

//Is the current angle diagonal 
int isDiagonal(int currentAngle);
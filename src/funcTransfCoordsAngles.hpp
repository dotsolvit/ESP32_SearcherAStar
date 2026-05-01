//funcTransfCoordsAngles.hpp
//Функції перетворення координат, кутів та ін. (Functions for transforming coordinates, angles, etc.)

//Перетворити реальні координати на координати сітки(Transform real coordinates to grid coordinates)
Coord TransformRealToGridCoords(realCoord &coord_in );

//Перетворити координати сітки на реальні координати(Transform grid coordinates to real coordinates) 
realCoord TransformGridToRealCoords(Coord &coord_in );  

//Розрахунок нових поточних координат(Calculation of new current coordinates)
realCoord calcRealCoords(realCoord &coord_in, int angle_in, int dictance_mm);

//Розрахунок кута на нову точку шляху(Calculation of the angle to the new path point)
int calcAngleToNewPointPath(realCoord &coord_in, Coord &new_coord_in);

// normalization of the angles in degrees to the range [0, 360]
int normalizeAngle(int angle);

//difference in angles  (perameter= -1 CCW; 1 CW; 0 min)
int differenceInAngles(int a1, int a0, int parameter);

//Calculating the distance between nodes
int distanceBetweenPoints(realCoord &coord_in, Coord &new_coord_in);

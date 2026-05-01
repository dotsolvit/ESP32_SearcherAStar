//funcTransfCoordsAngles.cpp
//Функції перетворення координат, кутів та ін. (Functions for transforming coordinates, angles, etc.)

#include <Arduino.h>
#include "config.hpp"       
#include "funcTransfCoordsAngles.hpp"

//Перетворити реальні координати на координати сітки(Transform real coordinates to grid coordinates)
Coord TransformRealToGridCoords(realCoord &coord_in ){
  Coord coord_out;
  coord_out.y=int(coord_in.y/STEP_GRID);
  coord_out.x=int(coord_in.x/STEP_GRID);
  return coord_out;
}

//Перетворити координати сітки на реальні координати(Transform grid coordinates to real coordinates) 
realCoord TransformGridToRealCoords(Coord &coord_in ){
  realCoord coord_out;
  coord_out.y=coord_in.y*STEP_GRID + int( STEP_GRID/2 );
  coord_out.x=coord_in.x*STEP_GRID + int( STEP_GRID/2 );
  return coord_out;
}

//Розрахунок нових поточних координат(Calculation of new current coordinates)
realCoord calcRealCoords(realCoord &coord_in, int angle_in, int dictance_mm){
  realCoord coord_out = coord_in;
  float angle_rad = float(angle_in) * M_PI /180;
  float dY = - sin(angle_rad) * dictance_mm /10; //в см, минус т.к. у нас угол по часовой стрелке
  float dX = cos(angle_rad) * dictance_mm /10; //в см
  coord_out.y += int(dY);
  coord_out.x += int(dX);
  return coord_out;
}

//Розрахунок кута на нову точку шляху(Calculation of the angle to the new path point)
int calcAngleToNewPointPath(realCoord &coord_in, Coord &new_coord_in){
  int algle_out;
  realCoord new_coord = TransformGridToRealCoords(new_coord_in);
  //Визначення різниці координат:
  float dY=float(new_coord.y - coord_in.y);
  float dX=float(new_coord.x - coord_in.x);
  //Розрахунок кута (в радіанах):
  float angleRad = - atan2(dY, dX);//мінус т.к. у нас кут за годинниковою стрілкою(minus because our angle is clockwise)
  //Перевод в градусы:
  float angleDeg = angleRad * 180 / M_PI;
  algle_out = normalizeAngle(int(angleDeg));
  return algle_out;
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
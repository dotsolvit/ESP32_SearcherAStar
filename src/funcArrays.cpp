//funcArray.cpp

#include <Arduino.h>
#include "config.hpp"   
#include "funcArray.hpp"

//Для шляху та перешкod (одномірних матриць координат)
//For path and obstacles (one-dimensional coordinate matrices)

//Очистити одновимірну матрицю координат (Clear a one-dimensional coordinate matrix)
void ClearCoords(Coord *set, Par &setp) {
  set[0].x = 0;
  set[0].y = 0;
  setp.setSize = 0;
  setp.setSizeRealMax =0;
}
//Додати точку в одновимірну матрицю координат (Add a point to a one-dimensional coordinate matrix)
int AddCoords( Coord *set, Par &setp, Coord &c) {
  if(setp.setSize + 1 > setp.setSizeMax) return -1;
  setp.setSize += 1;
  set[setp.setSize-1]= c;
  if(setp.setSize>setp.setSizeRealMax) setp.setSizeRealMax = setp.setSize;
  return 0;
}
//Знайти точку з координатами в одновимірній матриці координат (Find a point with coordinates in a one-dimensional coordinate matrix)
int indexFindPointCoords( Coord *set, Par &setp, Coord &c) {
  if(setp.setSize == 0) return -1;
  //Поиск в списке
  int index =-1;
  for(int i=0; i < setp.setSize; i++){
    //Serial.print(i); Serial.println("(1)");
    if(set[i].y == c.y and set[i].x == c.x){
      index=i;
      break;
    }
  }
  return index;
}
//Видалити точку за індексом в одновимірній матриці координат (Remove a point by index in a one-dimensional coordinate matrix)
int RemoveCoords(Coord *set, Par &setp, int index) {
  if(setp.setSize == 0) return -1;
  if(index < 0 or index > setp.setSize -1) return -1;
  //Сдвиг точек
  for(int i=index+1; i<setp.setSize; i++){
    set[i-1] = set[i];
  }
  setp.setSize -=1;
  return 0;
}
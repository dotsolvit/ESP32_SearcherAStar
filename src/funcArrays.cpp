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


//Операції зі списком вузлів (Operations with the list of nodes)
//Додати вузол(Add a node)
int AddNode( Node *set, Par &setp, Node &n) {
  if(setp.setSize + 1 > setp.setSizeMax) return -1;
  setp.setSize += 1;
  set[setp.setSize-1]= n;
  if(setp.setSize>setp.setSizeRealMax) setp.setSizeRealMax = setp.setSize;
  return 0;
}
//Видалити вузол(Remove a node)
int RemoveNode( Node *set, Par &setp, Node &n) {
  if(setp.setSize == 0) return -1;
  //Поиск в списке
  int index =-1;
  for(int i=0; i < setp.setSize; i++){
    //Serial.print(i); Serial.println("(1)");
    if(set[i].coord.y == n.coord.y and set[i].coord.x == n.coord.x){
      index=i;
      break;
    }
  }
  if(index ==-1) return -1;
  //Зсув списку
  for(int i=index+1; i<setp.setSize; i++){
    //Serial.print(i); Serial.println("(2)");
    set[i-1] = set[i];
  }
  setp.setSize -=1;
  return 0;
}
//Знайти вузол з координатами(Find a node with coordinates)
int indexFindNode( Node *set, Par &setp, Coord &c) {
  if(setp.setSize == 0) return -1;
  //Пошук в списку (Search in the list)
  int index =-1;
  for(int i=0; i < setp.setSize; i++){
    //Serial.print(i); Serial.println("(1)");
    if(set[i].coord.y == c.y and set[i].coord.x == c.x){
      index=i;
      break;
    }
  }
  return index;
}
//Очистити список вузлів(Clear the list of nodes)
void ClearNodes( Node *set, Par &setp ) {
  set[0].coord.x = 0;
  set[0].coord.y = 0;
  set[0].cost = 0;
  setp.setSize = 0;
  setp.setSizeRealMax =0;
}
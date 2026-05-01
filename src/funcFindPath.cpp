//funcFindPath.cpp

#include <Arduino.h>
#include "config.hpp"
#include "funcFindPath.hpp"
#include "funcArray.hpp"

//Створюємо списки вузлів - reachableSet(досяжні вузли) і exploredSet(Це досліджені вузли)
Node reachableSet[MAX_REACHABLE_NODES]; // Это список достижимых узлов
Par reachableSetPar = {0, 0, MAX_REACHABLE_NODES};
Node exploredSet[MAX_EXPLORED_NODES]; // Это список исследованных узлов
Par exploredSetPar = {0, 0, MAX_EXPLORED_NODES};

extern Coord pathSet[MAX_PATH_LENGH]; //Эта одномерная матрица путb объявлен в PathfindingAStar_Nano.ino
extern Par pathSetPar;
extern Coord obstacleSet[MAX_OBSTACLE_LENGH]; //Эта одномерная матрица препятствй объявлена в PathfindingAStar_Nano.ino
extern Par obstacleSetPar;

//Пошук шляху за алгоритмом A*
ReturnCode findPath (Coord &start_coord, Coord &goal_coord){
  //0) reachableSet = []
  ClearNodes( reachableSet, reachableSetPar );
  //1) reachableSet = [start_node]
  Node start_node = {0};
  start_node.coord = start_coord; 
  if(AddNode(reachableSet, reachableSetPar, start_node ) == -1) return ReturnFindPath(-101);
  //2) explored = []
  ClearNodes( exploredSet, exploredSetPar );
  //3) while reachable is not empty:
  while(reachableSetPar.setSize > 0){
    //4) Choose some node we know how to reach. 
    Node node = choose_node(reachableSet, reachableSetPar, goal_coord);
    //5) If we just got to the goal node, build and return the path.
    if (node.coord.y == goal_coord.y and node.coord.x == goal_coord.x){
      return ReturnFindPath( build_path(start_node, node) );
    }
    //6) Don't repeat ourselves.
    //reachable.remove(node)
    if(RemoveNode(reachableSet, reachableSetPar, node) == -1) return ReturnFindPath(-102); //Удалить узел
    //explored.add(node)
    if(AddNode(exploredSet, exploredSetPar, node) == -1) return ReturnFindPath(-103); //Добавить узел

    //7) Доповнюємо список досяжних вузлів сполученими вузлами(We supplement the list of reachable nodes with connected nodes)
    //add in reachable = get_adjacent_nodes(node) - explored - reachable
    if(get_adjacent_nodes(node, start_coord) == -1) return ReturnFindPath(-104);
    

    //Контроль-печать списков 
    //Печатать список узлов
    //Serial.print("reachableSet: ");
    //printNodes(reachableSet, reachableSetPar);
    //Serial.print("exploredSet: ");
    //printNodes(exploredSet, exploredSetPar);
  }
  //If we get here, no path was found :(
  return ReturnFindPath(-105);
}

//Вибираю вузол із найменшою вартістю (Choose a node with the lowest cost)
Node choose_node(Node *set, Par &setp, Coord &goal_coord){
  Node best_node = set[0];
  int best_cost = set[0].cost+estimate_distance(best_node, goal_coord);
  for(int index=0; index < setp.setSize; index++){
    int current_cost = set[index].cost + estimate_distance(set[index], goal_coord) ;
    if(current_cost < best_cost){
      best_cost = current_cost;
      best_node = set[index];
    }
  }
  return best_node;
}

//build and return the path 
//Функція, яка будує шлях, слідуючи за посиланнями previous назад до початкового вузла
int build_path(Node& start_node, Node& goal_node){
  Coord poin, parent_point;
  //1)Очистить путь
  ClearCoords(pathSet, pathSetPar);
  //added node Добавляемый узел
  Node added_node=goal_node;
  while(true){
    poin = added_node.coord;
    if(AddCoords(pathSet, pathSetPar, poin)==-1) return -106;
    if(start_node.coord.y == added_node.coord.y and start_node.coord.x == added_node.coord.x) break;
    parent_point = added_node.parent_coord;
    int index = indexFindNode(exploredSet, exploredSetPar,parent_point);
    if(index==-1) return -107;
    added_node=exploredSet[index];
  }
  return 0;
}

//7) Доповнюємо список досяжних вузлів сполученими вузлами(We supplement the list of reachable nodes with connected nodes)
//add in reachable = get_adjacent_nodes(node) - explored - reachable
int get_adjacent_nodes(Node &node, Coord &start_coord){
  Coord c;
  Node nodeNew;
  for(int dy=-1; dy<2; dy++){
    for(int dx=-1; dx<2; dx++){
      if(dx==0 and dy==0) continue; //Центральную точку не смотрим, это node
      int y=dy+node.coord.y; 
      if(y<0) continue; //Отрицательные координаты - вне карты
      c.y= y;
      int x= dx+node.coord.x;
      if(x<0) continue; //Отрицательные координаты - вне карты
      c.x= x;
      if( IsThisNodePossible(c) == -1) continue; //Если узел не допустим
      //Если узел уже исследован (есть в explored)
      if(indexFindNode(exploredSet, exploredSetPar, c) != -1) continue;//Узел уже исследован

      //Визначаємо вартість нового вузла nodeN.cost
      nodeNew.cost = node.cost + STEP_COST;
      //Если node.parent_coord не стартовый узел
      if( !(node.parent_coord.y==start_coord.y and node.parent_coord.x==start_coord.x) ){
        //Перевіряємо напрямок та додаємо вартість повороту(Check the direction and add the cost of turning)
        if(IsSameDirection(node.parent_coord, node.coord, c)== -1) nodeNew.cost += TURN_COST;
      }

      //Якщо вузол у списку досяжних, то порівнюємо вартість із наявною(If the node is in the list of reachable nodes, then we compare the cost with the existing one)
      int index=indexFindNode(reachableSet, reachableSetPar, c);
      if(index !=-1){ //Узел уже в списке достижимых
        if(reachableSet[index].cost > nodeNew.cost){ //Якщо новий шлях коротший, то міняємо шлях та вартість(If the new path is shorter, then we change the path and cost)
          reachableSet[index].parent_coord = node.coord;
          reachableSet[index].cost = nodeNew.cost;
        }
      }
      else { //Такого вузла немає у списку досяжних(If there is no such node in the list of reachable nodes)
        //Додаємо вузол до списку досяжних вузлів(Add a node to the list of reachable nodes)
        nodeNew.coord = c;
        nodeNew.parent_coord = node.coord;
        if(AddNode(reachableSet, reachableSetPar, nodeNew) == -1) return -1; //Не вдалося додати вузол(Failed to add node)
      }
    }
  }
  return 0;
}

//Is This Node Possible?
//Перевіряємо на знаходження вузла та суміжних з ним вузлів:
// - усередині карти
// - поза перешкодами
int IsThisNodePossible(Coord &c){
  Coord cc;
  for(int dy=-1; dy<2; dy++){
    for(int dx=-1; dx<2; dx++){
      int y=dy+c.y; 
      if(y<0) return -1; //Негативні координати - поза картою(Negative coordinates - outside the map)
      cc.y= y;
      int x= dx+c.x;
      if(x<0) return -1; //Негативні координати - поза картою(Negative coordinates - outside the map)
      cc.x= x;
      if(cc.y > SIZE_MAP_Y-1) return -1; //Вне карты по Y
      if(cc.x > SIZE_MAP_X-1) return -1; //Вне карты по X
      if(indexFindPointCoords(obstacleSet, obstacleSetPar, cc) != -1 ) return -1;   //У цій точці є перешкода(There is an obstacle at this point)
    }
  }
  return 0;
}

//Функція, що оцінює відстань від поточного до кінцевого вузла(Function that estimates the distance from the current node to the goal node)
int estimate_distance(Node &node, Coord &goal_coord){
  long dy=node.coord.y - goal_coord.y;
  dy=dy*dy;
  long dx=node.coord.x - goal_coord.x;
  dx=dx*dx;
  dy=dy+dx;
  double value = sqrt(dy) * STEP_COST;
  int maxValue = int(value);
  return maxValue;
  /*
  Проста евристика 
  Euclidean distance:  h(n) = sqrt((x1 - x2)^2 + (y1 - y2)^2)
  */
}

//Код повернення(Return code)
ReturnCode ReturnFindPath(int code){
  ReturnCode ret_code;
  ret_code.return_code=code;
  ret_code.reachableSet=reachableSetPar.setSizeRealMax;
  ret_code.exploredSet=exploredSetPar.setSizeRealMax;;
  return ret_code;
}

//Compress Path:
void compressPath(void){
  if(pathSetPar.setSize < 3) return; //Нема чого стискати(Nothing to compress)
  int index=pathSetPar.setSize-1;
  while(true){
    //Три точки на одній лінії? (Three points on the same line?)
    if(IsSameDirection(pathSet[index], pathSet[index-1], pathSet[index-2]) == 0 ) {
       //Удаляем точку с индексом index-1
       if(RemoveCoords( pathSet, pathSetPar, index-1 ) == 0){
         Serial.print("The Path Point has been removed. Index="); //Якщо вдалося видалити(If the point was successfully removed)
         Serial.println(index-1);
       }
       else{
         Serial.print("The Path Point was not deleted! Index="); //Якщо не вдалося видалити(If the point was not deleted)
         Serial.println(index-1);
       }
    }
    index -=1;
    if(index <= 1) break; //Нема чого стискати(Nothing to compress)
  }
  return;
}

//Directions

//Функція визначення простого напрямку з А в (градуси за годинниковою стрілкою):
//Function to determine simple direction from A to B (degrees clockwise):
int Direction(Coord &a, Coord &b){
  int angleInDegrees;
  if(a.y==b.y and a.x<b.x) angleInDegrees=0;
  else if(a.y==b.y and a.x>b.x) angleInDegrees=180;
  else if(a.y<b.y and a.x==b.x) angleInDegrees=270; //-90
  else if(a.y>b.y and a.x==b.x) angleInDegrees=90;
  else if(a.y<b.y and a.x<b.x) angleInDegrees=315; //-45
  else if(a.y>b.y and a.x>b.x) angleInDegrees=135; //90+45
  else if(a.y<b.y and a.x>b.x) angleInDegrees=225; //-90-45
  else if(a.y>b.y and a.x<b.x) angleInDegrees=45;
  else angleInDegrees=0;
  return angleInDegrees;
}

//Is this the same direction?
int IsSameDirection(Coord &a, Coord &b, Coord &c){
 if(Direction(a, b) == Direction(b, c) ) return 0;
 return -1;
}


//Тест функції напрямку (Test of the direction function)
void testDirection(){
  Coord a, b;
  a={5, 9};
  Serial.println("TEST:");

  for(int dy=1; dy > -2; dy--){
    b.y=a.y+dy;
    Serial.print(b.y); Serial.print("|");
    for(int dx=-1; dx<2; dx++){
      b.x=a.x+dx;
      Serial.print(Direction(a,b)); Serial.print(",");
    }
    Serial.println(" ");
  }
}

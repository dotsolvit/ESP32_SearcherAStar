//funcFindPath.cpp

#include <Arduino.h>
#include "config.hpp"
#include "funcFindPath.hpp"
#include "funcArray.hpp"

//Node lists - reachableSet (reachable nodes) and exploredSet (explored nodes) - from main.cpp
extern Node reachableSet[MAX_REACHABLE_NODES]; // This is a list of reachable nodes
extern Par reachableSetPar;
extern Node exploredSet[MAX_EXPLORED_NODES]; // This is a list of explored nodes
extern Par exploredSetPar;

extern Coord pathSet[MAX_PATH_LENGH]; // This is a one-dimensional matrix of the path declared in main.cpp
extern Par pathSetPar;
extern Coord obstacleSet[MAX_OBSTACLE_LENGH]; // This is a one-dimensional matrix of obstacles declared in main.cpp
extern Par obstacleSetPar;

//Path search using the A* algorithm
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
    if(RemoveNode(reachableSet, reachableSetPar, node) == -1) return ReturnFindPath(-102); //Delete node
    //explored.add(node)
    if(AddNode(exploredSet, exploredSetPar, node) == -1) return ReturnFindPath(-103); //Add a node

    //7)We supplement the list of reachable nodes with connected nodes
    //add in reachable = get_adjacent_nodes(node) - explored - reachable
    if(get_adjacent_nodes(node, start_coord) == -1) return ReturnFindPath(-104);
    
    /* //WRM
    if(exploredSetPar.setSize < 10) {
      wrmPrintListOfNodes();     //Control-printing of lists 
    }
    */
  }
  //If we get here, no path was found :(
  return ReturnFindPath(-105);
}

//Choose a node with the lowest cost
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
//Function that builds the path by following the previous pointers back to the starting node
int build_path(Node& start_node, Node& goal_node){
  Coord poin, parent_point;
  //1)Clear the waypoint list (pathSet) and its parameters (pathSetPar)
  ClearCoords(pathSet, pathSetPar);
  //added node = goal node
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

//7)We supplement the list of reachable nodes with connected nodes
//add in reachable = get_adjacent_nodes(node) - explored - reachable
int get_adjacent_nodes(Node &node, Coord &start_coord){
  Coord c;
  Node nodeNew;
  for(int dy=-1; dy<2; dy++){
    for(int dx=-1; dx<2; dx++){
      if(dx==0 and dy==0) continue; //We don't look at the central point, this is the node
      int y=dy+node.coord.y; 
      if(y<0) continue; //Negative coordinates - outside the map
      c.y= y;
      int x= dx+node.coord.x;
      if(x<0) continue; //Negative coordinates - outside the map
      c.x= x;
      if( IsThisNodePossible(c) == -1) continue; //If the node is not valid
      //If the node is already explored (is in explored)
      if(indexFindNode(exploredSet, exploredSetPar, c) != -1) continue;//Node already explored

      //We determine the cost of the new node nodeN.cost
      nodeNew.cost = node.cost + STEP_COST;
      //If the node and node.parent_coord are not the starting node
      if(!(node.coord.y==start_coord.y and node.coord.x==start_coord.x)){
        if( !(node.parent_coord.y==start_coord.y and node.parent_coord.x==start_coord.x) ){
          //Check the direction and add the cost of turning
          if(IsSameDirection(node.parent_coord, node.coord, c)== -1) nodeNew.cost += TURN_COST;
        }
      }

      //If the node is in the list of reachable nodes, then we compare the cost with the existing one
      int index=indexFindNode(reachableSet, reachableSetPar, c);
      if(index !=-1){ //The node is already in the reachable list.
        if(reachableSet[index].cost > nodeNew.cost){ //If the new path is shorter, then we change the path and cost
          reachableSet[index].parent_coord = node.coord;
          reachableSet[index].cost = nodeNew.cost;
        }
      }
      else { //If there is no such node in the list of reachable nodes
        //Add a node to the list of reachable nodes
        nodeNew.coord = c;
        nodeNew.parent_coord = node.coord;
        if(AddNode(reachableSet, reachableSetPar, nodeNew) == -1) return -1; //Failed to add node
      }
    }
  }
  return 0;
}

//Is This Node Possible?
//Checks for the location of the node and its adjacent nodes:
// - inside the map
// - outside the obstacles)
int IsThisNodePossible(Coord &c){
  Coord cc;
  for(int dy=-1; dy<2; dy++){
    for(int dx=-1; dx<2; dx++){
      int y=dy+c.y; 
      if(y<0) return -1; //Negative coordinates - outside the map
      cc.y= y;
      int x= dx+c.x;
      if(x<0) return -1; //Negative coordinates - outside the map
      cc.x= x;
      if(cc.y > SIZE_MAP_Y-1) return -1; //Outside the map along the Y-axis
      if(cc.x > SIZE_MAP_X-1) return -1; //Outside the map along the X-axis
      if(indexFindPointCoords(obstacleSet, obstacleSetPar, cc) != -1 ) return -1;   //There is an obstacle at this point
    }
  }
  return 0;
}

//Function that estimates the distance from the current node to the goal node
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

//Return code
ReturnCode ReturnFindPath(int code){
  ReturnCode ret_code;
  ret_code.return_code=code;
  ret_code.reachableSet=reachableSetPar.setSizeRealMax;
  ret_code.exploredSet=exploredSetPar.setSizeRealMax;;
  return ret_code;
}

//Compress Path:
void compressPath(void){
  if(pathSetPar.setSize < 3) return; //Nothing to compress
  int index=pathSetPar.setSize-1;
  while(true){
    //Three points on the same line?
    if(IsSameDirection(pathSet[index], pathSet[index-1], pathSet[index-2]) == 0 ) {
       //Removing the point with index-1
       if(RemoveCoords( pathSet, pathSetPar, index-1 ) == 0){
         Serial.print("The Path Point has been removed. Index="); //If the point was successfully removed
         Serial.println(index-1);
       }
       else{
         Serial.print("The Path Point was not deleted! Index="); //If the point was not deleted
         Serial.println(index-1);
       }
    }
    index -=1;
    if(index <= 1) break; //Nothing to compress
  }
  return;
}

//Directions

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


//Test of the direction function
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

/*
//Control-printing of lists
void wrmPrintListOfNodes(){
    Serial.print("reachableSet: ");
    wrmPrintNodes(reachableSet, reachableSetPar);
    Serial.print("exploredSet: ");
    wrmPrintNodes(exploredSet, exploredSetPar);
}
//Print the list of nodes
void wrmPrintNodes(Node *set, Par &setp){
   if(setp.setSize>0){
   for(int i=0; i<setp.setSize; i++){
    Serial.print(i); Serial.print("|");
    Serial.print("y"); Serial.print(set[i].coord.y);
    Serial.print(",x"); Serial.print(set[i].coord.x);
    Serial.print(",c"); Serial.print(set[i].cost);
    Serial.print(","); Serial.print(set[i].parent_coord.y);
    Serial.print(","); Serial.print(set[i].parent_coord.x);
    Serial.print("|");
   }
   Serial.println("\n============");
  }
}*/
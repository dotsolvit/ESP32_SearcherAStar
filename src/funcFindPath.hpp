//funcFindPath.hpp

//Path search using the A* algorithm
ReturnCode findPath (Coord &start_coord, Coord &goal_coord);

//Choose a node with the lowest cost
Node choose_node(Node *set, Par &setp, Coord &goal_coord);

//build and return the path
//Function that builds the path by following the previous pointers back to the initial node
int build_path(Node& start_node, Node& goal_node);

//We supplement the list of reachable nodes with connected nodes
//add in reachable = get_adjacent_nodes(node) - explored - reachable
int get_adjacent_nodes(Node &node, Coord &start_coord);

//Is This Node Possible?
//We check for the location of the node and its adjacent nodes:
// - inside the map
// - outside the obstacles
int IsThisNodePossible(Coord &c);   

//Function that estimates the distance from the current node to the goal node
int estimate_distance(Node &node, Coord &goal_coord);

//Return code
ReturnCode ReturnFindPath(int code);    

//Compress Path:
void compressPath(void);    

//Directions

//Function to determine simple direction from A to B (degrees clockwise):
int Direction(Coord &a, Coord &b);

//Function to determine if three points are on the same line
int IsSameDirection(Coord &a, Coord &b, Coord &c);  

//Is this node possible? (Is this node possible?)
int IsThisNodePossible(Coord &c);   

//Test of the direction function
void testDirection();

/*
//WRM
//WRM print list of nodes (WRM - print list of nodes)
void wrmPrintListOfNodes(void);
//WRM print nodes (WRM - print nodes)
void wrmPrintNodes(Node *set, Par &setp);
*/

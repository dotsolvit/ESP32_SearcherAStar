//funcArray.hpp

//For path and obstacles (one-dimensional coordinate matrices)
//Clear a one-dimensional coordinate matrix
void ClearCoords(Coord *set, Par &setp);

//Add a point to a one-dimensional coordinate matrix
int AddCoords( Coord *set, Par &setp, Coord &c);

//Find a point with coordinates in a one-dimensional coordinate matrix
int indexFindPointCoords( Coord *set, Par &setp, Coord &c);

//Remove a point by index in a one-dimensional coordinate matrix
int RemoveCoords(Coord *set, Par &setp, int index);

//Operations with the list of nodes
//Add a node 
int AddNode( Node *set, Par &setp, Node &n);

//Remove a node
int RemoveNode( Node *set, Par &setp, Node &n); 

//Find a node with coordinates
int indexFindNode( Node *set, Par &setp, Coord &c);

//Clear a list of nodes
void ClearNodes(Node *set, Par &setp);  

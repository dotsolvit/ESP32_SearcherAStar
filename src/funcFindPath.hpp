//funcFindPath.hpp

//Пошук шляху за алгоритмом A*
ReturnCode findPath (Coord &start_coord, Coord &goal_coord);

//Вибираю вузол із найменшою вартістю (Choose a node with the lowest cost)
Node choose_node(Node *set, Par &setp, Coord &goal_coord);

//build and return the path
//Функція, яка будує шлях, слідуючи за посиланнями previous назад до початкового вузла
int build_path(Node& start_node, Node& goal_node);

//Доповнюємо список досяжних вузлів сполученими вузлами(We supplement the list of reachable nodes with connected nodes)
//add in reachable = get_adjacent_nodes(node) - explored - reachable
int get_adjacent_nodes(Node &node, Coord &start_coord);

//Is This Node Possible?
//Перевіряємо на знаходження вузла та суміжних з ним вузлів:
// - усередині карти
// - поза перешкодами
int IsThisNodePossible(Coord &c);   

//Функція, що оцінює відстань від поточного до кінцевого вузла(Function that estimates the distance from the current node to the goal node)
int estimate_distance(Node &node, Coord &goal_coord);

//Код повернення(Return code)
ReturnCode ReturnFindPath(int code);    

//Compress Path:
void compressPath(void);    

//Directions

//Функція визначення простого напрямку з А в (градуси за годинниковою стрілкою):
//Function to determine simple direction from A to B (degrees clockwise):
int Direction(Coord &a, Coord &b);

//Функція визначення, чи три точки на одній лінії (Function to determine if three points are on the same line)
int IsSameDirection(Coord &a, Coord &b, Coord &c);  

//Is this node possible? (Is this node possible?)
int IsThisNodePossible(Coord &c);   

//ест функції напрямку (Test of the direction function)
void testDirection();


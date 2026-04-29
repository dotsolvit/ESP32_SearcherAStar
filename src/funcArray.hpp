//funcArray.hpp

//Для шляху та перешкod (одномірних матриць координат)
//For path and obstacles (one-dimensional coordinate matrices)

//Очистити одновимірну матрицю координат (Clear a one-dimensional coordinate matrix)
void ClearCoords(Coord *set, Par &setp);

//Додати точку в одновимірну матрицю координат (Add a point to a one-dimensional coordinate matrix)
int AddCoords( Coord *set, Par &setp, Coord &c);

//Знайти точку з координатами в одновимірній матриці координат (Find a point with coordinates in a one-dimensional coordinate matrix)
int indexFindPointCoords( Coord *set, Par &setp, Coord &c);

//Видалити точку за індексом в одновимірній матриці координат (Remove a point by index in a one-dimensional coordinate matrix)
int RemoveCoords(Coord *set, Par &setp, int index);
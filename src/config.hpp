//config.hpp
//Заголовочний файл для зберігання конфігураційних налаштувань та констант

// Для більшості плат ESP32 вбудований світлодіод на 2 піні
#define LED_BUILTIN 2

#define STEP_GRID 10 //шаг сетки карты 10см
//Размер карты 55*35
#define SIZE_MAP_Y 55  //55
#define SIZE_MAP_X 35  //35

#define MAX_PATH_LENGH 100       //Максимум координат в pathSet
#define MAX_OBSTACLE_LENGH 150   //Максимум координат в obstacleSet

//Структура координат сетки
struct Coord {
  uint8_t y, x;     // Координаты (0-255)
};
//Структура реальных координат
struct realCoord {
  int y, x;     // Координаты реальные
};

//Параметры списка узлов (parameters)
struct Par {
  int setSize;
  int setSizeRealMax;
  int setSizeMax; 
};


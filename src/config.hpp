//config.hpp
//Заголовочний файл для зберігання конфігураційних налаштувань та констант

// Для більшості плат ESP32 вбудований світлодіод на 2 піні
#define LED_BUILTIN 2

#define BUZ_PIN 4 //Buzzer
//ADC1 GPIO 34 – IR sensor
#define PIN_IR 34
//ADC1 GPIO 35 – battery control
#define PIN_BAT 35

#define STEP_COST 50
#define TURN_COST 1
#define STEP_GRID 10 //шаг сетки карты 10см
//Размер карты 55*35
#define SIZE_MAP_Y 55  //55
#define SIZE_MAP_X 35  //35

#define MAX_REACHABLE_NODES 100  //Максимум узлов в reachableSet
#define MAX_EXPLORED_NODES 300   //Максимум узлов в exploredSet
#define MAX_PATH_LENGH 100       //Максимум координат в pathSet
#define MAX_OBSTACLE_LENGH 150   //Максимум координат в obstacleSet


#define RESISTOR_RATIO 3.119 // (9.81k + 4.63k) / 4.63k делитель напряжения 

//Display OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//Структура координат сітки
struct Coord {
  uint8_t y, x;     // Координаты (0-255)
};
//Структура реальних координат
struct realCoord {
  int y, x;     // Координаты реальные
};

//Параметри списку вузлів чи координат(parameters)
struct Par {
  int setSize;
  int setSizeRealMax;
  int setSizeMax; 
};

//Структура вузла (Node)
struct Node {
    Coord coord;     // Координаты y,x (0-255)
    uint16_t cost;   // Стоимость пути от старта
    Coord parent_coord; // Координаты восстановления пути
};

//Код повернення функції пошуку шляху Return code of the pathfinding function
struct ReturnCode {
  int return_code;
  int reachableSet;
  int exploredSet;
};


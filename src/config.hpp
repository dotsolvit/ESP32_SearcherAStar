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

//interrupt:
//GPIO 32 – optical sensor L
//GPIO 33 – optical sensor R
#define IRSENSOR_LEFT_PIN 32
#define IRSENSOR_RIGHT_PIN 33

//distance per impulse cm
#define DISTANCE_PER_IMPULSE 1.028  // cm/pulse

//Coefficients of the approximating equation distance = A * x ^ B
//5,0V :
//#define COEFFICIENT_A 22261
//#define COEFFICIENT_B -0.8476
//4,8V :
#define COEFFICIENT_A 24292
#define COEFFICIENT_B -0.8586

// PWM settings for motors
#define MOTORS_PWM_FREQ 5000
#define MOTORS_PWM_RESOLUTION 8

// Define motor driver pins
#define MOTOR_L_A_Pin 16 //in1
#define MOTOR_L_B_Pin 17 //in2
#define MOTOR_R_A_Pin 19 //in3
#define MOTOR_R_B_Pin 18 //in4
#define MOTOR_L_A_Channel 0
#define MOTOR_L_B_Channel 1 
#define MOTOR_R_A_Channel 2
#define MOTOR_R_B_Channel 3 
//speed adjustment between motors:
#define SPEED_ADJUSTMENT 15  //17

//SERVO
//servo with min/max pulse widths:
#define SERVO_MIN_PULSE 350
#define SERVO_MAX_PULSE 2400
#define SERVO_MAX_ANGLE 86 //Maximum rotation angle from the center position - 0 +
#define PIN_SERVO 23 //GPIO 23 servo

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


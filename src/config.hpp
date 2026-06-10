//config.hpp
//Заголовочний файл для зберігання конфігураційних налаштувань та констант

// Для більшості плат ESP32 вбудований світлодіод на 2 піні
#define LED_BUILTIN 2

#define BUZ_PIN 4 //Buzzer

//Ultrasonic sensor pins
#define TRIG_PIN 27
#define ECHO_PIN 34
#define ULTRASONIC_MAX_DISTANCE 120 //the maximum distance that the ultrasonic sensor can measure (in centimeters)
#define ULTRASONIC_MIN_DISTANCE 20  //the minimum distance that the ultrasonic sensor can measure (in centimeters)
#define ULTRASONIC_MAX_TIMEOUT 10000 //timeout for pulseIn in microseconds (170cm * 58us/cm)

//Scanner
#define SCANNING_PERIOD 200 //scanning period 200 ms
#define SCANNING_ADDITIONAL_PERIOD 25 //scanning additional period 25 ms
#define SCANNING_ANGLE_STEP 6 //scanning angle step 6 degrees 
#define SCANNER_OFFSET 6 //offset for scanner (in centimeters)
#define NARROW_SCANNING_DISTANCE 80 //cm
#define NARROW_SCANNING_ANGLE_STEP 6 //narrow scanning angle step 6 degrees

//IR distance sensor
//ADC1 GPIO 34 – IR sensor
#define PIN_IR 34
#define IR_MAX_DISTANCE 150 //the maximum distance that the IR sensor can measure (in centimeters)
#define IR_MIN_DISTANCE 20  //the minimum distance that the IR sensor can measure (in centimeters)
//Coefficients of the approximating equation distance = A * x ^ B
//5,0V :
//#define COEFFICIENT_A 22261
//#define COEFFICIENT_B -0.8476
//4,8V :
#define COEFFICIENT_A 24292
#define COEFFICIENT_B -0.8586

//Battery voltage control
//ADC1 GPIO 35 – battery control
#define PIN_BAT 35
//Battery monitoring period (in milliseconds)
#define BATTERY_MONITORING_PERIOD 2000
//EMA filtering coefficient (from 0.0 to 1.0)
#define EMA_FILTERING_COEFFICIENT 0.25 

#define STEP_COST 50
#define TURN_COST 1
#define STEP_GRID 10 //шаг сетки карты 10см
#define DIAGONAL_STEP_GRID 14 //шаг сетки по диагонали (10 * sqrt(2))

//Размер карты 55*35
#define SIZE_MAP_Y 55  //55
#define SIZE_MAP_X 35  //35

#define MAX_REACHABLE_NODES 150  //Maximum nodes in reachableSet //100
#define MAX_EXPLORED_NODES 500   //Maximum nodes in exploredSet
#define MAX_PATH_LENGH 150       //Maximum coordinates in pathSet //100
#define MAX_OBSTACLE_LENGH 400   //Maximum coordinates in obstacleSet //150
#define MAX_ROUTE_LENGH 200      //Maximum coordinates in routeSet //150

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

//odometer - distance per impulse cm
#define DISTANCE_PER_IMPULSE 1.028  // cm/pulse



// PWM settings for motors
#define MOTORS_PWM_FREQ 5000
#define MOTORS_PWM_RESOLUTION 8

// Define motor driver pins
#define MOTOR_L_A_Pin 16 //in1
#define MOTOR_L_B_Pin 17 //in2
#define MOTOR_R_A_Pin 19 //in3
#define MOTOR_R_B_Pin 18 //in4
#define MOTOR_L_A_Channel 10
#define MOTOR_L_B_Channel 11 
#define MOTOR_R_A_Channel 12
#define MOTOR_R_B_Channel 13 
//speed adjustment between motors:
#define SPEED_ADJUSTMENT 8  //15 //17
//speed settings:
#define SPEED_NORMAL 220
#define SPEED_SLOW 170  //180
#define SPEED_SLOW_TURN 200
#define DISTANCE_SPEED_SLOW 20 //cm  //15
//braking distance in cm
#define BRAKING_DISTANCE 5


//SERVO
//servo with min/max pulse widths:
#define SERVO_MIN_PULSE 450  //350
#define SERVO_MAX_PULSE 2390 //2400
#define SERVO_MAX_ANGLE 90 //Maximum rotation angle from the center position - 0 + 90 degrees
#define PIN_SERVO 23 //GPIO 23 servo
#define SERVO_Channel 14
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION 16



//Tank signals:
#define SIGNAL_GO 0
#define SIGNAL_NOPATH 1
#define SIGNAL_OBSTACLE 2

//Jurnal:
#define JOURNAL_SIZE 150 //Maximum number of entries in the journal
#define JOURNAL_MESSAGE_LENGTH 50 //Maximum length of a message in the journal

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


//Project: ESP32_SearcherAStar
//main.cpp  
//

#include <Arduino.h>
#include <Wire.h>
#include "config.hpp" 
#include "funcDrive.hpp"
#include "funcWiFiWeb.h"
#include "funcOLED.hpp"
#include "funcMPU6050.hpp"

//Create arrays for shared use in tasks (Створюємо масиви для спільного використання у завданнях)
//Створюємо шлях (як масив координат та його параметри) 
//Create a path (as an array of coordinates and its parameters)
volatile Coord pathSet[MAX_PATH_LENGH];
volatile Par pathSetPar = {0, 0, MAX_PATH_LENGH};

//Створюємо масив перешкоди у вигляді одномірної матриці координат (Create an obstacle array as a one-dimensional matrix of coordinates)
volatile Coord obstacleSet[MAX_OBSTACLE_LENGH];
volatile Par obstacleSetPar = {0, 0, MAX_OBSTACLE_LENGH};

//Real Coordinates
volatile realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель

//Стврюємо мютекс для синхронізації доступу до спільних масивів (Create a mutex for synchronizing access to shared matrixes) 
SemaphoreHandle_t xMutex;
//

QueueHandle_t toWebQueue, toDriveQueue;   // створюємо дві черги: toWeb і toDive (two FreeRTOS queues: toWeb and toDrive)

TaskHandle_t driveTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

volatile int currentAngle, displayed_currentAngle ; //Текущий угол по Х

//Distance covered counters:
volatile int distancePulseCounterLeft = 0;
volatile int distancePulseCounterRight = 0;

// Функция-обработчик прерывания (ISR)
void IRAM_ATTR handlePulseLeft() {
  distancePulseCounterLeft++;
}
void IRAM_ATTR handlePulseRight() {
  distancePulseCounterRight++;
}

//FreeRTOS tasks Drive:
void driveTask(void *pvParameters) {    // функція задачі FreeRTOS (task function)
  (void) pvParameters;                  // ігноруємо вхідні параметри

   
  initDisplay(); //Display initialization
  //Init Buzzer:
  initBuzzer();

  initRealCoords(); //Init Real Coordinates
  initializationObstacleSet();  //Init Obstacle Set
  initStage(); //Init Stage
  
  displayMessage(1, "Waiting 10 seconds..", 0, "");
  initMPU6050(); //Init MPU6050
  vTaskDelay(10000 / portTICK_PERIOD_MS);       // пауза 10 seconds (delay 10 seconds, lets other tasks run)
  displayMessage(1, "OK", 0, "");

  //Display distance to obstacles
  displayDistance();
  //Display angle
  displayAngle( getAngleX() );
  //Display battery
  displayBattery();


  
  bool pr_show = true; // прапорець для виводу залишку стека (flag for printing stack high water mark)  
  while (true) {  // безкінечний цикл задачі

    cycleDrive(); // виконуємо основну логіку керування (execute main drive logic)



    vTaskDelay(100 / portTICK_PERIOD_MS); // затримка 100 мс для зменшення навантаження (poll every 100ms)
    
    if(pr_show) { // перевіряємо скільки залишилось слів в стеку (check of stack remain)
      Serial.println("uxTaskGetStackHighWaterMark(driveTaskHandle): " + String(uxTaskGetStackHighWaterMark(driveTaskHandle))); // виводимо залишок стека (print stack high water mark)
       pr_show = false; // вимикаємо прапорець після першого виводу (disable flag after first print)}
    }
    
    if(uxTaskGetStackHighWaterMark(driveTaskHandle) < 500) { // перевіряємо, чи залишилось менше 500 слів стека (check if less than 100 words of stack remain)
      Serial.println("Warning: driveTask stack is running low!"); // виводимо попередження (print warning)
    } 
  }
}

//FreeRTOS tasks Web:
void webTask(void *pvParameters) {    // функція задачі FreeRTOS (task function)
  (void) pvParameters;                  // ігноруємо вхідні параметри
  
  setupWiFi(); //Connecting to WiFi in STA mode

  Web_starting(); //Starting the web server
  
  bool pr_show = true; // прапорець для виводу залишку стека (flag for printing stack high water mark)  
  while (true) {  // безкінечний цикл задачі
                         
    handle_Client();  //Processing client requests

    //vTaskDelay(100 / portTICK_PERIOD_MS); // затримка 100 мс для зменшення навантаження (poll every 100ms)
    
    if(pr_show) { // перевіряємо скільки залишилось слів в стеку (check of stack remain)
      Serial.println("uxTaskGetStackHighWaterMark(webTaskHandle): " + String(uxTaskGetStackHighWaterMark(webTaskHandle))); // виводимо залишок стека (print stack high water mark)
       pr_show = false; // вимикаємо прапорець після першого виводу (disable flag after first print)}
    }
    
    if(uxTaskGetStackHighWaterMark(webTaskHandle) < 500) { // перевіряємо, чи залишилось менше 500 слів стека (check if less than 100 words of stack remain)
      Serial.println("Warning: webTask stack is running low!"); // виводимо попередження (print warning)
    } 
  }
}


void setup() {
  Serial.begin(115200); // ініціалізуємо Serial для відладки (init Serial monitor)
  Wire.begin();

  //Set interrupt////////////
  // 
  pinMode(IRSENSOR_LEFT_PIN, INPUT);
  pinMode(IRSENSOR_RIGHT_PIN, INPUT);
  //FALLING : HIGH to LOW)
  attachInterrupt(digitalPinToInterrupt(IRSENSOR_LEFT_PIN), handlePulseLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(IRSENSOR_RIGHT_PIN), handlePulseRight, FALLING);
  //Ddistance covered counters:
  distancePulseCounterLeft = 0;
  distancePulseCounterRight = 0;
  Serial.println("Interrupts are initialized");
  ////////////////////////////

  // створюємо мютекс (create mutex)
  xMutex = xSemaphoreCreateMutex(); 

  // створюємо дві черги на 5 елементів типу byte кожна (create two queues, 5 elements each)
  toWebQueue = xQueueCreate(5, sizeof(byte));
  toDriveQueue  = xQueueCreate(5, sizeof(byte));
  
  //Create FreeRTOS tasks
  //Priority 2 for drive and 1 for web to give more priority to drive task (Пріоритет 2 для drive і 1 для web, щоб надати більший пріоритет задачі drive)
  //Create a webserver task pinned to core 0 
  xTaskCreatePinnedToCore(driveTask, "driveTask", 4096, NULL, 2, &driveTaskHandle, 0);

  //Create a webserver task pinned to core 1
  xTaskCreatePinnedToCore(webTask, "webTask", 4096, NULL, 1, &webTaskHandle, 1);
}
  


void loop() {
  // Empty because FreeRTOS scheduler runs the tasks (порожній, бо FreeRTOS керує виконанням задач)
}


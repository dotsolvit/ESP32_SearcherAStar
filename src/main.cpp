/*********************************************************************
 * @brief Searcher AStar on ESP32
 * @author Dotsenko Oleksandr
 * @date 2026-06-12
 * https://github.com/dotsolvit/ESP32_SearcherAStar
**********************************************************************/
//Project: ESP32_SearcherAStar
//main.cpp  
//

#include <Arduino.h>
#include <Wire.h>
#include "config.hpp" 
#include "funcDrive.hpp"
#include "funcWiFiWeb.hpp"
#include "funcOLED.hpp"
#include "funcMPU6050.hpp"
#include "funcMotors.hpp"
#include "funcSensors.hpp"

//Creating node lists - reachableSet (reachable nodes) and exploredSet (explored nodes)
Node reachableSet[MAX_REACHABLE_NODES]; // This is a list of reachable nodes
Par reachableSetPar = {0, 0, MAX_REACHABLE_NODES};
Node exploredSet[MAX_EXPLORED_NODES]; // This is a list of explored nodes
Par exploredSetPar = {0, 0, MAX_EXPLORED_NODES};

//Create arrays for shared use in tasks 
//Create a path (as an array of coordinates and its parameters)
volatile Coord pathSet[MAX_PATH_LENGH];
volatile Par pathSetPar = {0, 0, MAX_PATH_LENGH};

//Create an obstacle array as a one-dimensional matrix of coordinates
volatile Coord obstacleSet[MAX_OBSTACLE_LENGH];
volatile Par obstacleSetPar = {0, 0, MAX_OBSTACLE_LENGH};

//Create a route array
volatile Coord routeSet[MAX_ROUTE_LENGH];
volatile Par routeSetPar = {0, 0, MAX_ROUTE_LENGH};

//Real Coordinates
volatile realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель

//Create a mutex for synchronizing access to shared matrixes
SemaphoreHandle_t xMutex;
//

QueueHandle_t toWebQueue, toDriveQueue;   //Two FreeRTOS queues: toWeb and toDrive

TaskHandle_t driveTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;

volatile int currentAngle, displayed_currentAngle ; // Current X-angle

//Jurnal:
char journal[JOURNAL_SIZE][JOURNAL_MESSAGE_LENGTH]; //Journal of messages
int journalIndex = 0; //Index for adding messages to the journal
unsigned long journalInitTime = 0;

//Distance covered counters:
volatile int distancePulseCounterLeft = 0;
volatile int distancePulseCounterRight = 0;

// Interrupt handler function (ISR)
void IRAM_ATTR handlePulseLeft() {
  distancePulseCounterLeft++;
}
void IRAM_ATTR handlePulseRight() {
  distancePulseCounterRight++;
}

//FreeRTOS tasks Drive:
void driveTask(void *pvParameters) {    
  (void) pvParameters;                  // Ignores input parameters
  
  //Init EEPROM:
  initEEPROM() ;
  //Servo intialization
  initServo();
  //Sonic initialization
  //initEcho();
   
  initDisplay(); //Display initialization
  //Init Buzzer:
  initBuzzer();

  initRealCoords(); //Init Real Coordinates
  initializationObstacleSet();  //Init Obstacle Set
  initializationRouteSet(); //Init Route Set
  initStage(); //Init Stage
  
  displayMessage(1, "Waiting 10 seconds..", 0, "");
  initMPU6050(); //Init MPU6050
  vTaskDelay(10000 / portTICK_PERIOD_MS);   //Pause for 10 seconds for MPU6050 to stabilize
  displayMessage(1, "OK", 0, "");

  //Display distance to obstacles
  displayDistance();
  //Display angle
  displayAngle( getAngleX() );
  //Display battery
  displayBattery();

  //Motor initialization
  initializationMotors();
  Serial.println("Motors are initialized");

  
  bool pr_show = true; // Flag for printing of stack remain  
  while (true) {  // Endless task loop

    cycleDrive(); // Execute main drive logic

    if(pr_show) { // Check of stack remain
      Serial.println("uxTaskGetStackHighWaterMark(driveTaskHandle): " + String(uxTaskGetStackHighWaterMark(driveTaskHandle))); // Print stack high water mark
       pr_show = false; // Disable flag after first print
    }
    
    if(uxTaskGetStackHighWaterMark(driveTaskHandle) < 500) { // Check if less than 100 words of stack remain
      Serial.println("Warning: driveTask stack is running low!"); // Print warning
    } 
  }
}

//FreeRTOS tasks Web:
void webTask(void *pvParameters) {    
  (void) pvParameters;                  // Ignores input parameters
  
  setupWiFi(); //Connecting to WiFi in STA mode

  Web_starting(); //Starting the web server
  
  bool pr_show = true; // Flag for printing of stack remain  
  while (true) {  //Endless task loop
                         
    handle_Client();  //Processing client requests

    if(pr_show) { // Check of stack remain
      Serial.println("uxTaskGetStackHighWaterMark(webTaskHandle): " + String(uxTaskGetStackHighWaterMark(webTaskHandle))); // Print stack high water mark
       pr_show = false; // Disable flag after first print
    }
    
    if(uxTaskGetStackHighWaterMark(webTaskHandle) < 500) { // Check if less than 100 words of stack remain
      Serial.println("Warning: webTask stack is running low!"); // Print warning
    } 
  }
}


void setup() {
  Serial.begin(SERIAL_BAUD_RATE); // Init Serial monitor
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

  //Create mutex
  xMutex = xSemaphoreCreateMutex(); 

  // Create two queues, 5 elements each
  toWebQueue = xQueueCreate(5, sizeof(byte));
  toDriveQueue  = xQueueCreate(5, sizeof(byte));
  
  //Create FreeRTOS tasks
  //Priority 2 for drive and 1 for web to give more priority to drive task 
  //Create a drive task pinned to core 0 
  xTaskCreatePinnedToCore(driveTask, "driveTask", 4096, NULL, 2, &driveTaskHandle, 0);

  //Create a webserver task pinned to core 1
  xTaskCreatePinnedToCore(webTask, "webTask", 4096, NULL, 1, &webTaskHandle, 1);
}
  


void loop() {
  // Empty because FreeRTOS scheduler runs the tasks 
}


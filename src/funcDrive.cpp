//funcDrive.cpp

#include <Arduino.h>
#include "config.hpp"
#include "funcDrive.hpp"
#include "funcArray.hpp"

//Path array (in main.cpp):
extern Coord pathSet[MAX_PATH_LENGH];
extern Par pathSetPar;
//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;

//Real Coordinates (in main.cpp):
extern realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель
extern int currentAngle; //Текущий угол по Х
//Mutex for synchronizing access to shared matrixes (in main.cpp):
extern SemaphoreHandle_t xMutex;

extern QueueHandle_t toWebQueue, toDriveQueue;   // two FreeRTOS queues: toWeb and toDrive (in main.cpp)

//STAGES:
#define STAGE_WAITE 0
#define STAGE_FINDPATH 1
#define STAGE_GO 2

//Stage
byte stage, displayed_stage;

// змінні для обміну даними між задачами (variables for data exchange between tasks)
byte sendedByte, receivedByte; 

//Init Real Coordinates:
void initRealCoords() {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
      realCoordsCurrent={155, 125}; //Текущие 
      realCoordsGoal={260, 210};   //Цель
      xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    } 
    else {
      Serial.println("Failed to take mutex in initRealCoords!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
    }   
}

//Init Obstacle Set:
int initializationObstacleSet(){
    Serial.println("initializationObstacleSet() called"); // Виводимо повідомлення про виклик функції (Print message about function call)
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        //Clear Obstacle Set:
        ClearCoords(obstacleSet, obstacleSetPar);
        //temporarily place obstacles manually (тимчасово розміщуємо перешкоди вручну)
        Coord obstacle;
        obstacle={0,0};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={1,0};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={2,0};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={2,1};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={2,2};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={2,8};   
        AddCoords(obstacleSet, obstacleSetPar, obstacle);
        obstacle={2,9};
        AddCoords(obstacleSet, obstacleSetPar, obstacle);

        Serial.println(obstacleSet[0].y); // Виводимо координату y першої перешкоди для перевірки (Print y coordinate of the first obstacle for verification)
        Serial.println(obstacleSet[0].x); // Виводимо координату x
        //...........

        xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    } 
    else {
        Serial.println("Failed to take mutex in initializationObstacleSet!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
        return -1; // Повертаємо -1 у випадку помилки (Return -1 on error)
    }  
    return 0; // Повертаємо 0 при успішному завершенні (Return 0 on success) 
}

//Init Stage:
void initStage() {
    stage = STAGE_WAITE; // Початковий етап - очікування (Initial stage - waiting)
    displayed_stage = 255; // Невідображений етап (Undisplayed stage)
}

// виконуємо основну логіку керування (execute main drive logic)
void cycleDrive(void){
    //display the stage:
    if(displayed_stage != stage) { // Якщо етап змінився (If stage has changed)
        Serial.print("Stage: ");
        if(stage == STAGE_WAITE) {
            Serial.println("WAITE");
        } else if(stage == STAGE_FINDPATH) {
            Serial.println("FINDPATH");
        } else if(stage == STAGE_GO) {
            Serial.println("GO");
        } else {
            Serial.println("UNKNOWN");
        }
        displayed_stage = stage; // Оновлюємо відображений етап (Update displayed stage)
    }  
    
    //Stage 0 - Waiting control stage
    if(stage == STAGE_WAITE){ 
        if(uxQueueMessagesWaiting( toDriveQueue ) > 0) { //Як що є данні від Web (If there are data from Web)
            xQueueReceive(toDriveQueue, &receivedByte, 0);
            // Обробляємо отриманий байт (Process received byte)
            if(receivedByte == ' ') { //If get ' ' command
                return;
            }
            else if(receivedByte == 'G') { //If get "GO" command
                stage = STAGE_GO;
                //time_finish=0; //Нулевое время финиша - это начало отработки пути
                return;
            }
            else if(receivedByte == 'F') { //If get "Find Path" command
                stage = STAGE_FINDPATH;
                return; 
            }
        }

    }

    //Stage STAGE_FINDPATH *******************************************
    if(stage == STAGE_FINDPATH){
        //.........
        vTaskDelay(100 / portTICK_PERIOD_MS); // затримка 100 мс (poll every 100ms)
        stage = STAGE_WAITE;
    }

    //Stage STAGE_GO *******************************************
    if(stage == STAGE_GO){
        //.........
         vTaskDelay(100 / portTICK_PERIOD_MS); // затримка 100 мс (poll every 100ms)
        stage = STAGE_WAITE;
    }
}
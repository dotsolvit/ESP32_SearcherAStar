//funcDrive.cpp

#include <Arduino.h>
#include "config.hpp"
#include "funcDrive.hpp"
#include "funcArray.hpp"
#include "funcMPU6050.hpp"
#include "funcFindPath.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"
#include "funcIRSensors.hpp"
#include "funcMotors.hpp" 

//Path array (in main.cpp):
extern Coord pathSet[MAX_PATH_LENGH];
extern Par pathSetPar;
//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;

//Real Coordinates (in main.cpp):
extern realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель

//Mutex for synchronizing access to shared matrixes (in main.cpp):
extern SemaphoreHandle_t xMutex;

extern QueueHandle_t toWebQueue, toDriveQueue;   // two FreeRTOS queues: toWeb and toDrive (in main.cpp)

extern int currentAngle, displayed_currentAngle; //Текущий угол по Х (in main.cpp)

//Distance covered counters:
extern int distancePulseCounterLeft;
extern int distancePulseCounterRight;

//STAGES:
#define STAGE_WAITE 0
#define STAGE_FINDPATH 1
#define STAGE_GO 2
#define STAGE_RUN 3 //STAGE_FINDPATH + STAGE_GO

//Stage
byte stage, displayed_stage;

// змінні для обміну даними між задачами (variables for data exchange between tasks)
byte sendedByte, receivedByte; 

//Distance covered:
int currentDistanceCovered, finishedDistanceCovered;

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
    
    //Get rotation angle in degrees:
    //currentAngle = getAngleX();
    displayed_currentAngle = currentAngle ;
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
        obstacle={20,16};
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


//Init MPU6050:
void initMPU6050() {
    initializationMPU6050(); // Ініціалізація MPU6050 (MPU6050 initialization)
}

// виконуємо основну логіку керування (execute main drive logic)
void cycleDrive(void){
    //display the stage:
    if(displayed_stage != stage) { // Якщо етап змінився (If stage has changed)
        Serial.print("Stage: ");
        if(stage == STAGE_WAITE) {
            displayMessage(1, "Stage: WAITE", 0, "");
            Serial.println("WAITE");
        } else if(stage == STAGE_FINDPATH) {
            displayMessage(1, "Stage: FINDPATH", 0, "");
            Serial.println("FINDPATH");
        } else if(stage == STAGE_GO) {
            displayMessage(1, "Stage: GO", 0, "");
            Serial.println("GO");
        } else if(stage == STAGE_RUN) {
            displayMessage(1, "Stage: RUN", 0, "");
            Serial.println("RUN");
        } else {
            displayMessage(1, "Stage: UNKNOWN", 0, "");
            Serial.println("UNKNOWN");
        }
        displayed_stage = stage; // Оновлюємо відображений етап (Update displayed stage)
    }  
    
    //Stage 0 - Waiting control stage
    if(stage == STAGE_WAITE){ 
        //Show angl: 
        currentAngle=getAngleX();
        displayAngle( currentAngle );
        // 
        displayBattery();  //Display battery
        displayDistance(); //Display distance to obstacles
        
        // WRM show distance to obstacles
        int distance = IR_Distance();
        displayMessage(2, "Dist= ", distance, "cm");
        // WRM show distance covered
        int distanceCovered = odometer();
        displayMessage(3, "Dist Covered= ", distanceCovered, "cm");
        //WRM

        if(uxQueueMessagesWaiting( toDriveQueue ) > 0) { //Як що є данні від Web (If there are data from Web)
            xQueueReceive(toDriveQueue, &receivedByte, 0);
            String receivedByteStr = String((char)receivedByte); // Convert byte to String for display 
            Serial.println("Received byte from Web: " + receivedByteStr); // Виводимо отриманий байт для перевірки (Print received byte for verification)   
            displayMessage(3, receivedByteStr.c_str(), 0, "");
            // Обробляємо отриманий байт (Process received byte)
            if(receivedByte == ' ') { //If get ' ' command
                return;
            }
            else if(receivedByte == 'G') { //If get "GO" command
                stage = STAGE_GO;
                //Distance covered:
                finishedDistanceCovered = 0; //Finish distance covered - it's zero at the start of path execution
                return;
            }
            else if(receivedByte == 'F') { //If get "Find Path" command
                stage = STAGE_FINDPATH;
                return; 
            }
        }

    }

    //Stage STAGE_FINDPATH *******************************************
    if(stage == STAGE_FINDPATH  or stage == STAGE_RUN){
        Serial.println("Start find path:");
        //Cleat path set 
        ClearCoords(pathSet, pathSetPar);
        Coord start_coord = TransformRealToGridCoords(realCoordsCurrent);
        Coord goal_coord = TransformRealToGridCoords(realCoordsGoal);
        ReturnCode code = findPath (start_coord , goal_coord );
        if( code.return_code != 0) {
            Serial.print(code.return_code);
            Serial.println("Pathfinding is unsuccessful!");
        }
        else{
            Serial.println("The path has been found");

            //Compress Path:
            Serial.println("Compress Path");
            compressPath();
            Serial.println("The path has been compressed");
        }

        if(stage == STAGE_RUN) {
            stage = STAGE_GO;
            finishedDistanceCovered = 0; //Finish distance covered - it's zero at the start of path execution
        }
        else stage = STAGE_WAITE; //Stage Waiting control stage
    }

    //Stage STAGE_GO *******************************************
    if(stage == STAGE_GO){
        //If we've even reached this stage
        if(finishedDistanceCovered == 0){
            //Distance covered counters:
            distancePulseCounterLeft = 0;
            distancePulseCounterRight = 0;  
            
            TankBuz(SIGNAL_GO);
            Serial.println("Start moving to the goal");
        }
        currentDistanceCovered = odometer() + 1;
        if(currentDistanceCovered < finishedDistanceCovered){ //Еще не доехали до след точки пути
            //Сканируем наличие препятствий***************************************************
            //,,,
            //
        }
        else{
            if(finishedDistanceCovered !=0){ //Если это было не начало пути
                //Останавливаемся
                TankStop();
                vTaskDelay(250 / portTICK_PERIOD_MS); // затримка 250 мс до полної зупинки (250 ms delay to complete stop)
                Serial.println("Tank stopped");
                currentDistanceCovered = odometer();
                currentAngle = getAngleX();
                Serial.println("Current distance covered: " + String(currentDistanceCovered) + " cm"); 
                // and other...
                
                stage = STAGE_WAITE;
                return; //Завершение отработки пути (Finish path execution)
            }
            //WRM 
            finishedDistanceCovered = 150; //For testing, let's say we need to cover 150 cm to reach the goal )
            TankForward(200); //Tank Forward
            Serial.println("Tank is moving forward");
        }
    }
}
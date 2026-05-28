//funcDrive.cpp

#include <Arduino.h>
#include "config.hpp"
#include "funcDrive.hpp"
#include "funcArray.hpp"
#include "funcMPU6050.hpp"
#include "funcFindPath.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"
#include "funcSensors.hpp"
#include "funcMotors.hpp" 
#include "funcPilot.hpp"

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

//pathIndexForGo
int pathIndexForGo;

//Current tank speed
int currentTankSpeed;

//STAGES:
#define STAGE_WAITE 0
#define STAGE_FINDPATH 1
#define STAGE_GO 2
#define STAGE_RUN 3 //STAGE_FINDPATH + STAGE_GO
#define STAGE_TEST 4

//Stage
byte stage, displayed_stage;


//Angle
extern int currentAngle, displayed_currentAngle; //Текущий угол по Х

//Scanner angle:    
int scannerAngle;

//Movement stage:
#define MOVEMENT_WAIT 0
#define MOVEMENT_INIT 1
#define MOVEMENT_TURN 2
#define MOVEMENT_FORWARD 3
#define MOVEMENT_STOP 4
#define MOVEMENT_STOP_SCANNER 5

byte movementStage;


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
    movementStage = MOVEMENT_WAIT; // Початковий етап руху - очікування (Initial movement stage - waiting)
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
        } else if(stage == STAGE_TEST) {
            displayMessage(1, "Stage: TEST", 0, "");
            Serial.println("TEST");    
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
        displayEchoDistance(); //Display distance to obstacles
        
        // WRM show distance to obstacles
        int distance = distanceEcho();
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
                movementStage = MOVEMENT_WAIT; // Початковий етап руху - очікування (Initial movement stage - waiting)
                //Distance covered:
                finishedDistanceCovered = 0; //Finish distance covered - it's zero at the start of path execution
                return;
            }
            else if(receivedByte == 'F') { //If get "Find Path" command
                stage = STAGE_FINDPATH;
                return; 
            }
            else if(receivedByte == 'R') { //If get "Run" command
                stage = STAGE_RUN;
                return; 
            }
             else if(receivedByte == 'T') { //If get "Test" command
                stage = STAGE_TEST;
                return; 
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // затримка 100 мс для зменшення навантаження (poll every 100ms)
    }

    //Stage STAGE_FINDPATH *******************************************
    if(stage == STAGE_FINDPATH ){
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
            //stage = STAGE_GO;
            //finishedDistanceCovered = 0; //Finish distance covered - it's zero at the start of path execution
        }
        else stage = STAGE_WAITE; //Stage Waiting control stage
    }

    //Stage STAGE_GO *******************************************
    if(stage == STAGE_GO){
        if(movementStage == MOVEMENT_WAIT){
            displayMessage(2, "MOVEMENT_WAIT", 0, "");
            if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
                Serial.println("Failed to take mutex in initRealCoords!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
                stage = STAGE_WAITE; //Stage Waiting control stage
                return;
            }
            //Distance covered counters:
            distancePulseCounterLeft = 0;
            distancePulseCounterRight = 0;  
            currentTankSpeed = 0; //Current tank speed is zero at the start of movement       
            movementStage = MOVEMENT_INIT;
        }
        if(movementStage == MOVEMENT_INIT){
            displayMessage(2, "MOVEMENT_INIT", 0, "");
            if(pilotInit() != 0) { //If pilot initialization is unsuccessful
                Serial.println("Pilot initialization is unsuccessful!");
                stage = STAGE_WAITE; //Stage Waiting control stage
                movementStage = MOVEMENT_WAIT; //
                xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
                return;
            }
            TankBuz(SIGNAL_GO);
            Serial.println("Start moving to the goal");
            movementStage = MOVEMENT_TURN;
        }
        if(movementStage == MOVEMENT_TURN){
            displayMessage(2, "MOVEMENT_TURN", 0, "");
            pilotTurn(); //Turn to the current point of the path
            currentTankSpeed = 0; //Current tank speed is zero at the start of movement
            movementStage = MOVEMENT_FORWARD;
            displayMessage(2, "MOVEMENT_FORWARD", 0, "");
        }
        if(movementStage == MOVEMENT_FORWARD){
            if(pilotScanner != 0) { //If the scanner detects a new obstacle
                Serial.println("The scanner detects a new obstacle!");
                pilotStop();
                TankBuz(SIGNAL_OBSTACLE);
                movementStage = MOVEMENT_STOP_SCANNER; //
            }
            else { 
                //Go to the next point of the path
                if(pilotForward() != 0) {
                    Serial.println("Stop!");
                    pilotStop();
                    //If we have reached the final point of the journey
                    if(pathIndexForGo == 0){
                    TankBuz(SIGNAL_GO);
                    xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
                    stage = STAGE_WAITE;
                    Serial.println("It is Goal Point!");

                    //Для контроля высылаем на сайт координаты - текущие и целевые
                    //sendToWebsiteRealCoords();
                    return;
                    }
                    movementStage = MOVEMENT_TURN; //
                }
            }
        }
        if(movementStage == MOVEMENT_STOP_SCANNER){
            displayMessage(2, "STOP_SCANNER", 0, "");
            pilotStopScanner();
            xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
            stage = STAGE_WAITE; //Stage Waiting control stage
            //Для контроля высылаем на сайт координаты - текущие и целевые
            //sendToWebsiteRealCoords();
            return;
        }
        
    }

    //WRM Stage STAGE_RUN *******************************************
    if(stage == STAGE_RUN) {
        for(int i=-SERVO_MAX_ANGLE; i<= SERVO_MAX_ANGLE; i+=45){ //Test servo
            setServo(i);
            delay(800);     
        }

        stage = STAGE_WAITE;
        return;
    }

    //WRM Stage STAGE_TEST *******************************************
    if(stage == STAGE_TEST) {
        initJournal(); //Init journal

        for(int i=0; i<400; i++) { //Test journal
            //The scanner for detect a new obstacle
            int ret = pilotScanner() ;


            vTaskDelay(10 / portTICK_PERIOD_MS); // затримка 10 мс (poll every 100ms)
        }
        
        scannerAngle = 0; //Set scanner angle to 0  
        setServo(scannerAngle);
        //Here should be the code for testing the robot
        stage = STAGE_WAITE;
        return;
    }   

}
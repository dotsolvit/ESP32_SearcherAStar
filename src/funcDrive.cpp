//funcDrive.cpp

#include <Arduino.h>
#include <EEPROM.h>
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
//Route array (in main.cpp):
extern Coord routeSet[MAX_ROUTE_LENGH];
extern Par routeSetPar;

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

//STAGES:
#define STAGE_WAITE 0
#define STAGE_FINDPATH 1
#define STAGE_GO 2
#define STAGE_RUN 3 //STAGE_FINDPATH + STAGE_GO
#define STAGE_TEST 4
#define STAGE_LOADOBSTACLES 5
#define STAGE_SAVEOBSTACLES 6
#define STAGE_CLEAROBSTACLES 7

//Stage
byte stage, displayed_stage;


//Angle
extern int currentAngle, displayed_currentAngle; //Текущий угол по Х

//Scanner angle:    
int scannerAngle;

//Movement (GO and RUN)stage:
#define MOVEMENT_WAIT 0
#define MOVEMENT_INIT 1
#define MOVEMENT_TURN 2
#define MOVEMENT_FORWARD 3
#define MOVEMENT_STOP 4
#define MOVEMENT_STOP_SCANNER 5

//RUN stage
#define RUN_FINDPATH 0
#define RUN_GO 1

byte movementStage;
byte runStage;


// змінні для обміну даними між задачами (variables for data exchange between tasks)
byte sendedByte, receivedByte; 

//Distance covered:
int currentDistanceCovered; 

//Init EEPROM:
void initEEPROM() {
    EEPROM.begin(sizeof(obstacleSet)+1); // Initialize EEPROM with size equal to the size of obstacleSet plus 1 byte
    Serial.println("EEPROM initialized with size: " + String(sizeof(obstacleSet)+1)); // Print the initialized size for verification
}

//Init Real Coordinates:
void initRealCoords() {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
      realCoordsCurrent={155, 125}; //Текущие   175, 125
      realCoordsGoal={155, 295};   //Цель 155, 295
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
        /*
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
        */
       
        xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    } 
    else {
        Serial.println("Failed to take mutex in initializationObstacleSet!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
        return -1; // Повертаємо -1 у випадку помилки (Return -1 on error)
    }  
    return 0; // Повертаємо 0 при успішному завершенні (Return 0 on success) 
}

//Initialization Route Set:
int initializationRouteSet(){
    Serial.println("initializationRouteSet() called"); // Виводимо повідомлення про виклик функції (Print message about function call)
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        //Clear Route Set:
        ClearCoords(routeSet, routeSetPar);
        xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    } 
    else {
        Serial.println("Failed to take mutex in initializationRouteSet!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
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
        } else if(stage == STAGE_LOADOBSTACLES) { 
            displayMessage(1, "Stage: LOADOBSTACLES", 0, "");
            Serial.println("LOADOBSTACLES");
        } else if(stage == STAGE_SAVEOBSTACLES) {
            displayMessage(1, "Stage: SAVEOBSTACLES", 0, "");
            Serial.println("SAVEOBSTACLES");
        } else if(stage == STAGE_CLEAROBSTACLES) {
            displayMessage(1, "Stage: CLEAROBSTACLES", 0, "");
            Serial.println("CLEAROBSTACLES");
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
                initJournal(); //Init journal for GO stage
                stage = STAGE_GO;
                movementStage = MOVEMENT_WAIT; // Початковий етап руху - очікування (Initial movement stage - waiting)
                return;
            }
            else if(receivedByte == 'F') { //If get "Find Path" command
                stage = STAGE_FINDPATH;
                return; 
            }
            else if(receivedByte == 'R') { //If get "Run" command
                initJournal(); //Init journal for RUN stage
                stage = STAGE_RUN;
                runStage = RUN_FINDPATH; // Initial RUN stage - find path
                return; 
            }
             else if(receivedByte == 'T') { //If get "Test" command
                stage = STAGE_TEST;
                return; 
            }
            else if(receivedByte == 'L') { //If get "Load Obstacles" command
                stage = STAGE_LOADOBSTACLES;
                return; 
            }
            else if(receivedByte == 'S') { //If get "Save Obstacles" command
                stage = STAGE_SAVEOBSTACLES;
                return; 
            }
            else if(receivedByte == 'C') { //If get "Clear Obstacles" command
                stage = STAGE_CLEAROBSTACLES;
                return; 
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms delay to reduce load
    }

    //Stage STAGE_FINDPATH *******************************************
    if(stage == STAGE_FINDPATH or (stage == STAGE_RUN and runStage == RUN_FINDPATH)) { //If it's Find Path stage or Run stage (if it's Run stage, we will find the path in the beginning of it)
        String message;
        Serial.println("Start find path:");
        //Cleat path set 
        ClearCoords(pathSet, pathSetPar);
        Coord start_coord = TransformRealToGridCoords(realCoordsCurrent);
        Coord goal_coord = TransformRealToGridCoords(realCoordsGoal);
        ReturnCode code = findPath (start_coord , goal_coord );
        if( code.return_code != 0) {
            Serial.print(code.return_code);
            Serial.println("Pathfinding is unsuccessful!");
            displayMessage(3, "No path found!", 0, "");
            addToJournal("No path found!" );
            stage = STAGE_WAITE; //Stage Waiting control stage
            TankBuz(SIGNAL_NOPATH);
            return;
        }
        else{
            Serial.println("The path has been found");
            //Compress Path:
            Serial.println("Compress Path");
            compressPath();
            Serial.println("The path has been compressed");
            message = "Path found and compressed! Stage: " ;
            if(stage == STAGE_RUN) message += "STAGE_RUN";
            else message += "STAGE_FINDPATH";
            addToJournal(message.c_str());
        }

        if(stage == STAGE_RUN) {
            runStage = RUN_GO; // Move to RUN stage - go!
            movementStage = MOVEMENT_WAIT; // Початковий етап руху - очікування (Initial movement stage - waiting)
            addToJournal("Stage RUN, RUN_GO, MOVEMENT_WAIT" );
        }
        else stage = STAGE_WAITE; //Stage Waiting control stage
    }

    //Stage STAGE_GO *******************************************
    if(stage == STAGE_GO or (stage == STAGE_RUN and runStage == RUN_GO)) { //If it's GO stage or Run stage 
        if(movementStage == MOVEMENT_WAIT){
            displayMessage(2, "MOVEMENT_WAIT", 0, "");
            if (xSemaphoreTake(xMutex, portMAX_DELAY) != pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
                Serial.println("Failed to take mutex in initRealCoords!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
                displayMessage(3, "Failed mutex!", 0, "");
                addToJournal("Failed to take mutex in GO stage!" );
                stage = STAGE_WAITE; //Stage Waiting control stage
                movementStage = MOVEMENT_WAIT;
                return;
            }
            movementStage = MOVEMENT_INIT;
        }
        if(movementStage == MOVEMENT_INIT){
            displayMessage(2, "MOVEMENT_INIT", 0, "");
            if(pilotInit() != 0) { //If pilot initialization is unsuccessful
                Serial.println("Pilot initialization is unsuccessful!");
                displayMessage(3, "Failed pilot init!", 0, "");
                addToJournal("Failed pilot init!" );
                stage = STAGE_WAITE; //Stage Waiting control stage
                movementStage = MOVEMENT_WAIT; //
                xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
                return;
            }
            TankBuz(SIGNAL_GO);
            Serial.println("Start moving to the goal");
            //Set the first point of the route
            addPointToRoute(realCoordsCurrent);
            //
            movementStage = MOVEMENT_TURN;
        }
        if(movementStage == MOVEMENT_TURN){
            displayMessage(2, "MOVEMENT_TURN", 0, "");
            //Fix the turning point of the route
            addPointToRoute(realCoordsCurrent);
            resetPreviousDistanceCoveredRoute(); //Reset previous distance covered for the route
            //
            pilotTurn(); //Turn to the current point of the path
            movementStage = MOVEMENT_FORWARD;
            //Distance covered counters:
            distancePulseCounterLeft = 0;
            distancePulseCounterRight = 0;  
            displayMessage(2, "MOVEMENT_FORWARD", 0, "");
        }
        if(movementStage == MOVEMENT_FORWARD){          
            if(pilotNarrowScanner() == 1) { //If the scanner detects a new obstacle
                Serial.println("The scanner detects a new obstacle!");
                pilotStopScanner();
                TankBuz(SIGNAL_OBSTACLE);
                currentAngle = getAngleX();
                currentDistanceCovered = odometer();
                //New current coordinates Calc Real current Coordinates:
                realCoordsCurrent=calcRealCoords(realCoordsCurrent, currentAngle, currentDistanceCovered);
                //Fix the turning point of the route
                addPointToRoute(realCoordsCurrent);
                //
                movementStage = MOVEMENT_STOP_SCANNER; //
                initCircularScanner(); //Init circular scanner
            }
            else { 
                //Add a point to the route in movement
                addPointToRouteInMovement(realCoordsCurrent, currentAngle, odometer());
                pilotForward(); //Go forward
                if(pilotStop() == 1) { //We reached the next point on the route
                    currentAngle = getAngleX();
                    //New current coordinates Calc Real current Coordinates:
                    realCoordsCurrent=calcRealCoords(realCoordsCurrent, currentAngle, currentDistanceCovered);
                    //Decreasing the path index
                    pathIndexForGo --;
                    //Fix the turning point of the route
                    addPointToRoute(realCoordsCurrent);
                    //
                    
                    //If we have reached the final point of the journey
                    if(pathIndexForGo == 0){
                        TankBuz(SIGNAL_GO);
                        xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
                        stage = STAGE_WAITE;
                        movementStage = MOVEMENT_WAIT; //
                        Serial.println("It is Goal Point!");
                        return;
                    }
                    movementStage = MOVEMENT_TURN; //
                }
            }
        }
        if(movementStage == MOVEMENT_STOP_SCANNER){
            displayMessage(2, "STOP_SCANNER", 0, "");
            //The circular scanner for detect new obstacles
            if(pilotScannerCircular() == 1) {
                addToJournal("End of circular scanning" );
                xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
                if(stage == STAGE_GO) {
                    stage = STAGE_WAITE; //Stage Waiting control stage
                    addToJournal("Stage WAIT" );
                }
                else if(stage == STAGE_RUN) {
                    runStage = RUN_FINDPATH; // Move to RUN stage - find path!
                    addToJournal("Stage RUN, RUN_FINDPATH, MOVEMENT_WAIT" );
                }
                movementStage = MOVEMENT_WAIT; //
            }
        }
    }

    //WRM Stage STAGE_TEST *******************************************
    if(stage == STAGE_TEST) {
        String message;
        initJournal(); //Init journal
        scannerAngle = 0; //Set scanner angle to 0  
        setServo(scannerAngle);
        //Distance covered counters:
        distancePulseCounterLeft = 0;
        distancePulseCounterRight = 0; 

        //Tast pilotCircularScanner:
        initCircularScanner(); //Init circular scanner
        for(int i=0; i<1100; i++) {
            if(pilotScannerCircular() == 1) { //If the scanner detects a new obstacle
                break;
            }
            vTaskDelay(5 / portTICK_PERIOD_MS); // delay 5 ms
        }

        /*
        //Test turn -45 fyd +45 degrees:
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        TankRorateOnAngle(-45);
        currentAngle=getAngleX();
        displayAngle( currentAngle );

        vTaskDelay(5000 / portTICK_PERIOD_MS);
        TankRorateOnAngle(45);
        currentAngle=getAngleX();
        displayAngle( currentAngle );
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        */
 
        /*
        //Pilot initialization
        if(pilotInit() == 0) {

            if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)

                for(int i=0; i<500; i++) { //Test journal
                    //The scanner for detect a new obstacle
                    if(pilotNarrowScanner() == 1) { //If the scanner detects a new obstacle
                        TankStop();
                        Serial.println("The scanner detects a new obstacle!");
                        addToJournal("Stop.Scanner detects a new obstacle!"); // Add message to journal
                        break; //
                    }
                    pilotForward(); //Go forward
                    if(pilotStop() == 1) { //If the robot has stopped successfully
                        break;
                    }
                    vTaskDelay(5 / portTICK_PERIOD_MS); // затримка 5 мс (poll every 100ms)
                }

                xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
            }
            else {
                Serial.println("Failed to take mutex in initRealCoords!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
            }
        }
        */

        scannerAngle = 0; //Set scanner angle to 0  
        setServo(scannerAngle);
        //Here should be the code for testing the robot
        stage = STAGE_WAITE;
        return;
    }  
    
    //EEPROM
    //Stage STAGE_LOADOBSTACLES
    if(stage == STAGE_LOADOBSTACLES){
        Serial.println("Load Obstacles from EEPROM");
        //Загрузить матрицу если она есть:
        if(EEPROM.read(0) != 0){
            obstacleSetPar.setSize = int(EEPROM.read(0));
            EEPROM.get(1, obstacleSet);
            Serial.println("Obstacle loaded");
        }
        else{
            Serial.println("EEPROM is empty!");
        }
        stage = STAGE_WAITE; //Stage Waiting control stage
    }
    //Stage STAGE_SAVEOBSTACLES
    if(stage == STAGE_SAVEOBSTACLES){
        Serial.println("SAVE Obstacles to EEPROM");
        uint8_t size=obstacleSetPar.setSize;
        EEPROM.put(0, size);
        EEPROM.put(1, obstacleSet);
        EEPROM.commit();
        Serial.println("Obstacle saved to EEPROM");
        stage = STAGE_WAITE; //Stage Waiting control stage
    }
    //Stage STAGE_CLEAROBSTACLES
    if(stage == STAGE_CLEAROBSTACLES){
        //Очистить одномерную матрицу координат
        ClearCoords(obstacleSet, obstacleSetPar);
        Serial.println("Set Obstacles cleared");
        stage = STAGE_WAITE; //Stage Waiting control stage
    }
    //

}
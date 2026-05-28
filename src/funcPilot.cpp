//Function Pilot

#include <Arduino.h>
#include "config.hpp" 
#include "funcPilot.hpp"
#include "funcMotors.hpp" 
#include "funcMPU6050.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"
#include "funcSensors.hpp"

//Path array (in main.cpp):
extern Coord pathSet[MAX_PATH_LENGH];
extern Par pathSetPar;
//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;
//Real Coordinates
extern realCoord realCoordsCurrent, realCoordsGoal; //Текущие и цель

//pathIndexForGo
extern int pathIndexForGo;

extern int currentAngle, displayed_currentAngle; //Текущий угол по Х

//Scanner angle:    
extern int scannerAngle;

//Jurnal:
extern char journal[JOURNAL_SIZE][JOURNAL_MESSAGE_LENGTH]; //Журнал сообщений (Journal of messages)
extern int journalIndex; //Индекс для добавления сообщений в журнал (Index for adding messages to the journal)

//WRM
unsigned long firstScanTime = 0;

//Pilot initialization
int pilotInit() {
    if(pathSetPar.setSize==0 or pathSetPar.setSize==1){
        Serial.println("The Path is empty. No GO!");
        TankBuz(SIGNAL_NOPATH);
        return -1; // Return an error code
    }
    pathIndexForGo=pathSetPar.setSize-1; //Index for path array, from which the robot will start moving
    return 0; // Return success code
}

//Turn to the current point of the path
void pilotTurn() {
    int new_angle=calcAngleToNewPointPath(realCoordsCurrent, pathSet[pathIndexForGo-1]); //Calculating the angle to a new waypoint
    currentAngle = getAngleX();
    if(currentAngle != new_angle){
        Serial.print("Rotate from ");Serial.print(currentAngle); Serial.print(" to "); Serial.println(new_angle);
        TankRorateOnAngle(new_angle);
        currentAngle = getAngleX();
    }
    else Serial.println("The angle is the same");
}

//Pilot forward
int pilotForward(void) {
    //Here should be the code for moving the robot forward to the next waypoint in the path
    //For now, we will just return 0, which means that the movement is successful
    return 0; 
}

//Pilot stop
void pilotStop(void) {
  //....
  TankStop();
}


//The scanner for detect a new obstacle
int pilotScanner(void) { 
    static unsigned long lastScanTime = 0;
    //static int measurement_number = 0; //Counter for measurements
    //static int previousDistance = 0; //Previous distance measurement
    int averagedDistance = 0; //Averaged distance measurement

    if (millis() - lastScanTime >= SCANNING_PERIOD) {
        lastScanTime = millis();
        int distance = distanceEcho();
        if(scannerAngle==0){
            if(firstScanTime == 0) firstScanTime  = millis();
            vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
            int distance1 = distanceEcho();
            vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
            int distance2 = distanceEcho();
            int distanceM= medianFilter(distance, distance1, distance2); // Apply median filter to the three measurements
            String message = String(lastScanTime-firstScanTime)+" 0=" + String(distance) +" 1=" + String(distance1)+ " 2=" + String(distance2) + " M=" + String(distanceM)+ "S="+String(scannerAngle);
            addToJournal(message.c_str()); // Add message to journal

            scannerAngle =-SCANNING_ANGLE_STEP;
            setServo(scannerAngle);
        }
        else {
            vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
            int distance1 = distanceEcho();
            vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
            int distance2 = distanceEcho();
            int distanceM= medianFilter(distance, distance1, distance2); // Apply median filter to the three measurements   
            String message = String(lastScanTime-firstScanTime)+" 0=" + String(distance) +" 1=" + String(distance1)+ " 2=" + String(distance2) + " M=" + String(distanceM)+ "S="+String(scannerAngle);
            addToJournal(message.c_str()); // Add message to journal

            if(scannerAngle == 0) scannerAngle = -SCANNING_ANGLE_STEP;
            else scannerAngle = 0;
            setServo(scannerAngle);
        }
    }
    return 0; 
}


//Wide scan after stopping due to an obstacle
void pilotStopScanner(void) {
  //....
  
}

//Journal:
void initJournal() {
    journalIndex = 0; // Сбрасываем индекс (Reset index)
    journal[journalIndex][0] = '\0'; // Инициализируем первую строку пустой строкой (Initialize the first entry with an empty string)   
}
//Function to add a message to the journal
void addToJournal(const char* message) {
    if (journalIndex < JOURNAL_SIZE) {
        strncpy(journal[journalIndex], message, JOURNAL_MESSAGE_LENGTH - 1); // Копируем сообщение в журнал (Copy message to journal)
        journal[journalIndex][JOURNAL_MESSAGE_LENGTH - 1] = '\0'; // Гарантируем нуль-терминирование (Ensure null-termination)
        journalIndex++; // Увеличиваем индекс для следующей записи (Increment index for next entry)
    } else {
        Serial.println("Journal is full! Cannot add more entries."); // Журнал полон (Journal is full)
    }
}

//median filter: from three numbers, the program selects the one that is in the middle in value
int medianFilter(int a, int b, int c) {
    if (a <= b && a <= c) {
        return (b <= c) ? b : c;
    } else if (b <= a && b <= c) {
        return (a <= c) ? a : c;
    } else {
        return (a <= b) ? a : b;
    }
}

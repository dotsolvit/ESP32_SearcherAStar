//Function Pilot

#include <Arduino.h>
#include "config.hpp" 
#include "funcPilot.hpp"
#include "funcMotors.hpp" 
#include "funcMPU6050.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"
#include "funcIRSensors.hpp"

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
    static int measurement_number = 0; //Counter for measurements
    static int previousDistance = 0; //Previous distance measurement
    int averagedDistance = 0; //Averaged distance measurement

    if (millis() - lastScanTime >= SCANNING_PERIOD) {
        lastScanTime = millis();
        int distance = IR_Distance();
        if(scannerAngle==0){
            previousDistance = 0; 
            measurement_number=0;
            scannerAngle =SCANNING_ANGLE_STEP;
            setServo(scannerAngle);
        }
        else if(measurement_number==0){
            previousDistance = distance; //Set the first distance measurement as previous distance
            measurement_number++;
        }
        else {
            if(distance == 0 or previousDistance==0) {
                averagedDistance = 0; 
            }
            else 
            averagedDistance = (previousDistance + distance) / 2; //Averaging the current and previous distance measurements
            String message = "D1=" + String(previousDistance) +"D2=" + String(distance)+ "Do=" + String(averagedDistance) +",An=" + String(currentAngle)+"ScAn="+String(scannerAngle);
            addToJournal(message.c_str()); // Add message to journal
            //
            previousDistance = 0;
            measurement_number=0; //Reset the measurement counter
            //
            if(scannerAngle > 0) scannerAngle = -SCANNING_ANGLE_STEP;
            else scannerAngle = SCANNING_ANGLE_STEP;
            //Turn the servo to the new scanning angle
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

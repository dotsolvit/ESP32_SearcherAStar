//Function Pilot

#include <Arduino.h>
#include "config.hpp" 
#include "funcPilot.hpp"
#include "funcMotors.hpp" 
#include "funcMPU6050.hpp"
#include "funcTransfCoordsAngles.hpp"
#include "funcOLED.hpp"
#include "funcSensors.hpp"
#include "funcTransfCoordsAngles.hpp"

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

//Distance covered:
extern int currentDistanceCovered, finishedDistanceCovered;


//Scanner angle:    
extern int scannerAngle;

bool circularScannerActive = false; // Sign of a circular scanner in operation

//Jurnal:
extern char journal[JOURNAL_SIZE][JOURNAL_MESSAGE_LENGTH]; //Журнал сообщений (Journal of messages)
extern int journalIndex; //Индекс для добавления сообщений в журнал (Index for adding messages to the journal)
extern unsigned long journalInitTime;

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


//The narrow scanner for detect a new obstacle
int pilotNarrowScanner(void) { 
    static unsigned long lastScanTime = 0;
    static int measurement_number = 0; // Counter for measurements
    static int distance0, distance1, distance2; //Distance measurements
    if (scannerAngle == 0) {
        lastScanTime = millis();
        scannerAngle = -NARROW_SCANNING_ANGLE_STEP; // Start from the leftmost position
        measurement_number = 0;
        distance0 = distance1 = distance2 = 0; // Reset distance measurements
        setServo(scannerAngle);
        Serial.println("Start scanning -NARROW_SCANNING_ANGLE_STEP");
    }
    if (millis() - lastScanTime >= (SCANNING_PERIOD + SCANNING_ADDITIONAL_PERIOD * measurement_number) ) {
        if(measurement_number == 0) {
            distance0 = IR_Distance();
            distance1 = distance2 = 0; // Reset distance measurements
            if(distance0 == 0 or distance0 > NARROW_SCANNING_DISTANCE) {
                lastScanTime = millis();
                measurement_number = 0;
            }
            else {
                measurement_number = 1; // Move to the next measurement
            }
        }
        else if(measurement_number == 1) {
            distance1 = IR_Distance(); 
            distance2 = 0; // Reset distance measurements
            measurement_number = 2; // Move to the next measurement
        }
        else if(measurement_number == 2) {
            distance2 = IR_Distance();
            lastScanTime = millis();
            measurement_number = 0; // Reset measurement number for the next scan
            int distanceM= medianFilter(distance0, distance1, distance2); // Apply median filter to the three measurements
            if(distanceM > 0 and distanceM <= NARROW_SCANNING_DISTANCE) {
                currentDistanceCovered=odometer();
                if(seekObstacle(realCoordsCurrent, currentAngle, currentDistanceCovered, SCANNER_OFFSET, scannerAngle, distanceM) != 0 ) {
                    Serial.println("New obstacle detected by narrow scanner!");
                    String message = " D1=" + String(distance0) + " D2=" + String(distance1) + " D3=" + String(distance2) + " M=" + String(distanceM) + "S="+String(scannerAngle);
                    addToJournal(message.c_str()); // Add message to journ
                    scannerAngle = 0; 
                    setServo(scannerAngle);
                    return 1; // New obstacle detected
                }
            }
        }
        if(measurement_number == 0) {
            if(scannerAngle == NARROW_SCANNING_ANGLE_STEP) {
                scannerAngle = -NARROW_SCANNING_ANGLE_STEP; // Move to the next position  
            } else {
                scannerAngle = NARROW_SCANNING_ANGLE_STEP; // Move to the next position 
            }
            setServo(scannerAngle);
        }
    }
    return 0; 
}

//The circular scanner initiation 
void initCircularScanner() {
    circularScannerActive = true;
}

//The circular scanner for detect a new obstacle
int pilotScannerCircular(void) {
    static unsigned long lastScanTime = 0;
    int distance0, distance1, distance2; //Distance measurements

    if(circularScannerActive) {
        lastScanTime = millis();
        circularScannerActive = false; // Deactivate circular scanning 
        scannerAngle = -SERVO_MAX_ANGLE; // Start from the leftmost position
        setServo(scannerAngle);
        Serial.println("Start scanning -SERVO_MAX_ANGLE");
    }

    if (millis() - lastScanTime >= SCANNING_PERIOD) {
        lastScanTime = millis();
        distance0 = IR_Distance();
        vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
        distance1 = IR_Distance(); 
        vTaskDelay(25 / portTICK_PERIOD_MS); // затримка 25 мс
        distance2 = IR_Distance();
        int distanceM= medianFilter(distance0, distance1, distance2); // Apply median filter to the three measurements
        String message = " 0=" + String(distance0) +" 1=" + String(distance1)+ " 2=" + String(distance2) + " M=" + String(distanceM)+ "S="+String(scannerAngle);
        addToJournal(message.c_str()); // Add message to journal
        scannerAngle += SCANNING_ANGLE_STEP; // Move to the next position
        if(scannerAngle <= SERVO_MAX_ANGLE) {
            setServo(scannerAngle);
            if(distanceM > 0) seekAndSetObstacle(realCoordsCurrent, currentAngle, SCANNER_OFFSET, scannerAngle, distanceM); // Process the measurement to seek and set an obstacle
        } else {
            scannerAngle = 0;
            setServo(scannerAngle);
            return 1; //End of scanning
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
    journalInitTime = millis(); // Record the time of journal initialization
}
//Function to add a message to the journal
void addToJournal(const char* message) {
    if (journalIndex < JOURNAL_SIZE) {
        String mes = String(millis()-journalInitTime)+" ";
        mes += message;
        strncpy(journal[journalIndex], mes.c_str(), JOURNAL_MESSAGE_LENGTH - 1); // Копируем сообщение в журнал (Copy message to journal)
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

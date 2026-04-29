//funcWiFiWeb.cpp
//Модуль фунукццій для роботи з WiFi та веб-сервером на ESP32
//

#include <Arduino.h>

#include "config.hpp"
#include "ssid.h"
#include "funcWiFiWeb.h"

#include <WiFi.h>
#include <WebServer.h>

//Path array (in main.cpp):
extern Coord pathSet[MAX_PATH_LENGH];
extern Par pathSetPar;
//Obstacle array (in main.cpp):
extern Coord obstacleSet[MAX_OBSTACLE_LENGH];
extern Par obstacleSetPar;

//Real Coordinates
extern realCoord realCoordsCurrent, realCoordsGoal; 

//Mutex for synchronizing access to shared matrixes (in main.cpp):
extern SemaphoreHandle_t xMutex;

extern QueueHandle_t toWebQueue, toDriveQueue;   // two FreeRTOS queues: toWeb and toDrive (in main.cpp)

WebServer server(80);  // Вебсервер на порту 80 (Web server on port 80)

//web message
String web_message, web_messageMap;

//Current coords
int current_y, current_x;
//Goal coords
int goal_y, goal_x;

//For Table:
//number of lines displayed
#define MAX_NUBMER_OF_LINES SIZE_MAP_Y
#define NUMBER_OF_ROWS_IN_THE_TABLE 20
#define SHIFTING_ROWS_IN_THE_TABLE 10
//Top linr number for map obstacle and path visualization
int top_line_number=MAX_NUBMER_OF_LINES-1;

//WiFi
//Connecting to WiFi in STA mode
void setupWiFi(void) {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(WIFI_SSID);
  pinMode(LED_BUILTIN, OUTPUT);//Setting the output mode for the built-in LED
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  //Waiting for network connection
  while (WiFi.status() != WL_CONNECTED) {
    //Serial.println("Connecting...");
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(200 / portTICK_PERIOD_MS); // poll every 200ms
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(600 / portTICK_PERIOD_MS); // poll every 600ms
  }

  digitalWrite(LED_BUILTIN, HIGH);//Indication after successful connection to WiFi network
  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

//Web
// Основна логіка керування Web (main Web logic)
//Starting the web server
void Web_starting(){
  //initialization parameters:
  web_message="Test";
  web_messageMap="Test";

  //Loading current and goal coords for the main page:
  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
    //Current coords
    current_y = realCoordsCurrent.y; 
    current_x = realCoordsCurrent.x;
    //Goal coords
    goal_y = realCoordsGoal.y;
    goal_x = realCoordsGoal.x;
    xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
  } 
  else {
    Serial.println("Failed to take mutex in Web_starting!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
  }  

  //Configuring routes for the web server
  server.on("/", handle_OnConnect); //Home page
  server.on("/sendyx", handle_sendyx);  //Actions when pressing send y x
  server.on("/findpath", handle_findpath);  //Actions when pressing find path
  server.on("/sendgo", handle_sendgo); //Actions when pressing send go
  server.on("/seemap", handle_seemap); //Actions when pressing see map
  server.on("/seemapup", handle_seemapup);  //Actions when pressing See Map Up 10
  server.on("/seemapdown", handle_seemapdown);  //Actions when pressing See Map Down 10
  server.begin(); //Starting the web server
}

//Processing client requests
void handle_Client(){
  server.handleClient(); 
}

//Home page handler
void handle_OnConnect() {
  server.send(200, "text/html", SendHTML()); //Sending an HTML page)
}

//Actions when pressing send y x
void handle_sendyx(void) {
  if( server.hasArg("Y") ) {
    String valY = server.arg("Y");
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
      realCoordsGoal.y = valY.toInt(); // Преобразование в число
      Serial.println("Y"+valY); //Send Y coords to Serial
      current_y = realCoordsCurrent.y; 
      current_x = realCoordsCurrent.x;
      goal_y = realCoordsGoal.y;
      goal_x = realCoordsGoal.x;
      xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    } 
    else {
      Serial.println("Failed to take mutex in handle_sendyx!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
    }  
  }
  if( server.hasArg("X") ) {
    String valX = server.arg("X");
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { // Блокування м'ютекса для безпечного доступу до спільних змінних (Lock mutex for safe access to shared variables)
      realCoordsGoal.x = valX.toInt(); // Преобразование в число
      Serial.println("X"+valX); //Send X coords to Serial
      current_y = realCoordsCurrent.y; 
      current_x = realCoordsCurrent.x;
      goal_y = realCoordsGoal.y;
      goal_x = realCoordsGoal.x;
      xSemaphoreGive(xMutex); // Звільнення м'ютекса після завершення роботи (Release mutex after done)
    }
    else {
      Serial.println("Failed to take mutex in handle_sendyx!"); // Виводимо повідомлення про помилку, якщо не вдалося взяти м'ютекс (Print error message if failed to take mutex)
    }
  }
  server.sendHeader("Location", "/"); //Redirect to the main page
  server.send(303);
}

//Actions when pressing finde path
void handle_findpath(void) {
  Serial.println("F"); //Send find path command to Serial
  byte sendedByteW = 'F'; //Set find path command to sendedByte
  xQueueSend(toDriveQueue, &sendedByteW, 0); //Send find path command to toDriveQueue
  server.sendHeader("Location", "/"); //Redirect to the main page
  server.send(303);
}
//Actions when pressing send go
void handle_sendgo() {
  Serial.println("G"); //Send Go command to Serial
  byte sendedByteW = 'G'; //Set Go command to sendedByte
  xQueueSend(toDriveQueue, &sendedByteW, 0); //Send Go command to toDriveQueue
  server.sendHeader("Location", "/"); //Redirect to the main page
  server.send(303);
}
//Actions when pressing See Map
void handle_seemap(void) {
  web_messageMap=mapObstacleAndPath(top_line_number);
  server.send(200, "text/html", SendSeeMapHTML(web_messageMap)); //Sending an HTML page SeeMap
}
void handle_seemapup(void) {
  top_line_number +=SHIFTING_ROWS_IN_THE_TABLE;
  if(top_line_number > (MAX_NUBMER_OF_LINES-1) ) top_line_number=MAX_NUBMER_OF_LINES-1;
  web_messageMap=mapObstacleAndPath(top_line_number);
  server.send(200, "text/html", SendSeeMapHTML(web_messageMap)); //Sending an HTML page SeeMap
}
void handle_seemapdown(void) {
  top_line_number -=SHIFTING_ROWS_IN_THE_TABLE;
  if(top_line_number < (0 + NUMBER_OF_ROWS_IN_THE_TABLE - 1) ) top_line_number= 0 + NUMBER_OF_ROWS_IN_THE_TABLE - 1;
  web_messageMap=mapObstacleAndPath(top_line_number);
  server.send(200, "text/html", SendSeeMapHTML(web_messageMap)); //Sending an HTML page SeeMap
}

//Function for generating an HTML page
String SendHTML(void) {
  String html = R"(
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>SearcherA* Control</title>
        <style>
          body {
            text-align: center;
          }  
          button {
            width: 140px;
            margin: 10px;
            cursor: pointer; 
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2); 
          }
          form{
            text-align: center;
            margin: 10px;
          }
          input{
            width: 80px;
            cursor: pointer; 
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2);          
          }
        </style>      
    </head>
    <body>
        <h2>SearcherA* Control</h2>
        <p> current: y= )" + String(current_y) + R"(
         x= )" + String(current_x) + R"(
        <br>
        goal: Y= )" + String(goal_y) + R"(
         X= )" + String(goal_x) + R"(</p>
        <form action='/sendyx' method='GET'>
        Y: <input type='number' name='Y' step='any'>
        X: <input type='number' name='X' step='any'>
        <input type='submit' value='Send Y X'>
        </form>
        <div>
          <button onclick="location.href='/findpath'">Find Path</button>
          <br>
          <button onclick="location.href='/sendgo'">Send Go</button>
          <br>
          <button onclick="location.href='/seemap'">See Map</button>
        </div>
    </body>
    </html>
  )";
  return html; //Returning the finished HTML page
}

//Function for generating an HTML page Table
String SendSeeMapHTML(String mess) {
  String html = R"(
    <!DOCTYPE html>
    <html lang="en">
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>SearcherA* Control</title>
        <style>
          body {
            text-align: center;
          } 
          button {
            width: 130px;
            margin: 10px;
            cursor: pointer; 
            box-shadow: 0 4px 6px rgba(0, 0, 0, 0.2); 
          }
          table{
            border: 1px solid grey;
            margin-left: auto;
            margin-right: auto;
          }
          td{
            border: 1px solid grey;
            text-align:center;
          }  
        </style>      
    </head>
    <body>
        <h2>MAP</h2>
        <button onclick="location.href='/'">Home</button>
        <button onclick="location.href='/seemapup'">See Map Up 10</button>
        <button onclick="location.href='/seemapdown'">See Map Down 10</button>
        <table>)" + mess + R"(</table>
        <br>

    </body>
    </html>
  )";
  return html; //Повернення готової HTML-сторінки (Returning the finished HTML page)
}


//Формування рядка web_messageMap як карти перешкод та шляху
//Forming the web_messageMap string as a map of obstacles and path
String mapObstacleAndPath(int top_line_number){
  String web_m="";
  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) { //Taking the mutex to safely access shared data
    Coord c;
    int y_from, y_to;
    y_from = top_line_number;
    if(y_from > SIZE_MAP_Y - 1) y_from = SIZE_MAP_Y - 1 ;
    if(y_from < 0 + NUMBER_OF_ROWS_IN_THE_TABLE - 1) y_from = 0 + NUMBER_OF_ROWS_IN_THE_TABLE - 1;
    y_to=top_line_number - (NUMBER_OF_ROWS_IN_THE_TABLE-1);
    if(y_to < 0) y_to = 0 ;
    
    for(int y =y_from; y>=y_to; y--){
      web_m +="<tr>";
      web_m += "<td>"+String(y)+"</td>";
      for(int x=0; x<SIZE_MAP_X; x++) {
        c.y=y;
        c.x=x;
        web_m += "<td>";
        int index=indexFindPointCoords(pathSet, pathSetPar.setSize, c);
        if( index !=-1 ){
          if(index == (pathSetPar.setSize-1)) web_m +="A";
          else if(index == 0) web_m +="C";
          else web_m +="B";
        } 
        else if(indexFindPointCoords(obstacleSet, obstacleSetPar.setSize, c) !=-1) web_m +="X";
        else web_m +="...";
        web_m += "</td>";
      }
      web_m +="</tr>";
    }
    xSemaphoreGive(xMutex); //Releasing the mutex after accessing shared data
  } 
  else {
    Serial.println("Failed to take mutex in mapObstacleAndPath!"); //Printing an error message if failed to take mutex
    web_messageMap = "Error: Unable to access map data."; //Setting an error message for the web page
  } 
  return web_m;
}  

//Знайти точку з координатами в одновимірній матриці координат
//Find a point with coordinates in a one-dimensional matrix of coordinates
int indexFindPointCoords( Coord *set, uint8_t setSize, Coord &c) {
  if(setSize == 0) return -1;
  //Поиск в списке
  int index =-1;
  for(int i=0; i < setSize; i++){
    //Serial.print(i); Serial.println("(1)");
    if(set[i].y == c.y and set[i].x == c.x){
      index=i;
      break;
    }
  }
  return index;
}

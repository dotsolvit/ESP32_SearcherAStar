//Function OLED
//OLED дисплей 0.96" I2C 128x64 (желто-синий)
//Драйвер OLED модуля: SSD1306
//Lib:  use two Adafruit libraries: Adafruit_SSD1306 library and Adafruit_GFX library.
//Adafruit_SSD1306 by Adafruit 2.5.16
//Adafruit_GFX library by Adafruit 1.12.6



#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.hpp"
#include "funcOLED.hpp"

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Display initialization 
void initDisplay(void){
  //Wire.begin();  //in main.cpp

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  
  //Test display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("Hi, SearcherAStar!");
  display.display(); 
}

//Display WiFi sign
void displayWiFiSign(void){
  display.fillRect(0, 0, 19, 15, BLACK);
  display.setCursor(0, 0);
  display.println(" W ");
  display.display();
}

//Display angle
void displayAngle(int angle){
    static int prev_angle = -399; // Змінна для збереження останнього відображеного кута (Variable to store the last displayed angle)
    //Display only if angle has changed (to reduce flickering and unnecessary updates)
    if (angle != prev_angle) {
        prev_angle = angle; // Update the last displayed angle
        display.fillRect(20, 0, 60, 15, BLACK);
        display.setCursor(16, 0);
        display.print("Angl= ");
        display.println(angle);
        display.display();
    }
}

//Display battery
void displayBattery(void){
    static int prev_percentage = -1; // Змінна для збереження останнього відображеного відсотка заряду (Variable to store the last displayed battery percentage)
    // 1. Measurement
    int analogBatt=analogRead(PIN_BAT);
    float voltage = (float(analogBatt) / 4095.0) * 3.30 * RESISTOR_RATIO;
    // 2. Conversion to percentage (for 2S Li-ion 0%=6.6V - 100%=8.4V)
    int percentage = map(voltage * 100, 660, 840, 0, 100);
    percentage = constrain(percentage, 0, 100); 
    // 3. Display only if percentage has changed (to reduce flickering and unnecessary updates)
    if (percentage != prev_percentage) {
        prev_percentage = percentage; // Update the last displayed percentage
        // 4. Display on OLED   
        display.fillRect(80, 0, 48, 15, BLACK);
        display.setCursor(80, 0);
        display.print("B= ");
        display.print(percentage);
        display.println(" %");
        display.display();
    }
}

//Display Fill Rect White
void displayFillRectWhite(int str){ 
  //str = 1, 2 or 3 
  if(str<1) str=1; //16
  if(str>3) str=3; //48
  display.fillRect(0, 16*str, 127, 15, WHITE);
  display.display();
}

//Display Message
void displayMessage(int str, const char* srt_in1, int int_in, const char* str_in2){
  if(str<1) str=1; //16
  if(str>3) str=3; //48
  int pos=16*str;
  display.fillRect(0, pos, 127, 15, BLACK);
  display.setCursor(5, pos);
  display.setTextSize(1);
  display.print(srt_in1);
  if(str_in2 != ""){
   display.print(int_in);
   display.print(str_in2);
  }
  display.display();
}

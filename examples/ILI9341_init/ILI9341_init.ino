#include <SPI.h>  
#define _ILI9341_
#include <TFTCanvas.h>    

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  Serial.println("Hi ILI9341");
  tft.init(); 
  tft.fillScreen(YELLOW);
  tft.print("TFTMatrix ILI9341");   
  tft.setRotation(1);  // Set for landscape display on Esplora
  tft.setTextWrap(false); 
  
  tft.canvas(12,12,true);
}

void loop() {
  
}
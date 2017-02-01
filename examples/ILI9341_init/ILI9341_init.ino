#include <SPI.h>  
#include <Adafruit_GFX.h>    // Core graphics library
#include <TFTCanvas.h>    

#define TFT_CS   7
#define TFT_DC   6

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

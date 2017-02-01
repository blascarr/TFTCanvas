#include <SPI.h>
#include <Adafruit_GFX.h>  

#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  tft.init(2, BLUE);
  
}

void loop() {
  
}
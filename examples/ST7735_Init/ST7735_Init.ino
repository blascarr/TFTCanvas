#include <SPI.h>
#define _ST7735H_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  tft.init(2, YELLOW);
  tft.canvas(16,16, true,BLACK);
}

void loop() {
  
}
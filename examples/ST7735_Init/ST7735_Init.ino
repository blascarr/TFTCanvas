#include <SPI.h>
#define DEBUG
#define TFTDEBUG
#define SERIAL_DEBUG
#define _ST7735H_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);

  delay(3000);
  tft.init(2, MAGENTA);
  tft.canvas(16,16, true,BLACK);
}

void loop() {
  
}

#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN);
  tft.canvas(20,12, true);
}

void loop() {
  
}
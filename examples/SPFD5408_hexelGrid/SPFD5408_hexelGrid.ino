#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  Serial.println(tft.size_x);
  Serial.println(tft.size_y);
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN,1);

  tft.canvas(5,6, true);
  Serial.println(tft.mode);
  delay(4000);
  tft.setHexelMode();

  tft.canvas(5,6, true);
}

void loop() {
  
}

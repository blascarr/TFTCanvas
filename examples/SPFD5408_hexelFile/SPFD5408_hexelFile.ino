#define _SPFD5408_
#include <SD.h>
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

//TFTMatrix <7, 7, uint16_t> model;
//hexelFile hx("Hexel.csv"); 
 
void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  Serial.println(tft.size_x);
  Serial.println(tft.size_y);
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN,1);

  tft.setHexelMode();

  tft.canvas(2,6, true);
  tft.setRoot("Arduino.txt");

  Serial.println(tft._fileRoot);
  
  tft.openFile();
  tft.readFile(true, true);
}

void loop() {
  
}

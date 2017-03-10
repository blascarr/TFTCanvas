#define _SPFD5408_
#include <SPI.h>
#include <SD.h>
#include <ArduParser.h>

#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

arduParser hexelParser ("",",","");

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

  tft.setRoot("hexel.csv", "/Arduino/");

  Serial.println(tft._fileRoot);
  
  tft.openFile();
  tft.drawCSV();
  
}

void loop() {
  
}

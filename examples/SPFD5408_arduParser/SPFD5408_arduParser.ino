#define _SPFD5408_
#include <TFTCanvas.h>

#include <ArduParser.h>

//Definition bauds per second for Serial Communication (ESP8266, BlueTooth, Serial)
long bps=115200;
arduParser TFTParser ("<",",",">"); 

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(bps);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);

  String ParserLine = "<1,1,39,167,121,255>";
  //ParserLine = "<1, 1, 39, 167, 121, 255>";
  
  Serial.println("TFTParser Init!");

  parseString d;
  d=TFTParser.parser(ParserLine);

  //if(TFTParser.entry){
      for(int n=0; n < numdata; n++){
        Serial << "TFTParser data: ";
        Serial.print( TFTParser.data.dataString[n]); 
        Serial <<"  Type:  ";
        Serial.println(TFTParser.data.typeString[n]);
      }
  //}
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN);
  tft.canvas(20,12, true);
}

void loop() {
  
  if (Serial.available()){
    String dataBT = Serial.readString();
    
  
  }
}

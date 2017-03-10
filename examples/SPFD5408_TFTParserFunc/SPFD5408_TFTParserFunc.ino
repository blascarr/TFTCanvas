#include <SoftwareSerial.h>
#include <ArduParser.h>
#define _SPFD5408_
#include <TFTCanvas.h>

#define pinLED 13

TFTCanvas tft = TFTCanvas();
//Definition bauds per second for Serial Communication (ESP8266, BlueTooth, Serial)
long bps=9600;

SoftwareSerial BT (0,1);
arduParser prsr("<","|",">");

int width, height;

void setup() {
  Serial.begin(bps);
  BT.begin(bps);
  uint16_t TFTID = tft.readID();
  tft.init(TFTID, GREEN);

  tft.println("TFTFirmata Ready");
  
  width = tft.width();
  height = tft.height();

  pinMode(pinLED, OUTPUT);
}
 
void loop() {
  
  unsigned long currentMillis = millis();
 
  if (BT.available()){
    String dataBT = BT.readString();
    Serial.println(dataBT);
    //
    while(dataBT.length() >0){
      Serial.println();
      int indexTo = dataBT.indexOf(prsr.END_CMD);
      //Serial.println(indexTo );
      String subLine = dataBT.substring(0,indexTo+prsr.END_CMD.length());
      Serial.print("Rest dataBT Before: ");Serial.println(dataBT);
      dataBT = dataBT.substring(indexTo+prsr.END_CMD.length());
      Serial.print("Rest dataBT After: ");Serial.println(dataBT);
      Serial.print("SubLine: ");Serial.println(subLine);
      
      Serial.print("RAM Usage before:  ");
      Serial.println(freeRAM());
  
      prsr.parser(subLine);
      Serial.print("RAM Usage After:  ");
      Serial.println(freeRAM());
      delay(200);
      
      if(prsr.entry){
        Serial.println("Entry Success:   ");
        
  
        for(int n=0; n < prsr.ndata; n++){
          Serial.print(prsr.data.typeString[n]);
          Serial.print(" : ");
          Serial.println(prsr.data.dataString[n]);
        }
        
        digitalWrite(pinLED, HIGH);
        delay(1000);
        digitalWrite(pinLED, LOW);
        //Cuando recoge una linea completa con caracteres raros hay problemas de lectura.
        
        
        /*if(prsr.data.dataString[0]=="CL"){
          Serial.println("Clean");
          
          tft.clean();
          Serial.print("RAM Usage in Clean:  ");
          Serial.println(freeRAM());
        }*/
        
        if(prsr.data.dataString[0]=="DR"){
          Serial.print("Draw ");
          Serial.print("Data 1:  ");
          //Serial.print(prsr.getFloat(1));
          Serial.print("   Data 2:  ");
          //Serial.println(prsr.getFloat(2));
          Serial.print("RAM Usage in Draw:  ");
          Serial.println(freeRAM());
          TFTMatrix<1,1,uint16_t> P;
          P.m(0,0)=RED;
          P.setOrigin(prsr.getFloat(1),prsr.getFloat(2));
          P.draw(tft);
        }
    
        if(prsr.data.dataString[0]=="GR"){
          
          tft.canvas(12,16,true, BLACK);
          Serial.print("Grid ");
          Serial.println(prsr.data.dataString[1]);
          Serial.println(prsr.getInt(1));
          Serial.print("RAM Usage in Grid:  ");
          Serial.println(freeRAM());
        }
  /*
        if(prsr.data.dataString[0]=="FLOAT"){
          Serial.print("Move ");
          Serial.print("Data 1:  ");
          Serial.print(prsr.getFloat(1));
          Serial.print("   Data 2:  ");
          Serial.println(prsr.getFloat(2));
          
          Serial.print("RAM Usage in Move:  ");
          Serial.println(freeRAM());
          //Serial.println(prsr.getInt(1));
          //Serial.println(prsr.getInt(2));
          tft.drawPixel(prsr.getFloat(1), prsr.getFloat(2), RED);
        }
        
        if(prsr.data.dataString[0]=="INT"){
          Serial.print("Move ");
          Serial.print("Data 1:  ");
          Serial.print(prsr.getInt(1));
          Serial.print("   Data 2:  ");
          Serial.println(prsr.getInt(2));
          
          Serial.print("RAM Usage in Move:  ");
          Serial.println(freeRAM());
          //Serial.println(prsr.getInt(1));
          //Serial.println(prsr.getInt(2));
          tft.drawPixel(prsr.getInt(1), prsr.getInt(2), RED);
        }*/
        
        //BT.flush();
      }  
    }  
  }
  
  //Serial.print("Time reading Parser --> \t");
  //Serial.println(millis() - currentMillis );
}

int freeRAM () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

TFTMatrix <7, 7, uint16_t> model;

  int rows = 20;
  int cols = 16;
  
void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN);

  tft.canvas(rows,cols, true);

  model.m.Fill(0);

  //Create a 3x3 subMatrix and we repeat 4 times on a greater matrix to evaluate its movement
  uint16_t arrayA[3][3] = {{YELLOW,BLUE,BLUE},{BLACK,CYAN,CYAN},{YELLOW,BLUE, BLUE }};
  model.m.Submatrix(Range<3>(0), Range<3>(0))= arrayA;
  model.m.Submatrix(Range<3>(4), Range<3>(0))= arrayA;
  model.m.Submatrix(Range<3>(4), Range<3>(4))= arrayA;
  model.m.Submatrix(Range<3>(0), Range<3>(4))= arrayA;
  
}

void loop() {

  //Set origin Reference of the entire matrix on the screen
  model.setOrigin(2,3);
  
  int delayTime = 400;
  model.draw(tft);
  delay(delayTime);
  for (int i = 0; i < 5 ; i++){
    model.move(tft,1,-2);
    delay(delayTime);
    model.move(tft,1,2);
    delay(delayTime);
  }

  for (int i = 0; i < 5 ; i++){
    model.move(tft,-1,-2);
    delay(delayTime);
    model.move(tft,-1,2);
    delay(delayTime);
  }
  
  model.clean(tft);
}

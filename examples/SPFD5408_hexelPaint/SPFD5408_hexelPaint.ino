#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

TFTMatrix <7, 7, uint16_t> model;
  
void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  Serial.println(tft.size_x);
  Serial.println(tft.size_y);
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN,0);

  tft.setHexelMode();

  tft.canvas(5,5, true);

   model.m.Fill(0);

  //Create a 3x3 subMatrix and we repeat 4 times on a greater matrix to evaluate its movement
  uint16_t arrayA[3][3] = {{YELLOW,BLUE,BLUE},{BLACK,CYAN,CYAN},{YELLOW,BLUE, BLUE }};
  model.m.Submatrix(Range<3>(0), Range<3>(0))= arrayA;
  model.m.Submatrix(Range<3>(4), Range<3>(0))= arrayA;
  model.m.Submatrix(Range<3>(4), Range<3>(4))= arrayA;
  model.m.Submatrix(Range<3>(0), Range<3>(4))= arrayA;

  model.drawHexels(tft);
}

void loop() {
  
}

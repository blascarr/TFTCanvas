#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

TFTMatrix <7, 7, uint16_t> eye;

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

  eye.m.Fill(0);

  //Create a 3x3 subMatrix and we repeat 4 times on a greater matrix to evaluate its movement
  uint16_t arrayA[3][3] = {{YELLOW,BLUE,BLUE},{BLACK,YELLOW,YELLOW},{YELLOW,BLUE, BLACK }};
  eye.m.Submatrix(Range<3>(0), Range<3>(0))= arrayA;
  eye.m.Submatrix(Range<3>(4), Range<3>(0))= arrayA;
  eye.m.Submatrix(Range<3>(4), Range<3>(4))= arrayA;
  eye.m.Submatrix(Range<3>(0), Range<3>(4))= arrayA;
  
}

void loop() {

  //Set origin Reference of the entire matrix on the screen
  eye.setOrigin(2,4);
  
  int delayTime = 600;
  eye.draw(tft);
  delay(delayTime);
  for (int i = 0; i < 6 ; i++){
    eye.move(tft,1,-2);
    delay(delayTime);
    eye.move(tft,1,2);
    delay(delayTime);
  }
  eye.clean(tft);
}

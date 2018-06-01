#define _SPFD5408_
//#define _ILI9341_
#include <Robo2Duino.h>
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  
  //Initialize chip TFT screen
  tft.init(TFTID, WHITE);

  int width = tft.width();
  int height = tft.height();
  Pose2D P(width/2,height/2, 0);
  tft.trplot(P);

  Pose2D A(width/10,height/10, 0);
  Pose2D B(width*9/10,height*9/10, 0);
  tft.line(A,B, BLUE);
  Point2D a(width*9/10,height/2, 0);
  Point2D b(width/2,height*9/10, 0);
  tft.line(a,b);
}


void loop() {
  
}

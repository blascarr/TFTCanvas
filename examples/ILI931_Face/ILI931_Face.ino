#define _ILI9341_
//#define _ST7735H_
//#define _SPFD5408_
#include <TFTCanvas.h>
#include <DFPlayer_Mini_Mp3.h>
SoftwareSerial DF(2, 3); // RX, TX

TFTCanvas tft = TFTCanvas();

TFTMatrix <5, 5, uint16_t> eyei;
TFTMatrix <5, 5, uint16_t> eyed;
TFTMatrix <5, 5, uint16_t> browi;
TFTMatrix <5, 5, uint16_t> browd;
TFTMatrix <6, 8, uint16_t> mouth;

  int rows = 20;
  int cols = 16;
  
void setup() {
  Serial.begin(9600);

  DF.begin (9600);
  mp3_set_serial (DF);    //set softwareSerial for DFPlayer-mini mp3 module 
  delay(1);                     // delay 1ms to set volume
  mp3_set_volume (15);          // value 0~30

  Serial.println("First song");
  mp3_next();
  mp3_play(1);
  
  //Initialize chip TFT screen
  tft.init();
  tft.setRotation(3);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.setCursor(tft.width()/2-85,50);
  tft.println("Hi, my name is: ");
  tft.setTextSize(4);
  tft.setTextColor(BLUE);
  tft.setCursor(tft.width()/2-80,tft.height()/2-40);
  tft.println("STEWIE");
  delay(2000);
  tft.setRotation(0);
  tft.clean();
  tft.canvas(rows,cols, true);

  eyei.m.Fill(0);
  eyed.m.Fill(0);
  browi.m.Fill(0);
  browd.m.Fill(0);
  mouth.m.Fill(0);
  eyei.setOrigin(6,4);
  eyed.setOrigin(6,9);

  browi.setOrigin(0,3);
  browd.setOrigin(0,10);
  mouth.setOrigin(12,5);
  //Create a 3x3 subMatrix and we repeat 4 times on a greater matrix to evaluate its movement
  uint16_t arrayA[2][4] = {{BLACK,BLACK,BLACK,BLACK},{BLACK,BLACK,BLACK, BLACK }};
  eyei.m.Submatrix(Range<2>(0), Range<4>(0))= arrayA;
  eyed.m.Submatrix(Range<2>(0), Range<4>(0))= arrayA;


  uint16_t arrayB[3][2] = {{WHITE,BLACK},{BLACK,WHITE},{BLACK,WHITE}};
  uint16_t arrayC[3][2] = {{BLACK,WHITE},{BLACK,WHITE},{WHITE,BLACK}};
  browi.m.Submatrix(Range<3>(0), Range<2>(1))= arrayB;
  browd.m.Submatrix(Range<3>(0), Range<2>(1))= arrayC;

  uint16_t arrayM[6][2] = {{BLACK,WHITE},{WHITE,BLACK},{WHITE,BLACK},{WHITE,BLACK},{WHITE,BLACK},{BLACK,WHITE}};
  mouth.m.Submatrix(Range<6>(0), Range<2>(0))= arrayM;

  browi.draw(&tft);
  browd.draw(&tft);
  mouth.draw(&tft);
}

void loop() {

    eyei.move(&tft,0,1);
    eyed.move(&tft,0,1);
    eyed.update(&tft);
    eyei.update(&tft);
    delay(500);
    eyei.move(&tft,0,-1);
    eyed.move(&tft,0,-1);
    eyed.update(&tft);
    eyei.update(&tft);
    delay(500);
  

}

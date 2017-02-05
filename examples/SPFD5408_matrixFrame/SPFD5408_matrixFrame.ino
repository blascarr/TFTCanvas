#define _SPFD5408_
#include <TFTCanvas.h>

TFTCanvas tft = TFTCanvas();

  int rows = 20;
  int cols = 16;

//PixelFrame Class for Animation

class pixelFrame{
    public:

      //Number of Frames
      int nFrames= 10;
      //Declare times for each frame
      unsigned long keyFrames[30] = {
        5000,
        10000,
        15000,
        20000,
        35000,
        40000,
        54000,
        58000,
        62000,
        66000
      };
    void draw(TFTCanvas tft);

    void Frame0(){
      //face();
    }
    void Frame1(){
      //clean();
    }
    void Frame2(){
      //boot();
    }
    void Frame3(){
      //cleanboot();
    }
    void Frame4(){}
    void Frame5(){}
    void Frame6(){}
    void Frame7(){}
    void Frame8(){}
    void Frame9(){}
    void Frame10(){}
    
    // typedef for class function
    typedef void (pixelFrame::*GeneralFunction) ();
   
   // array of function pointers
     GeneralFunction FrameArray [11] ={
        &pixelFrame::Frame0, 
        &pixelFrame::Frame1, 
        &pixelFrame::Frame2, 
        &pixelFrame::Frame3, 
        &pixelFrame::Frame4, 
        &pixelFrame::Frame5, 
        &pixelFrame::Frame6, 
        &pixelFrame::Frame7, 
        &pixelFrame::Frame8, 
        &pixelFrame::Frame9, 
        &pixelFrame::Frame10, 
      };
   
};

int indexKeyFrame =0;
unsigned long keyRange= 100;

pixelFrame roboto; 

TFTMatrix <5, 5, uint16_t> eye_i;
TFTMatrix <5, 5, uint16_t> eye_d;
TFTMatrix <5, 5, uint16_t> brow_i;
TFTMatrix <5, 5, uint16_t> brow_d;
TFTMatrix <6, 7, uint16_t> mouth;

void setup() {
  Serial.begin(9600);
  uint16_t TFTID = tft.readID();
  Serial.print("TFT ID: ");
  Serial.println(TFTID);
  
  //Initialize chip TFT screen
  tft.init(TFTID, GREEN);

  tft.canvas(rows,cols, true);

  eye_i.m.Fill(0);
  eye_d.m.Fill(0);
  brow_i.m.Fill(0);
  brow_d.m.Fill(0);
  mouth.m.Fill(0);
  
  //Create a 3x3 subMatrix and we repeat 4 times on a greater matrix to evaluate its movement
  uint16_t array_eye[4][2] = {{BLACK,BLACK},{BLACK,BLACK},{BLACK,BLACK},{BLACK,BLACK}};
  eye_i.m.Submatrix(Range<4>(0), Range<2>(2))= array_eye;
  eye_d.m.Submatrix(Range<4>(0), Range<2>(1))= array_eye;

  eye_i.setOrigin(3,6);
  eye_d.setOrigin(13,6);

  eye_i.draw(tft);
  eye_d.draw(tft);
  delay(1000);

  uint16_t array_browi[2][3] = {{0,BLACK,BLACK},{BLACK,0,0}};
  uint16_t array_browd[2][3] = {{BLACK,BLACK,0},{0,0,BLACK}};
  brow_i.m.Submatrix(Range<2>(2), Range<3>(1))= array_browi;
  brow_d.m.Submatrix(Range<2>(2), Range<3>(1))= array_browd;

  brow_i.setOrigin(3,0);
  brow_d.setOrigin(13,0);
  
  brow_i.draw(tft);
  brow_d.draw(tft);
  delay(1000);

  uint16_t array_mouth[2][5] = {{BLACK,0,0,0,BLACK},{0,BLACK,BLACK,BLACK,0}};
  mouth.m.Submatrix(Range<2>(1), Range<5>(1))= array_mouth;
  mouth.setOrigin(7,10);
  mouth.draw(tft);
}

void loop() {

  unsigned long currentMillis = millis(); 
  if((currentMillis >= (roboto.keyFrames[indexKeyFrame])-keyRange)&&(currentMillis <= (roboto.keyFrames[indexKeyFrame])+keyRange)){
    tft.print("Time Before: ");
    tft.print(currentMillis);
    
    //Manage Functions by Reference
    pixelFrame::GeneralFunction k = roboto.pixelFrame::FrameArray [indexKeyFrame];
    // call the function
    (roboto.*k) ();
    
    tft.print("   Time After: ");
    tft.println(millis());
    indexKeyFrame++;
  }

}

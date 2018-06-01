#define _ILI9341_
//#define _ST7735H_
//#define _SPFD5408_
#include <TFTCanvas.h>

unsigned long auxtimer;

TFTCanvas tft = TFTCanvas( 7, 6, 5 );
TFTMatrix <2, 2, uint16_t> matrix;

void setup() {
  tft.init( tft.Color565(255,255,255) , 1 );

  tft.canvas(12,16,true);
  matrix.m.Fill((tft.Color565(0,255,0)));
  matrix.setOrigin(1,1);

}

void loop() {
  if( millis() - auxtimer > 100){
    auxtimer = millis();
    matrix.m.Fill((tft.Color565((mathRandomInt(1, 255)),(mathRandomInt(1, 255)),(mathRandomInt(1, 255)))));
    matrix.move(&tft, 1, 1);
  };
  matrix.update(&tft);
}

int mathRandomInt(int min, int max) {
  if (min > max) {
    // Swap min and max to ensure min is smaller.
    int temp = min;
    min = max;
    max = temp;
  }
  return min + (rand() % (max - min + 1));
}


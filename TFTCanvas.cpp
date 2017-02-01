/* TFTCanvas
  Design and created by Blascarr

  TFTCanvas
  Name    : Blascarr
  Description: TFTCanvas.h
  version : 1.0

TFTCanvas is a generic library that supports differents TFT Screens based on Adafruit_GFX.

This library is part of a educational course to learn C++ in practice in https://github.com/blascarr/TFTCourse or http://www.blascarr.com/ webpage.

 *	
 *  This Library gives some helpful classes to manipulate Matrix Frames on a TFT Screen
 *	With TFTCanvas we extend some dependency issues for use a variety of TFTScreens with Adafruit TFT library .
 *	Nowadays is supported
 *
 *		· ST7735 			  https://github.com/adafruit/Adafruit-ST7735-Library
 *		· SPFD5408 			https://github.com/adafruit/TFTLCD-Library
 *		· ILI9341 			https://github.com/adafruit/Adafruit_ILI9341
 *
 *	BasicLinearAlgebra is needed
 *
 *	https://github.com/tomstewart89/BasicLinearAlgebra
 *
 *Written by Adrian for Blascarr
 */

#include "TFTCanvas.h"

#define debug 0
#define TFT_bps 9600
/*----------------------------------------------------------*/
/*-------  Constructor Class for ST7735 TFT Screen -------*/
/*----------------------------------------------------------*/
#ifdef _ADAFRUIT_ST7735H_
	TFTCanvas::TFTCanvas() : TFTLib( TFT_CS, TFT_DC, TFT_RST) {
		TFTCanvas::size_x = 160;
		TFTCanvas::size_y = 128;

	}

	TFTCanvas::TFTCanvas(int CS,int DC,int RST) : TFTLib ( CS, DC, RST) {
		TFTCanvas::size_x = 160;
		TFTCanvas::size_y = 128;
	}

  void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
    if(debug){
      Serial.begin(TFT_bps);
      Serial.println("TFTCanvas");
    }
    TFTCanvas::initR(tab); 
    TFTCanvas::bckg_color= bckg_color;
    TFTCanvas::fillScreen(TFTCanvas::bckg_color);
    TFTCanvas::setRotation(rotation);
  }
#endif

/*----------------------------------------------------------*/
/*-------  Constructor Class for ILI9341 TFT Screen -------*/
/*----------------------------------------------------------*/
#ifdef _ADAFRUIT_ILI9341H_
  TFTCanvas::TFTCanvas() : TFTLib( TFT_CS, TFT_DC) {
      TFTCanvas::size_x = 320;
      TFTCanvas::size_y = 240;
  }

  TFTCanvas::TFTCanvas(int CS,int DC) : TFTLib( CS, DC) {
      TFTCanvas::size_x = 320;
      TFTCanvas::size_y = 240;
  }

  void TFTCanvas::init(uint16_t bckg_color, int rotation){
    if(debug){
      Serial.begin(TFT_bps);
      Serial.println("TFTCanvas");
    }
    TFTCanvas::bckg_color= bckg_color;
    TFTCanvas::begin();
    TFTCanvas::fillScreen(TFTCanvas::bckg_color);
    TFTCanvas::setRotation(rotation);
  }
#endif

/*----------------------------------------------------------*/
/*-------  Constructor Class for SPFD5408 TFT Screen -------*/
/*----------------------------------------------------------*/

#ifdef _ADAFRUIT_TFTLCD_H_
	TFTCanvas::TFTCanvas() : TFTLib (LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET){
		TFTCanvas::size_x = 320;
		TFTCanvas::size_y = 240;
	}

	TFTCanvas::TFTCanvas(int CS,int DC,int WR,int RD, int RESET) : Adafruit_TFTLCD ( CS, DC, WR, RD, RESET) {
		TFTCanvas::size_x = 320;
		TFTCanvas::size_y = 240;
	}

	void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
		if(debug){
			Serial.begin(TFT_bps);
			Serial.println("TFTCanvas");
		}
		
		TFTCanvas::bckg_color = bckg_color;
		TFTCanvas::begin(tab);
		TFTCanvas::fillScreen(TFTCanvas::bckg_color);
		TFTCanvas::setRotation(rotation);
	}
#endif



//-------------------------------------------------//

void TFTCanvas::setSize(int sizeX, int sizeY){
	TFTCanvas::size_x = sizeX;
	TFTCanvas::size_y = sizeY;
}

void TFTCanvas::setOffset(int offsetX, int offsetY){
	TFTCanvas::offset_x = offsetX;
	TFTCanvas::offset_y = offsetY;
}

void TFTCanvas::setColor(uint16_t bckg_color){
    TFTCanvas::bckg_color= bckg_color;
}

void TFTCanvas::canvas(int8_t col,int8_t row, bool displayGrid, uint16_t color){
    int gridX;
    int gridY;

    if (TFTCanvas::getRotation()%2){
      TFTCanvas::sub_w = round(TFTCanvas::size_x /col);
      TFTCanvas::sub_h = round(TFTCanvas::size_y /row);
      gridX = TFTCanvas::sub_h;
      gridY = TFTCanvas::sub_w;
      
    }else{
      TFTCanvas::sub_w = round(TFTCanvas::size_y /col);
      TFTCanvas::sub_h = round(TFTCanvas::size_x /row);
      gridX = TFTCanvas::sub_w;
      gridY = TFTCanvas::sub_h;
      
    }

    if(debug){
      Serial.println(TFTCanvas::sub_w);
      Serial.println(TFTCanvas::sub_h);
    }
    if(displayGrid){
        for(int i = 0; i < col; ++i){
            for(int j=0; j< row; j++){

                TFTCanvas::drawRect(TFTCanvas::offset_x+i*TFTCanvas::sub_w,TFTCanvas::offset_y+j*TFTCanvas::sub_h,TFTCanvas::sub_w,TFTCanvas::sub_h,color);
            }
        }
    }
}
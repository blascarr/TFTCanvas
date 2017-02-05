#ifndef define_h
#define define_h	
	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif

	#include "BasicLinearAlgebra.h"
 	#include <SD.h>


	#define debug 0
	#define TFT_bps 9600

	#if defined(_ST7735H_) 

		#define TFTLib Adafruit_ST7735
		
		//PinOut for Arduino Esplora
		#define TFT_CS    7 	//Chip Select
		#define TFT_DC   0  	//Data/Command 
		#define TFT_RST  1
		#define TFT_SD_CS 5
		//#define mosi 16
		
		//PinOut for Arduino UNO
		#define TFT_CS    7 	//Chip Select
		#define TFT_DC   6	 	//Data/Command 
		#define TFT_RST  5
		#define TFT_SD_CS 8
		//#define sclk 15
		//#define mosi 16

		#define BLACK 0x000001
		#define WHITE 0xFFFFFF
		#define BLUE 0x0000FF
		#define YELLOW 0x00FF00
		#define ROSE 0x99AA66


	#endif


	/*--------------------------------------------------------------*/

	#if defined(_ILI9341_)
		#define TFTLib Adafruit_ILI9341

		//PinOut for Arduino UNO
		#define TFT_CS   7 		//Chip Select
		#define TFT_DC   6  	//Data/Command 
	 	#define TFT_SD_CS 8 	//SD Pin

		#define BLACK   0x0001
		#define BLUE    0x001F
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define YELLOW  0xFFE0
		#define WHITE   0xFFFF
	#endif					


	/*--------------------------------------------------------------*/
	/*-------SPFD5408 also use an identifier for TFT Chip --------- */
	/*--------------------------------------------------------------*/
	/*
			Chip name 	-	Hex Identifier:			
				9341		0x9341
							0x932X
				9325			0x9325
				9328			0x9328
				8357		0x8357
				7575		0x7575


				932			932X    0
				7575		7575    1
							9341    2
							HX8357D    3
							UNKNOWN 0xFF
	*/
	#if (!defined (_ST7735H_) && !defined(_ILI9341_) && !defined(_SPFD5408_)) || defined(_SPFD5408_)

		#define TFTLib Adafruit_TFTLCD
		//PinOut for Arduino UNO
		#define LCD_CS A3 // Chip Select goes to Analog 3
		#define LCD_CD A2 // Command/Data goes to Analog 2
		#define LCD_WR A1 // LCD Write goes to Analog 1
		#define LCD_RD A0 // LCD Read goes to Analog 0

		#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
		#define TFT_SD_CS 10
		
		#define BLACK   0x0001
		#define BLUE    0x001F
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define YELLOW  0xFFE0
		#define WHITE   0xFFFF
	#endif

	/*----------------------------------------------------------*/
	/*-------  Constructor Class for SPFD5408 TFT Screen -------*/
	/*----------------------------------------------------------*/
	#ifdef _SPFD5408_
		#include <SPFD5408_Adafruit_GFX.h>
		#include <SPFD5408_Adafruit_TFTLCD.h>

			//-------  Constructor Class for SPFD5408 TouchScreen -------
		class TFTCanvas : public TFTLib {
			
			public:
				int size_x, size_y, sub_w, sub_h;
 				int offset_x = 0;
 				int offset_y = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas (int CS,int DC,int WR,int RD, int RESET) ;
				void init(uint16_t tab = 0x9341, uint16_t bckg_color = 0xFFFFFF,int rotation = 1);

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 0);
		};
	#endif	
	/*----------------------------------------------------------*/
	/*------Constructor Definition for SPFD5408 TFT Screen------*/
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

	#ifdef _ST7735H_
		#include <Adafruit_GFX.h>
 		#include <Adafruit_ST7735.h>
		
		/*----------------------------------------------------------*/
		/*-------  Constructor Class for ST7735 TFT Screen ------- */
		/*----------------------------------------------------------*/
		class TFTCanvas : public TFTLib {
			
			public:
				int size_x, size_y, sub_w, sub_h;
 				int offset_x = 0;
 				int offset_y = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas(int CS,int DC,int RST);
				void init(uint16_t tab, uint16_t bckg_color = 0xFFFFFF, int rotation = 1);
				

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 0);
		};

	#endif

	/*----------------------------------------------------------*/
	/*-------Constructor Definition for ST7735 TFT Screen-------*/
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
	//---------Constructor Class for ILI9341 TFT Screen --------*/
	/*----------------------------------------------------------*/
	#ifdef _ILI9341_
		#include <Adafruit_GFX.h>
		#include <Adafruit_ILI9341.h>
			
			//-------  Constructor Class for ILI9341 TouchScreen -------
			class TFTCanvas : public TFTLib {
				public:
				int size_x, size_y, sub_w, sub_h;
 				int offset_x = 0;
 				int offset_y = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas(int CS,int DC);
				void init(uint16_t bckg_color = 0xFFFFFF, int rotation = 1);

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 0);
			};
	#endif


	/*----------------------------------------------------------*/
	/*-------Constructor Definition for ILI9341 TFT Screen------*/
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
	/*----------------TFTCanvas Generic Methods-----------------*/
	/*----------------------------------------------------------*/
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

	void TFTCanvas::trplot(Matrix<3, 3, float> T, int size  , uint16_t color, char label, int dir ){
		int inv_x = 1;
		int inv_y = 1;
		if ((dir == 1 ) || (dir == 2)){
			inv_y = -1;
		}

		if ((dir == 2 ) || (dir == 3)){
			inv_x = -1;
		}

		TFTCanvas::drawLine(T(0,2), T(1,2),          (T(0,2) + (inv_x)*T(0,0)*size) , (T(1,2) + (inv_y)*T(1,0)*size) , color);
		TFTCanvas::drawLine(T(0,2), T(1,2),          (T(0,2) + (inv_x)*T(0,1)*size) , (T(1,2) + (inv_y)*T(1,1)*size) , color);
		TFTCanvas::drawChar((T(0,2) + (inv_x)*T(0,0)*size- 5),    (T(1,2) + (inv_y)*T(1,0)*size -15),       'x',color,TFTCanvas::bckg_color, 1);
		TFTCanvas::drawChar((T(0,2) + (inv_x)*T(0,1)*size- 15),   (T(1,2) + (inv_y)*T(1,1)*size -5),        'y',color,TFTCanvas::bckg_color, 1);
		TFTCanvas::drawChar((T(0,2) - 10),   (T(1,2) - 10),  label, color,TFTCanvas::bckg_color, 1);
	}

	/*
	void TFTCanvas::trplot(Pose2D T, int size  , uint16_t color, char label, int dir ){
	      int inv_x = 1;
	      int inv_y = 1;
	      if ((dir == 1 ) || (dir == 2)){
	        inv_y = -1;
	      }

	      if ((dir == 2 ) || (dir == 3)){
	        inv_x = -1;
	      }

	      TFTCanvas::drawLine(T.m(0,2), T.m(1,2),          (T.m(0,2) + (inv_x)*T.m(0,0)*size) , (T.m(1,2) + (inv_y)*T.m(1,0)*size) , color);
	      TFTCanvas::drawLine(T.m(0,2), T.m(1,2),          (T.m(0,2) + (inv_x)*T.m(0,1)*size) , (T.m(1,2) + (inv_y)*T.m(1,1)*size) , color);
	      TFTCanvas::drawChar((T.m(0,2) + (inv_x)*T.m(0,0)*size- 5),    (T.m(1,2) + (inv_y)*T.m(1,0)*size -15),       'x',color,TFTCanvas::bckg_color, 1);
	      TFTCanvas::drawChar((T.m(0,2) + (inv_x)*T.m(0,1)*size- 15),   (T.m(1,2) + (inv_y)*T.m(1,1)*size -5),        'y',color,TFTCanvas::bckg_color, 1);
	      TFTCanvas::drawChar((T.m(0,2) - 10),   (T.m(1,2) - 10),  label, color,TFTCanvas::bckg_color, 1);
	}
	*/
#endif
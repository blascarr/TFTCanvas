/*
  Design and created by Blascarr

  TFTCanvas
  Name    : Blascarr
  Description: TFTCanvas.h
  version : 1.0

	TFTCanvas is a generic library that supports differents TFT Screens based on Adafruit_GFX.

	This library is part of a educational course to learn C++ in practice in https://github.com/blascarr/TFTCourse or http://www.blascarr.com/ webpage.

	This Library gives some helpful classes to manipulate Matrix Frames on a TFT Screen With TFTCanvas we extend some dependency issues for use a variety of TFTScreens with Adafruit TFT library . Nowadays is supported

    	ST7735 https://github.com/adafruit/Adafruit-ST7735-Library
    	SPFD5408 https://github.com/adafruit/TFTLCD-Library

	    ILI9341 https://github.com/adafruit/Adafruit_ILI9341

	    BasicLinearAlgebra is needed to integrate Matrix concepts and graphics terms.

	https://github.com/tomstewart89/BasicLinearAlgebra
  	
  	Blascarr invests time and resources providing this open source code like some other libraries, please
  	respect the job and support open-source software.
    
    Written by Adrian for Blascarr
*/

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

	//-------------TrPlot Method for Robo2Duino library -------*/
	#ifdef Robo2Duino_h
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
	#endif

	/*----------------------------------------------------------*/
	/*---------------- TFTMatrix Pixel Class  ------------------*/
	/*----------------------------------------------------------*/

	template<int rows, int cols = 1, class ElemT = uint16_t, class MemT = Array<rows,cols,ElemT> >

		class TFTMatrix{

			public:
				//
				Matrix<rows, cols, uint16_t, MemT> m;
				int X = 0;
				int Y = 0;
				//CG (Center of gravity)
				int CG_x=0;
				int CG_y=0;
				//Subpixels Size
				int w = 1;
				int h = 1;

				TFTMatrix(){
				    int n = 0;
				};

				void setOrigin(int X, int Y){
					TFTMatrix::X = X;
					TFTMatrix::Y = Y;
				};

				void setCG(int CG_X, int CG_Y){
					TFTMatrix::CG_x = CG_x;
					TFTMatrix::CG_y = CG_y;
				};

				void pixel_size(int size){
					TFTMatrix::w = size;
					TFTMatrix::h = size;
				};

				void pixel_size(int width, int height){
					TFTMatrix::w = width;
					TFTMatrix::h = height;
				};

				void pattern();

				void fillColor(uint16_t color=0x000000){
					for (int i=0; i<TFTMatrix::m.Rows(); i++){
						for (int j=0; j<TFTMatrix::m.Cols(); j++){
							TFTMatrix::m(i,j)=color;
						}
					}
				};

				/*-----------------------Draw Functions on TFT Canvas-----------------------------*/
				void draw(TFTCanvas canvas){

					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){
						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							if((canvas.bckg_color-TFTMatrix::m(i,j)) != canvas.bckg_color){
								canvas.fillRect(canvas.offset_x+(TFTMatrix::X+j)*canvas.sub_w,canvas.offset_y+(TFTMatrix::Y+i)*canvas.sub_h,canvas.sub_w,canvas.sub_h,TFTMatrix::m(i,j));
							}
						}
					}
				};

				void drawPixels(TFTCanvas canvas){
					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){

						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							//Problem with dark color because is equal to zero. Everything initialize with 0.

							if( ((canvas.bckg_color-TFTMatrix::m(i,j)) != canvas.bckg_color)){

							//The orientation i is for Y-Axis and j is for X_axis 
								canvas.drawPixel(canvas.offset_x+TFTMatrix::CG_x+j,canvas.offset_y+TFTMatrix::CG_y+i,TFTMatrix::m(i,j)); 

								if(debug){
									Serial.print("Row: ");
									Serial.print(i);
									Serial.print(" Column: ");
									Serial.print(j);
									Serial.print(" Color: ");
									Serial.println(TFTMatrix::m(i,j));
									Serial.println(canvas.bckg_color-TFTMatrix::m(i,j));
								}
							}
						}
					}
				};

				void clean(TFTCanvas canvas){
					TFTMatrix::cleanSubMatrix(canvas,0,0,TFTMatrix::m.Rows(),TFTMatrix::m.Cols());
				};

				void cleanSubMatrix(TFTCanvas canvas, int row, int col, int rangeRow, int rangeCol){
					int maxRow = row + rangeRow;
					int maxCol = col + rangeCol;
					
					if (row + rangeRow > TFTMatrix::m.Rows() ){
						maxRow = TFTMatrix::m.Rows();
						if (debug){
							Serial.println("Out of matrix bounds in Rows");
						}
					}

					if (col + rangeCol > TFTMatrix::m.Cols() ){
						maxCol = TFTMatrix::m.Cols();
						if (debug){
							Serial.println("Out of matrix bounds in Columns");
						}
					}
					

					for (int i=row; i< maxRow; i++){
						for (int j=col; j< maxCol; j++){
							if( ((canvas.bckg_color-TFTMatrix::m(i,j)) != canvas.bckg_color)){
								canvas.fillRect(canvas.offset_x+(TFTMatrix::X+j)*canvas.sub_w,canvas.offset_y+(TFTMatrix::Y+i)*canvas.sub_h,canvas.sub_w,canvas.sub_h,canvas.bckg_color);
							}
						}
					}
				};

				void drawSubMatrix(TFTCanvas canvas, int d_x, int d_y, int row, int col, int rangeRow, int rangeCol){
					int maxRow = abs(row + rangeRow);
					int maxCol = abs(col + rangeCol);
					
					if (row + rangeRow > TFTMatrix::m.Rows() ){
						maxRow = TFTMatrix::m.Rows();
						if (debug){
							Serial.println("Out of matrix bounds in Rows");
						}
					}

					if (col + rangeCol > TFTMatrix::m.Cols() ){
						maxCol = TFTMatrix::m.Cols();
						if (debug){
							Serial.println("Out of matrix bounds in Columns");
						}
					}
					
					for(int i = row; i < maxRow; ++i){
						for(int j = col; j < maxCol; ++j){
							if((canvas.bckg_color-TFTMatrix::m(i,j)) != canvas.bckg_color){
								//Negative sum for d_y if we want to dispose up direction like positive number
								canvas.fillRect(canvas.offset_x+(TFTMatrix::X+j+d_x)*canvas.sub_w,canvas.offset_y+(TFTMatrix::Y+i-d_y)*canvas.sub_h,canvas.sub_w,canvas.sub_h,TFTMatrix::m(i,j));
							}
						}
					}
				}

				void move(TFTCanvas canvas, int step_X,int step_Y){
					bool d_x = (step_X) >= 0;
					bool d_y = (step_Y) >= 0;
					if ((abs(step_X) >= TFTMatrix::m.Cols()) || (abs(step_Y) >= TFTMatrix::m.Rows())){
						
						//Borrar y dibujar completamente
						TFTMatrix::clean(canvas);
						TFTMatrix::setOrigin( TFTMatrix::X+step_X, TFTMatrix::Y-step_Y);
						TFTMatrix::draw(canvas);

					}else {
						//Borrar parte y dibujar los que no se repiten
						int rangeX = TFTMatrix::m.Cols() - abs(step_X);
						int rangeY = TFTMatrix::m.Rows() - abs(step_Y);

						Matrix<4, 2, int8_t> dir;
						Matrix<4, 2, int8_t> o;
						
						if(d_x && d_y){
							//Sector I
							int8_t aux_o[4][2]={{0,0},{0,abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(step_Y),0},{abs(step_Y),abs(TFTMatrix::m.Cols()-abs(step_X))}};
							o = aux_o;
							int8_t auxdraw[4][2]={{abs(step_Y),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(step_Y),abs(step_X)},{abs(TFTMatrix::m.Rows()-abs(step_Y)),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(TFTMatrix::m.Rows()-abs(step_Y)),abs(step_X)}};
							dir = auxdraw;

						}else if(!d_x && d_y){
							//Sector II
							int8_t aux_o[4][2]={{0,0},{0,abs(step_X)},{abs(step_Y),0},{abs(step_Y),abs(step_X)}};
							o = aux_o;

							int8_t auxdraw[4][2]={{abs(step_Y),abs(step_X)},{abs(step_Y),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(TFTMatrix::m.Rows()-abs(step_Y)),abs(step_X)},{abs(TFTMatrix::m.Rows()-abs(step_Y)),abs(TFTMatrix::m.Cols()-abs(step_X))}};
							dir = auxdraw;

						}else if(!d_x && !d_y){
							//Sector III
							int8_t aux_o[4][2]={{0,0},{0,abs(step_X)},{abs( TFTMatrix::m.Rows()-abs(step_Y)) ,0},{abs( TFTMatrix::m.Rows()-abs(step_Y)), abs(step_X)}};
							o = aux_o;

							int8_t auxdraw[4][2]={{abs( TFTMatrix::m.Rows()-abs(step_Y)),abs(step_X)},{abs( TFTMatrix::m.Rows()-abs(step_Y)),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(step_Y),abs(step_X)},{abs(step_Y),abs(TFTMatrix::m.Cols()-abs(step_X))}};
							dir = auxdraw;

						}else if(d_x && !d_y){
							//Sector IV
							int8_t aux_o[4][2]={{0,0},{0,abs(TFTMatrix::m.Cols()-abs(step_X))},{abs( TFTMatrix::m.Rows()-abs(step_Y)) ,0},{abs( TFTMatrix::m.Rows()-abs(step_Y)), abs(TFTMatrix::m.Cols()-abs(step_X))}};
							o = aux_o;

							int8_t auxdraw[4][2]={{abs( TFTMatrix::m.Rows()-abs(step_Y)),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs( TFTMatrix::m.Rows()-abs(step_Y)),abs(step_X)},{abs(step_Y),abs(TFTMatrix::m.Cols()-abs(step_X))},{abs(step_Y),abs(step_X)}};
							dir = auxdraw;
						}else{

						}

						/*if (debug) {
							Serial << "Point O1: " << o(0,0) << "\t" << o(0,1);
							Serial << "Size O1: " << dir(0,0) << "\t" << dir(0,1);
							Serial << "Point O2: " << o(1,0) << "\t" << o(1,1);
							Serial << "Size O2: " << dir(1,0) << "\t" << dir(1,1);
							Serial << "Point O3: " << o(2,0) << "\t" << o(2,1);
							Serial << "Size O3: " << dir(2,0) << "\t" << dir(2,1);
							Serial << "Point O4: " << o(3,0) << "\t" << o(3,1);
							Serial << "Size O4: " << dir(3,0) << "\t" << dir(3,1);

							Serial << "ClearPoint O1: " << o(0,0) << "\t" << o(0,1);
							Serial << "ClearSize O1: " << dir(3,0) << "\t" << dir(3,1);
							Serial << "ClearPoint O2: " << o(1,0) << "\t" << TFTMatrix::m.Cols()-o(1,1);
							Serial << "ClearSize O2: " << dir(2,0) << "\t" << dir(2,1);
							Serial << "ClearPoint O3: " << TFTMatrix::m.Rows()-o(2,0) << "\t" << o(2,1);
							Serial << "ClearSize O3: " << dir(1,0) << "\t" << dir(1,1);
							Serial << "ClearPoint O4: " << TFTMatrix::m.Rows()-o(3,0) << "\t" << TFTMatrix::m.Cols()-o(3,1);
							Serial << "ClearSize O4: " << dir(0,0) << "\t" << dir(0,1);
						}*/
						
						//Erase Elements
						if (!( !d_x && d_y)){cleanSubMatrix(canvas,o(0,0),o(0,1),dir(3,0),dir(3,1));}
						if (!( d_x && d_y)){cleanSubMatrix(canvas,o(1,0),TFTMatrix::m.Cols()-o(1,1),dir(2,0),dir(2,1));}
						if (!( !d_x && !d_y)){cleanSubMatrix(canvas,TFTMatrix::m.Rows()-o(2,0),o(2,1),dir(1,0),dir(1,1));}
						if (!( d_x && !d_y)){cleanSubMatrix(canvas,TFTMatrix::m.Rows()-o(3,0),TFTMatrix::m.Cols()-o(3,1),dir(0,0),dir(0,1));}
						
						//Draw Elements
						if (!( d_x && !d_y)){TFTMatrix::drawSubMatrix(canvas, step_X, step_Y, o(0,0),o(0,1),dir(0,0),dir(0,1));}
						if (!( !d_x && !d_y)){TFTMatrix::drawSubMatrix(canvas, step_X, step_Y, o(1,0),o(1,1),dir(1,0),dir(1,1));}
						if (!( d_x && d_y)){TFTMatrix::drawSubMatrix(canvas, step_X, step_Y, o(2,0),o(2,1),dir(2,0),dir(2,1));}
						if (!( !d_x && d_y)){TFTMatrix::drawSubMatrix(canvas, step_X, step_Y, o(3,0),o(3,1),dir(3,0),dir(3,1));}
						
						int row_i, col_j,range_i,range_j;

						if (( d_x && !d_y)){ 
							row_i =	o(0,0);
							col_j = o(0,1);
							range_i = dir(0,0);
							range_j = dir(0,1);
						}

						if (( !d_x && !d_y)){
							row_i =	o(1,0);
							col_j = o(1,1);
							range_i = dir(1,0);
							range_j = dir(1,1);
						}	
						if (( d_x && d_y)){
							row_i =	o(2,0);
							col_j = o(2,1);
							range_i = dir(2,0);
							range_j = dir(2,1);
						}
						if (( !d_x && d_y)){
							row_i =	o(3,0);
							col_j = o(3,1);
							range_i = dir(3,0);
							range_j = dir(3,1);
						}
						
						for (int i=row_i; i< range_i+row_i ; i++){


							for (int j=col_j; j< col_j+range_j ; j++){

								/*if (debug) {
									Serial << " CompareRow: " << i << " CompareColumn:: " << j << " \t color IJ: " << TFTMatrix::m(i,j);
									Serial << " with Row: " << i-step_Y << " Column: " << j+step_X << " \t color d_IJ: " << TFTMatrix::m(i-step_Y,j+step_X);
								}*/

								//Inner Comparation
								if(TFTMatrix::m(i,j) != TFTMatrix::m(i-step_Y,j+step_X)){
									// Sector I, IV with "!=" .... Sector II & III with ==
									if((canvas.bckg_color-TFTMatrix::m(i,j) ) == canvas.bckg_color ){
										canvas.fillRect(canvas.offset_x+(TFTMatrix::X+j+step_X)*canvas.sub_w,canvas.offset_y+(TFTMatrix::Y+i-step_Y)*canvas.sub_h,canvas.sub_w,canvas.sub_h,canvas.bckg_color);
									}else{
										canvas.fillRect(canvas.offset_x+(TFTMatrix::X+j+step_X)*canvas.sub_w,canvas.offset_y+(TFTMatrix::Y+i-step_Y)*canvas.sub_h,canvas.sub_w,canvas.sub_h,TFTMatrix::m(i,j));
									}
									
									/*if (debug) {
										Serial << i << " \t : " << j << " \t color IJ: " << TFTMatrix::m(i,j) << " \t color d_IJ: " << TFTMatrix::m(i-step_Y,j+step_X);
										Serial.println();
									}*/
								}
								
							}
						}
						

						
						TFTMatrix::setOrigin( TFTMatrix::X+step_X, TFTMatrix::Y-step_Y);
					}
				};

				void subMatrix(){
					//eye_i.m.Submatrix(Range<3>(0), Range<3>(0))= arrayA;
					//RefMatrix<3,3,Array<10,10>> eyeRef(eye.m.Submatrix(Range<3>(1),Range<3>(1)));

					//eye_i.m.Submatrix(Range<3>(3), Range<3>(3))= arrayA;
					//eye_i.m.Submatrix(Range<3>(0), Range<3>(3))= arrayA;
					//eye_i.m.Submatrix(Range<3>(3), Range<3>(0))= arrayA;

				};
		}; 
#endif
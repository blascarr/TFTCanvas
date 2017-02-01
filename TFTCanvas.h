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

#ifndef TFTCanvas_h
#define TFTCanvas_h	

	#include "config.h"
 	#include "BasicLinearAlgebra.h"
 	#include <SD.h>

	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif
 
		#ifdef _ST7735H_
	 		#include <Adafruit_ST7735.h>
			
 			//-------  Constructor Class for ST7735 TFT Screen -------

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
					/*
					//Draw the matrix defined by the grid
					//void drawMatrix(aniMatrix matrix); 
					//Draw the matrix pixel by pixel
					//void drawPixels(aniMatrix matrix);

					//Matrix Movements
					//void move(aniMatrix &matrix, int step_X,int step_Y);
					*/
			};

		#endif
		
		//-------  Constructor Class for ILI9341 TFT Screen -------

		#ifdef _ILI9341_
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

					//Draw the matrix defined by the grid
					//void drawMatrix(aniMatrix matrix); 
					//Draw the matrix pixel by pixel
					//void drawPixels(aniMatrix matrix);

					//Matrix Movements
					//void move(aniMatrix &matrix, int step_X,int step_Y);
				};
		#endif
		
		//-------  Constructor Class for SPFD5408 TFT Screen -------

		#ifdef _SPFD5408_
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

					//Draw the matrix defined by the grid
					//void drawMatrix(aniMatrix matrix); 
					//Draw the matrix pixel by pixel
					//void drawPixels(aniMatrix matrix);

					//Matrix Movements
					//void move(aniMatrix &matrix, int step_X,int step_Y);
					//void rot(aniMatrix matrix);
					//void rot3D();

			};
		#endif	

	
	

#endif

			
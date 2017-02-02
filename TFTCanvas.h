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
 *		· ST7735 			https://github.com/adafruit/Adafruit-ST7735-Library
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
	#define debug 0
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
			};
		#endif	

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
						Serial.println(TFTMatrix::Y);
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

							Serial.print("Point O1: ");
							Serial.print(o(0,0));
							Serial.print("\t");
							Serial.println(o(0,1));
							Serial.print("Size O1: ");
							Serial.print(dir(0,0));
							Serial.print("\t");
							Serial.println(dir(0,1));

							Serial.print("Point O2: ");
							Serial.print(o(1,0));
							Serial.print("\t");
							Serial.println(o(1,1));
							Serial.print("Size O2: ");
							Serial.print(dir(1,0));
							Serial.print("\t");
							Serial.println(dir(1,1));

							Serial.print("Point O3: ");
							Serial.print(o(2,0));
							Serial.print("\t");
							Serial.println(o(2,1));
							Serial.print("Size O3: ");
							Serial.print(dir(2,0));
							Serial.print("\t");
							Serial.println(dir(2,1));

							Serial.print("Point O4: ");
							Serial.print(o(3,0));
							Serial.print("\t");
							Serial.println(o(3,1));
							Serial.print("Size O4: ");
							Serial.print(dir(3,0));
							Serial.print("\t");
							Serial.println(dir(3,1));

							Serial.print("ClearPoint O1: ");
							Serial.print(o(0,0));
							Serial.print("\t");
							Serial.println(o(0,1));
							Serial.print("ClearSize O1: ");
							Serial.print(dir(3,0));
							Serial.print("\t");
							Serial.println(dir(3,1));

							Serial.print("ClearPoint O2: ");
							Serial.print(o(1,0));
							Serial.print("\t");
							Serial.println(TFTMatrix::m.Cols()-o(1,1));
							Serial.print("ClearSize O2: ");
							Serial.print(dir(2,0));
							Serial.print("\t");
							Serial.println(dir(2,1));

							Serial.print("ClearPoint O3: ");
							Serial.print(TFTMatrix::m.Rows()-o(2,0));
							Serial.print("\t");
							Serial.println(o(2,1));
							Serial.print("ClearSize O3: ");
							Serial.print(dir(1,0));
							Serial.print("\t");
							Serial.println(dir(1,1));

							Serial.print("ClearPoint O4: ");
							Serial.print(TFTMatrix::m.Rows()-o(3,0));
							Serial.print("\t");
							Serial.println(TFTMatrix::m.Cols()-o(3,1));
							Serial.print("ClearSize O4: ");
							Serial.print(dir(0,0));
							Serial.print("\t");
							Serial.println(dir(0,1));
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
									Serial.print(" CompareRow: ");
									Serial.print(i);
									Serial.print(" CompareColumn:: ");
									Serial.print(j);
									Serial.print(" \t color IJ: ");
									Serial.println(TFTMatrix::m(i,j));
									
									Serial.print(" with Row: ");
									Serial.print(i-step_Y);
									Serial.print(" Column: ");
									Serial.print(j+step_X);
									Serial.print(" \t color d_IJ: ");
									Serial.println(TFTMatrix::m(i-step_Y,j+step_X));
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
										Serial.print(i);
										Serial.print(" \t : ");
										Serial.print(j);
										Serial.print(" \t color IJ: ");
										Serial.print(TFTMatrix::m(i,j));
										Serial.print(" \t color d_IJ: ");
										Serial.println(TFTMatrix::m(i-step_Y,j+step_X));
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



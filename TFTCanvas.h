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

#ifndef _TFTcanvas_h
#define _TFTcanvas_h	
	#if defined(ARDUINO) && ARDUINO >= 100
		#include "Arduino.h"
	#else
		#include "WProgram.h"
	#endif

	#include "BasicLinearAlgebra.h"

    //DEBUG and Communication Config

    #define TFT_bps 38400

	#ifdef SERIAL_DEBUG
		#define MODE "Serial"
		#define COMM_DEBUG 0
	#else
		#define MODE "NO Serial"
	#endif

	#ifdef MEGA_DEBUG 
		#if MEGA_DEBUG == 0 
			#define COMM_DEBUG 0
		#elif MEGA_DEBUG == 1 
			#define COMM_DEBUG 1
		#elif MEGA_DEBUG == 2 
			#define COMM_DEBUG 2
		#elif MEGA_DEBUG == 3 
			#define COMM_DEBUG 3
	  	#else
	  		#define COMM_DEBUG 0
		#endif
	#endif

    #ifndef DEBUG_RX
    	#define  DEBUG_RX 2
    #endif
    #ifndef DEBUG_TX
		#define  DEBUG_TX 3
    #endif

	#ifdef BT_DEBUG
		#include <SoftwareSerial.h>
		SoftwareSerial BT(DEBUG_RX, DEBUG_TX);
		#define COMM_DEBUG 4
	#endif
	
	#if COMM_DEBUG == 0
		#define SERIALDEBUG Serial
		#define MODE "Serial HS MODE"
	#elif COMM_DEBUG == 1
		#define SERIALDEBUG Serial1
		#define MODE "Serial1 HS MODE"
	#elif COMM_DEBUG == 2
		#define SERIALDEBUG Serial2
		#define MODE "Serial2 HS MODE"
	#elif COMM_DEBUG == 3
		#define SERIALDEBUG Serial3
		#define MODE "Serial3 HS MODE"
	#elif COMM_DEBUG == 4
		#define SERIALDEBUG BT
		#define MODE "BT Mode SoftwareSerial"
	#endif

	#ifdef DEBUG
		#define  DUMP(s, v)  { SERIALDEBUG.print(F(s)); SERIALDEBUG.print(v); }
		#define  DUMPV(v)  { SERIALDEBUG.print(v); }
		#define  DUMPS(s)    { SERIALDEBUG.print(F(s));}
		#define  DUMPPRINTLN() { SERIALDEBUG.println();}
	#else
		#define  DUMP(s, v)
		#define  DUMPV(v)
		#define  DUMPS(s)
		#define  DUMPPRINTLN() 
	#endif

	#ifdef TFTDEBUG
		#define  DUMPTFT(s, v)  { SERIALDEBUG.print(F(s)); SERIALDEBUG.print(v); }
		#define  DUMPVTFT(v)  { SERIALDEBUG.print(v); }
		#define  DUMPSTFT(s)    { SERIALDEBUG.print(F(s));}
	#else
		#define  DUMPTFT(s, v)
		#define  DUMPVTFT(v)
		#define  DUMPSTFT(s)
	#endif

	//Pattern for DEBUG detection
	#ifndef DEBUG_STARTCMD 
	  #define  DEBUG_STARTCMD "{" 
	#endif
	#ifndef DEBUG_ENDCMD 
	  #define  DEBUG_ENDCMD "}"
	#endif
	#ifndef DEBUG_SEPCM 
	  #define  DEBUG_SEPCMD ","
	#endif

	#ifdef DEBUG_TOUCHSCREEN
		#define  DUMP_TOUCHSCREEN(s, v)  { SERIALDEBUG.print(DEBUG_STARTCMD); SERIALDEBUG.print(F(s)); SERIALDEBUG.print(v); SERIALDEBUG.print(DEBUG_ENDCMD); }
		#define  DUMPS_TOUCHSCREEN(s)    { SERIALDEBUG.print(DEBUG_STARTCMD); SERIALDEBUG.print(F(s)); SERIALDEBUG.print(DEBUG_ENDCMD);}
	#else
		#define  DUMP_TOUCHSCREEN(s, v)
		#define  DUMPS_TOUCHSCREEN(s) 
	#endif

    #ifdef __SD_H__
		#define FILE_MAX_FILENAME_LEN 26
		#define FILE_MAX_FILEDIR_LEN 26
		#define MAX_FILE_LEN 100
		#define BUFFPIXEL 20

    	/*struct hexel{
			int column;
			int row;
			int R;
			int G;
			int B;
			int A;
			uint16_t color;
		};*/

    	long readNumber(String &data){

    		int indexTo = data.indexOf(',');

			String num = data.substring(0,indexTo);
			num.trim();
			data = data.substring(indexTo+1);
			//DUMP("data: ", data);
			return num.toInt();
		}

		/*bool isCommentChar(char c){
			return ( c == '#' || c == '//');
		}

		bool iscomma(char c){
			return ( c == ',');
		}

		bool isnumber(char c){
			return ( (c-'0') >= 0  && (c-'0') <= 9);
		}

		bool isLF(char c){
			return ( c == '\n'  );
		}

		bool isCR(char c){
			return ( c == '\r' );
		}

		bool isCRLF(char c){
			return ( c == '\n' ||  c == '\r' );
		}

		bool isspace(char c){
			return ( c == ' ' );
		}*/

	#endif


	#if defined(_ST7735H_) 
		
		#define TFTLib Adafruit_ST7735

		#if defined(ARDUINO_AVR_ESPLORA)
			
			//PinOut for Arduino Esplora
			#define TFT_CS    7 	//Chip Select
			#define TFT_DC   0  	//Data/Command 
			#define TFT_RST  1
			#define TFT_SD_CS 5
		#else
			//PinOut for Arduino UNO
			#define TFT_CS    7 	//Chip Select
			#define TFT_DC   6	 	//Data/Command 
			#define TFT_RST  5
			#define TFT_SD_CS 8
		#endif

		#define BLACK 0x000001
		#define WHITE 0xFFFFFF
		#define BLUE 0x0000FF
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define WHITE 0xFFFFFF
		#define YELLOW 0x00FF00
		#define ROSE 0x99AA66
		#define WHITE 0xFFFFFF

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
				9325		0x9325
				9328		0x9328
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
				uint16_t size_x, size_y, sub_w, sub_h;
 				uint16_t offset_x = 0;
 				uint16_t offset_y = 0;
 				bool mode = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas (int CS,int DC,int WR,int RD, int RESET) ;
				void init(uint16_t tab = 0x9341, uint16_t bckg_color = 0xFFFFFF,int rotation = 0);

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				void setHexelMode(bool hexelmode = true);

				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label  = ' ', int dir = 1);
				uint16_t Color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::color565(r,g,b); };
				#ifdef Robo2Duino_h

				void TFTCanvas::trplot(Pose2D T, int size =30 , uint16_t color = 0x000000, char label = ' ', int dir = 1 );
				void TFTCanvas::line(Pose2D T0, Pose2D T1  , uint16_t color = 0x000000);
				void TFTCanvas::line(Point2D p0, Point2D p1 , uint16_t color = 0x000000);

				#endif

				#ifdef __SD_H__
					
					uint16_t _nline=0;				// File line
					uint16_t _pos=0;				//Line Position
					uint8_t _mode = FILE_READ;
					static const uint8_t maxFilenameLen= FILE_MAX_FILENAME_LEN;
					//char _fileName[FILE_MAX_FILENAME_LEN];					char _fileDir[FILE_MAX_FILEDIR_LEN];
					//char _fileDir[FILE_MAX_FILEDIR_LEN];

					String _fileRoot;

					mutable File _file;

					void TFTCanvas::setRoot( String fileDir = "" );

					bool TFTCanvas::openSD();
					File TFTCanvas::openFile( String fileName );
					String TFTCanvas::readln(File &f);

					void TFTCanvas::readFile( String fileName, bool SerialDebug = true, bool TFTDebug = false);
					void TFTCanvas::drawBMP(char *fileName, int x, int y);
					void TFTCanvas::drawCSV(char *fileName, int x, int y);
					#ifdef _arduparser_
						
						void TFTCanvas::drawCSV();
						
					#endif

				#endif
		};
	#endif	
	/*----------------------------------------------------------*/
	/*------Constructor Definition for SPFD5408 TFT Screen------*/
	/*----------------------------------------------------------*/

	#ifdef _ADAFRUIT_TFTLCD_H_
		TFTCanvas::TFTCanvas() : TFTLib (LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET){

		}

		TFTCanvas::TFTCanvas(int CS,int DC,int WR,int RD, int RESET) : Adafruit_TFTLCD ( CS, DC, WR, RD, RESET) {

		}

		void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps);

			TFTCanvas::bckg_color = bckg_color;
			TFTCanvas::begin(tab);
			TFTCanvas::fillScreen(TFTCanvas::bckg_color);
			TFTCanvas::setRotation(rotation);
			TFTCanvas::size_x = TFTCanvas::width();
			TFTCanvas::size_y = TFTCanvas::height();
		}
	#endif

	/*----------------------------------------------------------*/
	/*-------  Constructor Class for ST7735 TFT Screen ------- */
	/*----------------------------------------------------------*/
	#ifdef _ST7735H_
		#include <Adafruit_GFX.h>
 		#include <Adafruit_ST7735.h>
		
		class TFTCanvas : public TFTLib {
			
			public:
				uint16_t size_x, size_y, sub_w, sub_h;
 				uint16_t offset_x = 0;
 				uint16_t offset_y = 0;
 				bool mode = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas(int CS,int DC,int RST = 10);
				void init(uint16_t tab, uint16_t bckg_color = 0xFFFFFF, int rotation = 1);
				
				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				void setHexelMode(bool hexelmode = true);

				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 1 );
				uint16_t color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::Color565(r,g,b); };

				#ifdef Robo2Duino_h

				void TFTCanvas::trplot(Pose2D T, int size =30 , uint16_t color = 0x000000, char label = ' ', int dir = 1 );
				void TFTCanvas::line(Pose2D T0, Pose2D T1  , uint16_t color = 0x000000);
				void TFTCanvas::line(Point2D p0, Point2D p1 , uint16_t color = 0x000000);
				
				#endif

				#ifdef __SD_H__

					uint16_t _nline=0;
					uint16_t _mode = FILE_READ;
					static const uint8_t maxFilenameLen= FILE_MAX_FILENAME_LEN;
					
					//char _fileName[FILE_MAX_FILENAME_LEN];
					//char _fileDir[FILE_MAX_FILEDIR_LEN];

					String _fileRoot;

					mutable File _file;
					
					void TFTCanvas::setRoot( String fileDir = "" );

					bool TFTCanvas::openSD();
					File TFTCanvas::openFile( String fileName );
					String TFTCanvas::readln(File &f);

					void TFTCanvas::readFile( String fileName, bool SerialDebug = true, bool TFTDebug = false);
					void TFTCanvas::drawBMP(char *fileName, int x, int y);
					void TFTCanvas::drawCSV(char *fileName, int x, int y);
					#ifdef _arduparser_
						
						void TFTCanvas::drawCSV();
						
					#endif

				#endif
		};

	#endif

	/*----------------------------------------------------------*/
	/*-------Constructor Definition for ST7735 TFT Screen-------*/
	/*----------------------------------------------------------*/
	#ifdef _ADAFRUIT_ST7735H_
		TFTCanvas::TFTCanvas() : TFTLib( TFT_CS, TFT_DC, TFT_RST) {

		}

		TFTCanvas::TFTCanvas(int CS,int DC,int RST) : TFTLib ( CS, DC, RST) {

		}

		void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps);

			TFTCanvas::initR(tab); 
			TFTCanvas::bckg_color= bckg_color;
			TFTCanvas::fillScreen(TFTCanvas::bckg_color);
			TFTCanvas::setRotation(rotation);
			TFTCanvas::size_x = TFTCanvas::width();
			TFTCanvas::size_y = TFTCanvas::height();
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
				uint16_t size_x, size_y, sub_w, sub_h;
 				uint16_t offset_x = 0;
 				uint16_t offset_y = 0;
 				bool mode = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas(int CS,int DC);
				void init(uint16_t bckg_color = 0xFFFFFF, int rotation = 0);

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				void setHexelMode(bool hexelmode = true);

				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 1 );
				uint16_t Color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::color565(r,g,b); };

				#ifdef Robo2Duino_h

				void TFTCanvas::trplot(Pose2D T, int size =30 , uint16_t color = 0x000000, char label = ' ', int dir = 1 );
				void TFTCanvas::line(Pose2D T0, Pose2D T1  , uint16_t color = 0x000000);
				void TFTCanvas::line(Point2D p0, Point2D p1 , uint16_t color = 0x000000);
				
				#endif

				#ifdef __SD_H__

					uint16_t _nline=0;
					uint16_t _mode = FILE_READ;
					static const uint8_t maxFilenameLen= FILE_MAX_FILENAME_LEN;
					//char *_fileName[FILE_MAX_FILENAME_LEN];
					//char *_fileDir[FILE_MAX_FILEDIR_LEN];
					String _fileRoot;

					mutable File _file;
					
					void TFTCanvas::setRoot( String fileDir = "" );

					bool TFTCanvas::openSD();
					File TFTCanvas::openFile( String fileName );
					String TFTCanvas::readln(File &f);

					void TFTCanvas::readFile( String fileName, bool SerialDebug = true, bool TFTDebug = false);
					void TFTCanvas::drawBMP(char *fileName, int x, int y);
					void TFTCanvas::drawCSV(char *fileName, int x, int y);

					#ifdef _arduparser_
						
						void TFTCanvas::drawCSV();
						
					#endif

				#endif
			};
	#endif

	/*----------------------------------------------------------*/
	/*-------Constructor Definition for ILI9341 TFT Screen------*/
	/*----------------------------------------------------------*/
	#ifdef _ADAFRUIT_ILI9341H_
		TFTCanvas::TFTCanvas() : TFTLib( TFT_CS, TFT_DC) {

		}

		TFTCanvas::TFTCanvas(int CS,int DC) : TFTLib( CS, DC) {

		}

		void TFTCanvas::init(uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps)

			TFTCanvas::bckg_color= bckg_color;
			TFTCanvas::begin();
			TFTCanvas::fillScreen(TFTCanvas::bckg_color);
			TFTCanvas::setRotation(rotation);
			TFTCanvas::size_x = TFTCanvas::width();
			TFTCanvas::size_y = TFTCanvas::height();
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

	void TFTCanvas::setHexelMode(bool hexelMode){
		TFTCanvas::mode = hexelMode;
	}

	void TFTCanvas::clean(){
		TFTCanvas::fillScreen(TFTCanvas::bckg_color);
	}

	void TFTCanvas::canvas(int8_t col,int8_t row, bool displayGrid, uint16_t color){
		int gridX;
		int gridY;

		if(TFTCanvas::mode){
			TFTCanvas::sub_w = round(TFTCanvas::size_x /col);
			TFTCanvas::sub_h = TFTCanvas::sub_w*sqrt(4/3);
			gridX = TFTCanvas::sub_h;
			gridY = TFTCanvas::sub_w;
			row = TFTCanvas::size_y/ TFTCanvas::sub_h;
		}else{
			TFTCanvas::sub_w = round(TFTCanvas::size_x /col);
			TFTCanvas::sub_h = round(TFTCanvas::size_y /row);
			gridX = TFTCanvas::sub_w;
			gridY = TFTCanvas::sub_h;
		}

		DUMP("Width: ", TFTCanvas::width() );
		DUMP(" Height: ", TFTCanvas::height() );
		DUMP("\t TFTsize_x: ", TFTCanvas::size_x );
		DUMP(" TFTsize_y: ", TFTCanvas::size_y );
		DUMP("\tWidth per column: ", TFTCanvas::sub_w );
		DUMP(" Height per column: ", TFTCanvas::sub_h);
		DUMPPRINTLN();
		if(displayGrid){
			if (TFTCanvas::mode){
				for(int i = 0; i < col; ++i){
					for(int j=0; j < row; j++){

						if(!(i%2)) TFTCanvas::subhexel(TFTCanvas::offset_x + TFTCanvas::sub_w*i, TFTCanvas::offset_y + TFTCanvas::sub_h*j, TFTCanvas::sub_w, TFTCanvas::sub_h/2, 1, BLACK);
						if (i%2) TFTCanvas::subhexel(TFTCanvas::offset_x + TFTCanvas::sub_w*i, TFTCanvas::offset_y + TFTCanvas::sub_h*j, TFTCanvas::sub_w, TFTCanvas::sub_h/2, 0, BLACK);
						if (!(i%2)) TFTCanvas::subhexel(TFTCanvas::offset_x + TFTCanvas::sub_w*i, TFTCanvas::offset_y + TFTCanvas::sub_h*(j+1/2)+TFTCanvas::sub_h/2, TFTCanvas::sub_w, TFTCanvas::sub_h/2, 0, BLACK);
						if (i%2) TFTCanvas::subhexel(TFTCanvas::offset_x + TFTCanvas::sub_w*i, TFTCanvas::offset_y + TFTCanvas::sub_h*(j+1/2)+TFTCanvas::sub_h/2, TFTCanvas::sub_w, TFTCanvas::sub_h/2, 1, BLACK);
					}
				}
			}else{
				for(int i = 0; i < col; ++i){
					for(int j=0; j< row; j++){

						TFTCanvas::drawRect(TFTCanvas::offset_x+i*TFTCanvas::sub_w,TFTCanvas::offset_y+j*TFTCanvas::sub_h,TFTCanvas::sub_w,TFTCanvas::sub_h,color);
					}
				}
			}
		}
	}

	void TFTCanvas::subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color){
		float PX[] = {0 + pos_x ,w/3 + pos_x, w/2 + pos_x, 2*w/3 + pos_x, w + pos_x};
		float PY[]= {0 + pos_y, h/2+ pos_y, h+ pos_y};

		if (!sym){

		  PY[0] = h + pos_y;
		  PY[1] = h/2 + pos_y;
		  PY[2] = 0 + pos_y;
		}

		//Vamos a crear 6 subtriangulos dentro de el area determinada 

		TFTCanvas::drawTriangle(   PX[0], PY[0], PX[1], PY[2] , PX[0], PY[2], color);
		TFTCanvas::drawTriangle(   PX[0], PY[0], PX[2], PY[1] , PX[1], PY[2], color);
		TFTCanvas::drawTriangle(   PX[0], PY[0], PX[3], PY[0] , PX[2], PY[1], color);

		TFTCanvas::drawTriangle(   PX[4], PY[2], PX[3], PY[0], PX[4], PY[0] , color);
		TFTCanvas::drawTriangle(   PX[4], PY[2], PX[2], PY[1], PX[3], PY[0] , color);
		TFTCanvas::drawTriangle(   PX[4], PY[2], PX[1], PY[2] , PX[2], PY[1] , color);
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

	#ifdef Robo2Duino_h
		void TFTCanvas::trplot(Pose2D T, int size  , uint16_t color, char label, int dir ){

			TFTCanvas::drawLine(T.m(0,2), T.m(1,2),          (T.m(0,2) + (T.inv_x)*T.m(0,0)*size) , (T.m(1,2) + (T.inv_y)*T.m(1,0)*size) , color);
			TFTCanvas::drawLine(T.m(0,2), T.m(1,2),          (T.m(0,2) + (T.inv_x)*T.m(0,1)*size) , (T.m(1,2) + (T.inv_y)*T.m(1,1)*size) , color);
			TFTCanvas::drawChar((T.m(0,2) + (T.inv_x)*T.m(0,0)*size- 5),    (T.m(1,2) + (T.inv_y)*T.m(1,0)*size -15),       'x',color,TFTCanvas::bckg_color, 1);
			TFTCanvas::drawChar((T.m(0,2) + (T.inv_x)*T.m(0,1)*size- 15),   (T.m(1,2) + (T.inv_y)*T.m(1,1)*size -5),        'y',color,TFTCanvas::bckg_color, 1);
			TFTCanvas::drawChar((T.m(0,2) - 10),   (T.m(1,2) - 10),  label, color,TFTCanvas::bckg_color, 1);
		}

		void TFTCanvas::line(Pose2D T0, Pose2D T1  , uint16_t color ){
			TFTCanvas::drawLine(T0.m(0,2), T0.m(1,2), T1.m(0,2), T1.m(1,2) , color);
		}

		void TFTCanvas::line(Point2D p0, Point2D p1 , uint16_t color ){
			TFTCanvas::drawLine(p0.x, p0.y, p1.x, p1.y , color);
		}
	#endif

	#ifdef __SD_H__
		
		void TFTCanvas::setRoot( String fileDir ){
			TFTCanvas::_fileRoot = fileDir ;
		}

		bool TFTCanvas::openSD(){
			if (!SD.begin(TFT_SD_CS)) {
				DUMPS("SD Card Failed");
				return false;
			}else{
				DUMPS("SD Card Success");
				return true;
			}
		}

		File TFTCanvas::openFile( String fileName ){

			String fileFolder = TFTCanvas::_fileRoot + fileName;
			//Reset File to the beginning
			if (SD.exists( fileFolder )) {
				DUMPS("File Exists");
				//TFTCanvas::_file.close();
				TFTCanvas::_file = SD.open( fileFolder , TFTCanvas::_mode);
				return TFTCanvas::_file;
			}else{
				DUMPS("File Does not Exist");
			}

		}

		void TFTCanvas::readFile( String fileName, bool SerialDebug, bool TFTDebug){
			String fileFolder = TFTCanvas::_fileRoot + fileName;
			
			TFTCanvas::_file = openFile( fileName );

			if (TFTCanvas::_file) {
				while (TFTCanvas::_file.available()) {

					char data = TFTCanvas::_file.read();
					DUMPV(data);
					DUMPVTFT(data);
				}
				TFTCanvas::_file.close();
				
			}else {
				// if the file isn't open, pop up an error:
				DUMP("Error opening File -->  ", TFTCanvas::_fileRoot);
			}
		}

		String TFTCanvas::readln(File &f){
			
			String data = f.readStringUntil('\n') ;
			DUMPV(data);
			DUMPPRINTLN();
			return data;
			
		};

		uint16_t _read16(File &f) {
			uint16_t result;
			((uint8_t *)&result)[0] = f.read(); // LSB
			((uint8_t *)&result)[1] = f.read(); // MSB
			return result;
		}

		uint32_t _read32(File &f) {
			uint32_t result;
			((uint8_t *)&result)[0] = f.read(); // LSB
			((uint8_t *)&result)[1] = f.read();
			((uint8_t *)&result)[2] = f.read();
			((uint8_t *)&result)[3] = f.read(); // MSB
			return result;
		}

		void TFTCanvas::drawBMP(char *fileName, int x, int y){
			File     bmpFile;
			int      bmpWidth, bmpHeight;   // W+H in pixels
			uint8_t  bmpDepth;              // Bit depth (currently must be 24)
			uint32_t bmpImageoffset;        // Start of image data in file
			uint32_t rowSize;               // Not always = bmpWidth; may have padding
			uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
				//#ifdef _SPFD5408_
					uint16_t lcdbuffer[BUFFPIXEL];  // pixel out buffer (16-bit per pixel)
					uint8_t  lcdidx = 0;
					boolean  first = true;
				//#endif
			uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
			boolean  goodBmp = false;       // Set to true on valid header parse
			boolean  flip    = true;        // BMP is stored bottom-to-top
			int      w, h, row, col;
			uint8_t  r, g, b;
			uint32_t pos = 0, startTime = millis();

			if((x >= TFTCanvas::width()) || (y >= TFTCanvas::height())) return;

			// Open requested file on SD card
			if ((bmpFile = SD.open(fileName)) == NULL) {
				DUMPS("File Not Found" ); DUMPPRINTLN();
				return;
			}

			if(_read16(bmpFile) == 0x4D42) { // BMP signature

				(void)_read32(bmpFile); // Read & ignore creator bytes
				DUMP("File size: ", _read32(bmpFile) ); 
				bmpImageoffset = _read32(bmpFile); // Start of image data
				DUMP(" Header size: ", _read32(bmpFile) );

				// Read DIB header

				bmpWidth  = _read32(bmpFile);
				bmpHeight = _read32(bmpFile);
				if(_read16(bmpFile) == 1) { // # planes -- must be '1'
					bmpDepth = _read16(bmpFile); // bits per pixel

					if((bmpDepth == 24) && (_read32(bmpFile) == 0)) { // 0 = uncompressed

						goodBmp = true; // Supported BMP format -- proceed!

						// BMP rows are padded (if needed) to 4-byte boundary
						rowSize = (bmpWidth * 3 + 3) & ~3;

						// If bmpHeight is negative, image is in top-down order.
						// This is not canon but has been observed in the wild.
						if(bmpHeight < 0) {
							bmpHeight = -bmpHeight;
							flip      = false;
						}

						// Crop area to be loaded
						w = bmpWidth;
						h = bmpHeight;
						if((x+w-1) >= TFTCanvas::width())  w = TFTCanvas::width()  - x;
						if((y+h-1) >= TFTCanvas::height()) h = TFTCanvas::height() - y;

						// Set TFT address window to clipped image bounds
						TFTCanvas::setAddrWindow(x, y, x+w-1, y+h-1);

						for (row=0; row<h; row++) { // For each scanline...
							// Seek to start of scan line.  It might seem labor-
							// intensive to be doing this on every line, but this
							// method covers a lot of gritty details like cropping
							// and scanline padding.  Also, the seek only takes
							// place if the file position actually needs to change
							// (avoids a lot of cluster math in SD library).
							if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
								pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
							else     // Bitmap is stored top-to-bottom
								pos = bmpImageoffset + row * rowSize;
							if(bmpFile.position() != pos) { // Need seek?
								bmpFile.seek(pos);
								buffidx = sizeof(sdbuffer); // Force buffer reload
							}

							for (col=0; col<w; col++) { // For each column...
								// Time to read more pixel data?
								if (buffidx >= sizeof(sdbuffer)) { // Indeed
									// Push LCD buffer to the display first
									#ifdef _SPFD5408_
										if(lcdidx > 0) {
											TFTCanvas::pushColors(lcdbuffer, lcdidx, first);
											lcdidx = 0;
											first  = false;
										}
									#endif
									bmpFile.read(sdbuffer, sizeof(sdbuffer));
									buffidx = 0; // Set index to beginning
								}

								// Convert pixel from BMP to TFT format
								b = sdbuffer[buffidx++];
								g = sdbuffer[buffidx++];
								r = sdbuffer[buffidx++];
									#ifdef _SPFD5408_
										lcdbuffer[lcdidx++] = TFTCanvas::Color565(r,g,b);
									#elif ( defined (_ST7735H_) || defined(_ILI9341_))
										TFTCanvas::pushColor(TFTCanvas::color565(r,g,b));
									#endif
								} // end pixel
							} // end scanline
							// Write any remaining data to LCD
							#ifdef _SPFD5408_
								if(lcdidx > 0) {
									TFTCanvas::pushColors(lcdbuffer, lcdidx, first);	
								} 
							#endif
							DUMP("Loaded in: ", millis() - startTime);
						} // end goodBmp
					}
				}

			bmpFile.close();
			if(!goodBmp) DUMPS("BMP format not recognized.");
		}

		
	#endif


	/*--------------------------------------------------------------------------------------------------------------------*/

	/*--------------------------------------------------------------------------------------------------------------------*/
								/*----------------------------------------------------------*/
								/*-------------------- TFTMatrix Class ---------------------*/
								/*----------------------------------------------------------*/
	/*--------------------------------------------------------------------------------------------------------------------*/

	/*--------------------------------------------------------------------------------------------------------------------*/
		
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
								DUMP("Row: ",i);
								DUMP(" Column: ",j);
								DUMP(" Color: ",TFTMatrix::m(i,j));
								DUMP(" Canvas ColorDiff: ",canvas.bckg_color-TFTMatrix::m(i,j));
							}
						}
					}
				};

				/*void drawHexel(TFTCanvas canvas, int row, int col, uint16_t color){
					int d_w=canvas.sub_w ;
					int d_h=canvas.sub_h; 
					float plus_w,plus_h;
					bool sym ;
					float pos_x, pos_y;

					if (!(col%2)){
						pos_x = d_w*col;
						pos_y = d_h*(row+1/2)+d_h/2;
						sym = false;
					} 

					if (col%2){
						pos_x = d_w*col;
						pos_y = d_h*(row+1/2)+d_h/2;
						sym = true;
					}

					float PX[] = {pos_x ,d_w/3 + pos_x, d_w/2 + pos_x, 2*d_w/3 + pos_x, d_w + pos_x};
					float PY[]= {pos_y, d_h/2+ pos_y, d_h+ pos_y};

					if (!sym){

						PY[0] = d_h + pos_y;
						PY[1] = d_h/2 + pos_y;
						PY[2] = 0 + pos_y;
					}

					plus_w = (row)/6*2*d_w;
					plus_h = (col)/4*d_h;

					if ((row%6) >= 3){
						plus_w = plus_w + w;
					} 

					if (col%4 >=2){
						plus_h = plus_h + h/2;
					}

					//Identificación del modo de la cuadricula
					if ((row%6 >= 3)^(col%4 >=2)){

					}else{

					}

					//Identificación de la mitad a la que pertenece
					if ((2*(row%3)+col%2)>=2){

					}else{

					}
				}*/

				void drawHexel(TFTCanvas canvas, int row, int col, uint16_t color){

					int d_w=canvas.sub_w ;
					int d_h=canvas.sub_h; 
					float plus_w,plus_h;
					bool inv, inv_x, inv_y;
					//Serial << " R: " << row << " C: " << col  << "\n";
					float w = d_w;
					float h = d_h;

					float x1 = 0;
					float x2 = 0;
					float y1 = 0;
					float y2 = 0;
					float x3 = 0;
					float y3 = 0;

					//Desplazamiento entre cuadriculas
					plus_w = (col)/6*d_w;
					plus_h = (row)/4*d_h;

					if ((col%6) >= 3){
						plus_w = plus_w + w;
					} 

					if (row%4 >=2){
						plus_h = plus_h + h/2;
					}

					//Identificación del modo de la cuadricula
					if ((col%6 >= 3)^(row%4 >=2)){
						inv = true;
						//Serial.println("Mode 2");
						if ((2*(col%3)+row%2) <= 2){
							row++;
						}else{
							row--;
						}
					}else{
						inv = false;
						//Serial.println("Mode 1");
					}

					//Identificación de la mitad a la que pertenece
					if ((2*(col%3)+row%2)>=2){
						inv_x = true;
					}else{
						inv_x = false;
					}

					      //Identificacion del triangulo al que pertenece
					switch ((col%3+2*(row%2))%4){

						//Case for parameters [0,0] & [2,1]
						case 0:
							x2 = w/2;
							y2 = h/4;
							x3 = 2*w/3;
							y3 = 0;

						break;

						//Case for parameters [1,0] 
						case 1:
							x2 = w/3;
							y2 = h/2;
							x3 = w/2;
							y3 = h/4;
						break;

						//Case for parameters [0,1] & [2,0] 
						case 2:
							x2 = 0;
							y2 = h/2;
							x3 = w/3;
							y3 = h/2;
						break;

						//Case for parameters [1,1] 
						case 3:
							x1 = w;
							y1 = h/2;
							x2 = 2*w/3;
							y2 = 0;
							x3 = w/2;
							y3 = h/4;

						break;
					}

					if(inv){
						y1 = h/2 - y1;
						y2 = h/2 - y2;
						y3 = h/2 - y3;
					}  

					if(inv_x){
						x1 = w-x1;
						x2 = w-x2;
						x3 = w-x3;
						y1 = h/2-y1;
						y2 = h/2-y2;
						y3 = h/2-y3;
					}
					

					if((canvas.bckg_color-color) != canvas.bckg_color){
						//Draw Hexel Triangle
						canvas.fillTriangle( canvas.offset_x +  x1 + plus_w, canvas.offset_y + y1 + plus_h, canvas.offset_x + x2 + plus_w, canvas.offset_x + y2 + plus_h, canvas.offset_x + x3 + plus_w, canvas.offset_x + y3 + plus_h, color);
						
						/*#if (debug)
							Serial << "Col, Row: " << col%3 << " : " << row%2 << " P1: " << x1 << " , " << y1 << "P2: " << x2 << " , " << y2 << "P3: " << x3 << " , " << y3 << "\n";
							Serial << "d: " << 2*(col%3)+row%2 << " d2: " << (col%3+2*(row%2))%4 << "\n";
						#endif*/
					}
				};

				void drawHexels(TFTCanvas canvas){

					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){
						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							//Desplazamiento en cuadriculas
							//TFTMatrix::drawHexel(canvas, j + (TFTMatrix::X)*2 , i + (TFTMatrix::Y)*6,TFTMatrix::m(i,j));
							TFTMatrix::drawHexel(canvas, j + (TFTMatrix::X) , i + (TFTMatrix::Y),TFTMatrix::m(i,j));
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
						DUMPS("Out of matrix bounds in Rows");	
					}

					if (col + rangeCol > TFTMatrix::m.Cols() ){
						maxCol = TFTMatrix::m.Cols();
						DUMPS("Out of matrix bounds in Columns");
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
						DUMPS("Out of matrix bounds in Rows");	
					}

					if (col + rangeCol > TFTMatrix::m.Cols() ){
						maxCol = TFTMatrix::m.Cols();
						DUMPS("Out of matrix bounds in Columns");
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


	/*--------------------------------------------------------------------------------------------------------------------*/

	/*--------------------------------------------------------------------------------------------------------------------*/
						/*---------------------------------------------------------------------------------*/
						/*-------------------- TFTCanvas Methods with TFTMatrix Class ---------------------*/
						/*---------------------------------------------------------------------------------*/
	/*--------------------------------------------------------------------------------------------------------------------*/

	/*--------------------------------------------------------------------------------------------------------------------*/

	//

		#ifdef __SD_H__
			
			void TFTCanvas::drawCSV(char *fileName, int x, int y){
				File CSVFile;
				String data;
				if ((CSVFile = SD.open(fileName)) == NULL) {
					DUMPS("File Not Found" ); DUMPPRINTLN();
					return;
				}
				DUMP("File Size: ", CSVFile.size() );

				if ( CSVFile.available()) {
					
					//Reading first 4 lines format
					data = TFTCanvas::readln(CSVFile);
					data = TFTCanvas::readln(CSVFile);

					//Read Rows and columns of Hexel Grid
					data = TFTCanvas::readln(CSVFile);
					
					int nrows = readNumber( data )/3;
					int ncols = readNumber( data )/3;
					DUMP("ROWS: ",nrows);
					DUMP("COLS: ",ncols);
					DUMPPRINTLN();
					TFTCanvas::canvas(nrows,ncols,true);

					data = TFTCanvas::readln(CSVFile);
					
					while ( CSVFile.available())  {

						// Parser Content
						data = TFTCanvas::readln( CSVFile );

						int row = readNumber(data);
						int column = readNumber(data);
						int R = readNumber(data);
						int G = readNumber(data);
						int B = readNumber(data);
						int A = readNumber(data);

						uint16_t color = ( R <<16)|( G <<8)| B;

						TFTMatrix <1,1, uint16_t > hexel ;
						//hexel.m(0,0) = TFTCanvas::Color565(R,G,B);
						//hexel.setOrigin(row, col);

						hexel.drawHexel(*this,  column, row, TFTCanvas::Color565(R,G,B));
						//hexel.drawHexel(*this, row, column, color);
					} 

					CSVFile.close();
					
				}else {
					// if the file isn't open, pop up an error:
					
					DUMP("Error opening CSV File -->  ",TFTCanvas::_fileRoot);
					
				}

			}
		#endif


	

#endif
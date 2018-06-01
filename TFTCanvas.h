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

		#define BLACK 	0x000001
		#define BLUE 	0x0000FF
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define WHITE 	0xFFFFFF
		#define YELLOW 	0x00FF00
		#define ROSE 	0x99AA66
		#define WHITE 	0xFFFFFF

	#endif

	#if defined(_PDQST7735H_)

		#ifndef FOV
			#define FOV 	64
		#endif
		
		
		//-----------------LUT--------------//

		int HALFW, HALFH;
		Matrix<4, 4, float> m_world; 
		//static unsigned char i;
		static int loops;
		
		//--------------Event for changing Draw Type-----------------------//
		static double next_tick;
		static double last_btn;                      // used for checking when the button was last pushed
		//static unsigned char draw_type = 1;          // 0 - vertex | 1 - wireframe | 2 - flat colors | ...

		#ifdef MESH_H
			#define NODE(a, b) (long)(pgm_read_dword(&nodes[a][b]))
			#define EDGE(a, b) pgm_read_byte(&faces[a][b])

			#ifndef SKIP_TICKS
				#define SKIP_TICKS 	20
			#endif
			#ifndef MAX_FRAMESKIP
				#define MAX_FRAMESKIP 	5
			#endif

			#define NODEFLOAT(a, b) (float)(pgm_read_float(&nodes[a][b]))
			#define EDGEFLOAT(a, b) pgm_read_byte(&faces[a][b])

			static int proj_nodes[NODECOUNT][2];         // projected nodes (x,y)
			static int old_nodes[NODECOUNT][2];          // projected nodes of previous frame to check if we need to redraw
			
		#else
			#ifndef NODECOUNT
				#define NODECOUNT 40
			#endif

			#ifndef TRICOUNT
				#define TRICOUNT 40
			#endif
		#endif

		#define TFTLib PDQ_ST7735

		enum{
			ST7735_INITB			= 0,				// 1.8" (128x160) ST7735B chipset (only one type)
			ST7735_INITR_GREENTAB		= 1,				// 1.8" (128x160) ST7735R chipset with green tab (same as ST7735_INITR_18GREENTAB)
			ST7735_INITR_REDTAB		= 2,				// 1.8" (128x160) ST7735R chipset with red tab (same as ST7735_INITR_18REDTAB)
			ST7735_INITR_BLACKTAB		= 3,				// 1.8" (128x160) ST7735S chipset with black tab (same as ST7735_INITR_18BLACKTAB)
			ST7735_INITR_144GREENTAB	= 4,				// 1.4" (128x128) ST7735R chipset with green tab
			ST7735_INITR_18GREENTAB		= ST7735_INITR_GREENTAB,	// 1.8" (128x160) ST7735R chipset with green tab
			ST7735_INITR_18REDTAB		= ST7735_INITR_REDTAB,		// 1.8" (128x160) ST7735R chipset with red tab
			ST7735_INITR_18BLACKTAB		= ST7735_INITR_BLACKTAB,	// 1.8" (128x160) ST7735S chipset with black tab
		};

		#ifndef ST7735_CHIPSET
			#define ST7735_CHIPSET	3
		#endif
			// <= Set ST7735 LCD chipset/variation here (from above list)
		#ifndef ST7735_CS_PIN
			#define ST7735_CS_PIN	7 	// <= /CS pin (chip-select, LOW to get attention of ST7735, HIGH and it ignores SPI bus)
		#endif
		#ifndef ST7735_DC_PIN
			#define ST7735_DC_PIN	6 	// <= DC pin (1=data or 0=command indicator line) also called RS
		#endif
		#ifndef ST7735_SAVE_SPCR
			#define ST7735_SAVE_SPCR	1
		#endif

		#define BLACK 	0x0001
		#define BLUE 	0x001F
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define WHITE 	0xFFFFFF
		#define YELLOW 	0xFFE0
		#define ROSE 	0x99AA66
		#define WHITE 	0xFFFFFF

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
		#define ROSE 	0xF81F
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
	#if (!defined (_ST7735H_) && !defined(_ILI9341_) && !defined(_SPFD5408_) && !defined (_PDQST7735H_))  || defined(_SPFD5408_)
	//#if  defined(_SPFD5408_)

		#define TFTLib Adafruit_TFTLCD

		//PinOut for Arduino UNO
		#define LCD_CS A3 // Chip Select goes to Analog 3
		#define LCD_CD A2 // Command/Data goes to Analog 2
		#define LCD_WR A1 // LCD Write goes to Analog 1
		#define LCD_RD A0 // LCD Read goes to Analog 0

		#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
		#define TFT_SD_CS 10
		
		#define BLACK   0x0000
		#define BLUE    0x001F
		#define RED     0xF800
		#define GREEN   0x07E0
		#define CYAN    0x07FF
		#define MAGENTA 0xF81F
		#define YELLOW  0xFFE0
		#define ROSE 	0xF81F
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
				uint16_t _rows, _cols, size_x, size_y, sub_w, sub_h;
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
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000 , bool rounded = true );
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label  = ' ', int dir = 1);
				void fillCircleSector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color);
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

		TFTCanvas::TFTCanvas(int CS,int DC,int WR,int RD, int RESET) : TFTLib ( CS, DC, WR, RD, RESET) {

		}

		void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps);
			uint16_t identifier = TFTCanvas::readID();
			DUMPPRINTLN();
			DUMP("TFTCanvas ID: ", identifier);
			TFTCanvas::bckg_color = bckg_color;
			TFTCanvas::begin(TFTCanvas::readID());
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
				uint16_t _rows, _cols, size_x, size_y, sub_w, sub_h;
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
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000, bool rounded = true);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 1 );
				uint16_t color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::Color565(r,g,b); };
				void fillCircleSector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color);

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

	/*-------------------------------------------------------------*/
	/*-------  Constructor Class for PDQ_ST7735 TFT Screen ------- */
	/*-------------------------------------------------------------*/
	
	#ifdef _PDQST7735H_
		#include <PDQ_GFX.h>
		#include <PDQ_ST7735.h>	
		#include "BasicLinearAlgebra.h"

		#ifdef _LUT_
			#define LUT(a) (float)(pgm_read_float(&lut[a]))// return value from LUT in PROGMEM

			const float lut[] PROGMEM = {         // 0 to 90 degrees fixed point COSINE look up table
				//16384, 16381, 16374, 16361, 16344, 16321, 16294, 16261, 16224, 16182, 16135, 16082, 16025, 15964, 15897, 15825, 15749, 15668, 15582, 15491, 15395, 15295, 15190, 15081, 14967, 14848, 14725, 14598, 14466, 14329, 14188, 14043, 13894, 13740, 13582, 13420, 13254, 13084, 12910, 12732, 12550, 12365, 12175, 11982, 11785, 11585, 11381, 11173, 10963, 10748, 10531, 10310, 10086, 9860, 9630, 9397, 9161, 8923, 8682, 8438, 8191, 7943, 7691, 7438, 7182, 6924, 6663, 6401, 6137, 5871, 5603, 5334, 5062, 4790, 4516, 4240, 3963, 3685, 3406, 3126, 2845, 2563, 2280, 1996, 1712, 1427, 1142, 857, 571, 285, 0
				1.0000,	0.9998,	0.9994,	0.9986,	0.9976,	0.9962,	0.9945,	0.9925,	0.9902,	0.9877,	0.9848,	0.9816,	0.9781,	0.9744,	0.9703,	0.9659,	0.9612,	0.9563,	0.9510,	0.9455,	0.9396,	0.9335,	0.9271,	0.9205,	0.9135,	0.9063,	0.8987,	0.8910,	0.8829,	0.8746,	0.8660,	0.8571,	0.8480,	0.8386,	0.8290,	0.8191,	0.8090,	0.7986,	0.7880,	0.7771,	0.7660,	0.7547,	0.7431,	0.7313,	0.7193,	0.7071,	0.6946,	0.6819,	0.6691,	0.6560,	0.6428,	0.6293,	0.6156,	0.6018,	0.5878,	0.5735,	0.5591,	0.5446,	0.5299,	0.5150,	0.4999,	0.4848,	0.4694,	0.4540,	0.4384,	0.4226,	0.4067,	0.3907,	0.3746,	0.3583,	0.3420,	0.3256,	0.3090,	0.2924,	0.2756,	0.2588,	0.2419,	0.2249,	0.2079,	0.1908,	0.1736,	0.1564,	0.1392,	0.1218,	0.1045,	0.0871,	0.0697,	0.0523,	0.0349,	0.0174,	0.0000
			};

			float sin_lut(unsigned int angle) {
				angle += 90;
				if (angle > 450) return LUT(0);
				if (angle > 360 && angle < 451) return -LUT(angle-360);
				if (angle > 270 && angle < 361) return -LUT(360-angle);
				if (angle > 180 && angle < 271) return  LUT(angle-180);
				return LUT(180-angle);
			}

			float cos_lut(unsigned int angle) {
				if (angle > 360) return LUT(0);
				if (angle > 270 && angle < 361) return  LUT(360-angle);
				if (angle > 180 && angle < 271) return -LUT(angle-180);
				if (angle > 90  && angle < 181) return -LUT(180-angle);
				return LUT(angle);
			}
		#endif

		class TFTCanvas : public TFTLib {
			public:
				uint16_t size_x, size_y, sub_w, sub_h;
 				uint16_t offset_x = 0;
 				uint16_t offset_y = 0;
 				bool mode = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				//---------------------------------------//
 				//----------Methods PDQ_TFTCANVAS--------//

				TFTCanvas();
				TFTCanvas(int CS,int DC,int RST = 10);
				void init(uint16_t tab, uint16_t bckg_color = 0xFFFFFF, int rotation = 1);
				
				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				void setHexelMode(bool hexelmode = true);

				#ifdef MESH_H
					void draw_mesh( int (*old_listnodes)[2], int (*proj_listnodes)[2] , uint16_t newcolor = BLACK, uint16_t oldcolor = WHITE,int draw_type = 1);
				#else
					void draw_mesh( int (*old_listnodes)[2], int (*proj_listnodes)[2] , uint16_t newcolor = BLACK, uint16_t oldcolor = WHITE,int draw_type = 1);
					void clear_dirty( int (*n)[2]);
					void draw_wireframe(int (*n)[2], uint16_t color);
					void draw_flat_color( int (*n)[2], uint16_t color);
					void draw_vertex( int (*n)[2], uint16_t color);
					bool is_hidden( int (*n)[2],  unsigned char index);
					int shoelace( int (*n)[2],  unsigned char index);

				#endif

				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000, bool rounded = true);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 1 );
				uint16_t color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::Color565(r,g,b); };
				void fillCircleSector(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color);
				void fillCircleXector(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color);
				void fillCircleSector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color);
				void fillCircleXector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color);
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

		//-----------------------------------------------------------------------------------//
		//------------------------------ Mesh Class -----------------------------------------//
		//-------------- Mesh Class with dependencies on TFTCanvas --------------------------//

		#ifdef MESH_H

		#else

			class mesh{
				public:
	 				int proj_nodes[NODECOUNT][2];
	 				int old_nodes[NODECOUNT][2];
					float nodes[NODECOUNT][3];
					unsigned char faces[TRICOUNT][3];
					unsigned char SKIP_TICKS = 20;
					unsigned char MAX_FRAMESKIP = 5;
					long next_tick;
					unsigned char last_btn;                      // used for events
					unsigned char draw_type = 1;          // 0 - vertex | 1 - wireframe | 2 - flat colors | ...

	 				mesh::mesh(){
	 					mesh::next_tick = millis();
	 				}

	 				mesh::mesh(float meshnodes[][3], unsigned char meshfaces[][3]){
	 					mesh::next_tick = millis();
	 					memcpy( mesh::nodes, meshnodes, sizeof(float)* NODECOUNT*3);
	 					memcpy( mesh::faces, meshfaces, sizeof(unsigned char)* TRICOUNT*3);
	 				}

	 				void setnodes( float meshnodes[][3] ){
	 					memcpy( mesh::nodes, meshnodes, sizeof(float)* NODECOUNT*3);
	 				}

	 				void setfaces( unsigned char meshfaces[][3] ){
	 					memcpy( mesh::faces, meshfaces, sizeof(unsigned char)* TRICOUNT*3);
	 				}

	 				void setdraw_type( uint8_t drawtype ){
	 					mesh::draw_type = drawtype;
	 				}

	 				void setskip_tick( unsigned char skip_tick ){
	 					mesh::SKIP_TICKS = skip_tick;
	 				}

	 				void setframe_skip( unsigned char frame_skip ){
	 					mesh::MAX_FRAMESKIP = frame_skip;
	 				}
	 				//---------------------------------------------------------------------------------------------//
					//------------------------------ Update Mesh Functions ----------------------------------------//
				
	 				int shoelace( const unsigned char index, boolean projnodes = true );
	 				bool is_hidden( const unsigned char index, boolean projnodes = true );
	 				void draw_vertex(TFTCanvas *canvas, const uint16_t color, boolean projnodes = true);
	 				void draw_wireframe(TFTCanvas *canvas, const uint16_t color, boolean projnodes = true);
	 				void draw_flat_color( TFTCanvas *canvas, uint16_t color, boolean projnodes = true);
	 				void clear_dirty(TFTCanvas *canvas , uint16_t color = WHITE , boolean projnodes = true);
	 				void update( Matrix<4, 4, float> *f() );
	 				void update( int rotx, int roty, int rotz );
	 				void draw(TFTCanvas *canvas, uint16_t timer = 0);
			};

		#endif

 	#endif	

	/*-------------------------------------------------------------*/
	/*-------Constructor Definition for PDQ ST7735 TFT Screen------*/
	/*-------------------------------------------------------------*/
	#ifdef _PDQ_ST7735H_
		TFTCanvas::TFTCanvas() : TFTLib( ) {

		}

		TFTCanvas::TFTCanvas(int CS,int DC,int RST) : TFTLib ( ) {

		}

		void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps);

			TFTCanvas::begin(); 
			TFTCanvas::bckg_color= bckg_color;
			TFTCanvas::fillScreen(TFTCanvas::bckg_color);
			TFTCanvas::setRotation(rotation);
			TFTCanvas::size_x = TFTCanvas::width();
			TFTCanvas::size_y = TFTCanvas::height();
			HALFW = TFTCanvas::size_x/2;
			HALFH = TFTCanvas::size_y/2;
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
				uint16_t _rows, _cols, size_x, size_y, sub_w, sub_h;
 				uint16_t offset_x = 0;
 				uint16_t offset_y = 0;
 				bool mode = 0;
 				uint16_t bckg_color = 0xFFFFFF;

				TFTCanvas();
				TFTCanvas(int CS,int DC);
				TFTCanvas(int CS,int DC, int RST);
				void init(uint16_t bckg_color = 0xFFFFFF, int rotation = 0);
				void init(uint16_t tab, uint16_t bckg_color = 0xFFFFFF, int rotation = 0);

				void setSize(int sizeX, int sizeY);
				void setOffset(int offsetX, int offsetY);
				void setColor(uint16_t bckg_color);
				void setHexelMode(bool hexelmode = true);

				//Canvas define the size of the grid where we´ll paint
				void canvas(int8_t col,int8_t row, bool displayGrid = false, uint16_t color = 0x000000, bool rounded = true);
				void clean();
				void refresh();
				void subhexel(float pos_x, float pos_y, int w , int h, bool sym, uint16_t color);
				void trplot(Matrix<3, 3, float> T, int size = 30, uint16_t color = 0x000000, char label= ' ', int dir = 1 );
				uint16_t Color565(uint8_t r, uint8_t g, uint8_t b){return TFTCanvas::color565(r,g,b); };
				void fillCircleSector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color);

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

		TFTCanvas::TFTCanvas(int CS,int DC, int RST) : TFTLib( CS, DC, RST) {

		}

		void TFTCanvas::init( uint16_t bckg_color, int rotation){
			DUMP("TFTCanvas BPS: ", TFT_bps)

			TFTCanvas::bckg_color= bckg_color;
			TFTCanvas::begin();
			TFTCanvas::fillScreen(TFTCanvas::bckg_color);
			TFTCanvas::setRotation(rotation);
			TFTCanvas::size_x = TFTCanvas::width();
			TFTCanvas::size_y = TFTCanvas::height();
		}

		void TFTCanvas::init(uint16_t tab, uint16_t bckg_color, int rotation){
			TFTCanvas::init(bckg_color, rotation);
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

	void TFTCanvas::canvas(int8_t col,int8_t row, bool displayGrid, uint16_t color, bool rounded ){
		int gridX;
		int gridY;
		_rows = row;
		_cols = col;

		if(TFTCanvas::mode){
			TFTCanvas::sub_w = round(TFTCanvas::size_x /col);
			TFTCanvas::sub_h = TFTCanvas::sub_w*sqrt(4/3);
			gridX = TFTCanvas::sub_h;
			gridY = TFTCanvas::sub_w;
			row = TFTCanvas::size_y/ TFTCanvas::sub_h;
		}else{

			TFTCanvas::sub_w = round( TFTCanvas::width() /col );
			TFTCanvas::sub_h = round( TFTCanvas::height() /row );
			if (rounded){
				col = ceil( (float) TFTCanvas::width()/TFTCanvas::sub_w );
				row = ceil( (float) TFTCanvas::height()/TFTCanvas::sub_h );
			}
			gridX = TFTCanvas::sub_w;
			gridY = TFTCanvas::sub_h;
		}

		DUMP("Width: ", TFTCanvas::width() );
		DUMP(" Height: ", TFTCanvas::height() );
		DUMP("\t TFTsize_x: ", TFTCanvas::size_x );
		DUMP(" TFTsize_y: ", TFTCanvas::size_y );
		DUMP("\tWidth per column: ", TFTCanvas::sub_w );
		DUMP(" Height per row: ", TFTCanvas::sub_h);
		DUMP("\tColumns: ", col );
		DUMP(" Row: ", row );
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

				for(int i = 0; i < row; ++i){
					TFTCanvas::drawFastHLine( 0, TFTCanvas::offset_y+i*TFTCanvas::sub_h, TFTCanvas::width(), color); 
				}
				for(int i = 0; i < col; ++i){
					TFTCanvas::drawFastVLine( TFTCanvas::offset_x+i*TFTCanvas::sub_w, 0, TFTCanvas::height(), color); 
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

	void TFTCanvas::fillCircleSector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color){
			int16_t f	= 1 - r;
			int16_t ddF_x	= 1;
			int16_t ddF_y	= -2 * r;
			int16_t x	= 0;
			int16_t y	= r;

			while (x < y)
			{
				if (f >= 0)
				{
					y--;
					ddF_y += 2;
					f += ddF_y;
				}
				x++;
				ddF_x += 2;
				f += ddF_x;
			if (cornername & 0x4)
			{
				TFTCanvas::drawFastVLine(x0+x-1, y0, y, color);
				TFTCanvas::drawFastVLine(x0+y-1, y0, x, color);
			}
			if (cornername & 0x2)
			{
				TFTCanvas::drawFastVLine(x0+x-1, y0 - y, y, color);
				TFTCanvas::drawFastVLine(x0+y-1, y0 - x, x, color);
			}
			if (cornername & 0x8)
			{
				TFTCanvas::drawFastVLine(x0-y, y0, x, color);
				TFTCanvas::drawFastVLine(x0-x, y0, y, color);
			}
			if (cornername & 0x1)
			{
				TFTCanvas::drawFastVLine(x0-x, y0 - y, y, color);
				TFTCanvas::drawFastVLine(x0-y, y0 - x, x, color);
			}
		}
	}

	/*void TFTCanvas::fillCircleXector(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t color){
			int16_t f	= 1 - r;
			int16_t ddF_x	= 1;
			int16_t ddF_y	= -2 * r;
			int16_t x	= 0;
			int16_t y	= r;

			while (x <= y + 1)
			{
				if (f >= 0)
				{
					y--;
					ddF_y += 2;
					f += ddF_y;
				}
				x++;
				ddF_x += 2;
				f += ddF_x;
			if (cornername & 0x4)
			{
				TFTCanvas::drawFastVLine(x0+x-1, y0 + y, r-y , color);
				TFTCanvas::drawFastVLine(x0+y-1, y0 + x, r-x, color);
			}
			if (cornername & 0x2)
			{
				TFTCanvas::drawFastVLine(x0+x-1, y0 - r , r-y , color);
				TFTCanvas::drawFastVLine(x0+y-1, y0 - r , r-x-1, color);
			}
			if (cornername & 0x8)
			{
				TFTCanvas::drawFastVLine(x0-x, y0 + y , r-y, color);
				TFTCanvas::drawFastVLine(x0-y, y0 + x , r-x, color);
			}
			if (cornername & 0x1)
			{
				TFTCanvas::drawFastVLine(x0-x, y0 - r , r-y, color);
				TFTCanvas::drawFastVLine(x0-y, y0 - r , r-x-1, color);
			}
		}
	}*/

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
				Matrix<rows, cols, uint16_t, MemT> old_m;
				int X = 0;
				int Y = 0;
				int oldX = 0;
				int oldY = 0;
				//CG (Center of gravity)
				int CG_x=0;
				int CG_y=0;
				//Subpixels Size
				int w = 1;
				int h = 1;
				bool bounce = false;
				uint8_t bounce_xmax = 0, bounce_ymax = 0, bounce_xmin = 0, bounce_ymin = 0 ;
				int8_t dir_x = 1;
				int8_t dir_y = -1;

				TFTMatrix(){
					
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
					TFTMatrix::m.Fill(color);
				};

				void setBounce(uint8_t XMax, uint8_t YMax, uint8_t XMin = 0, uint8_t YMin = 0){
					bounce_xmax = XMax;
					bounce_ymax = YMax;
					bounce_xmin = XMin;
					bounce_ymin = YMin;
				}

				void changedir( uint8_t XMax, uint8_t YMax ){
					uint8_t XMin = 0;
					uint8_t YMin = 0;
					if (bounce_xmax != 0) XMax = bounce_xmax;
					if (bounce_ymax != 0) YMax = bounce_ymax;
					if (bounce_xmin != 0) XMin = bounce_xmin;
					if (bounce_ymin != 0) YMin = bounce_ymin;

					if( TFTMatrix::X + cols >= XMax ){
						Serial.println( TFTMatrix::X + cols );
						dir_x = -1;
					}
					if( TFTMatrix::X <= XMin ){
						dir_x = 1;
					}
					if( TFTMatrix::Y + rows >= YMax ){
						dir_y = 1;
					}
					if( TFTMatrix::Y <= YMin ){
						dir_y = -1;
					}
				}
				/*-----------------------Draw Functions on TFT Canvas-----------------------------*/
				/*Most important function for animation events */
				void update(TFTCanvas *canvas ){
					
					//No drawing for outer space.
					/*if ( TFTMatrix::Y + rows >= YMax ){

					}*/
					if( (TFTMatrix::oldX != TFTMatrix::X ) || ( TFTMatrix::oldY != TFTMatrix::Y) || memcmp( &old_m,  &m, sizeof(m) )){
						if (bounce){
							changedir(canvas->_cols, canvas->_rows );
						}
						//Movement detection
						if( (TFTMatrix::oldX != TFTMatrix::X ) || ( TFTMatrix::oldY != TFTMatrix::Y) ){
							//Clean old matrix 
							TFTMatrix::clean( canvas, true );
							TFTMatrix::oldX = TFTMatrix::X; TFTMatrix::oldY = TFTMatrix::Y;
						}

						//Color Change detection
						if (memcmp( &old_m,  &m, sizeof(m) ) ) {
							// render frame
							memcpy(&old_m, &m, sizeof(m));
						}
						TFTMatrix::draw(canvas);
					}
				}

				void draw(TFTCanvas *canvas){

					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){
						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							if((canvas->bckg_color-TFTMatrix::m(i,j)) != canvas->bckg_color){
								canvas->fillRect(canvas->offset_x+(TFTMatrix::X+j)*canvas->sub_w,canvas->offset_y+(TFTMatrix::Y+i)*canvas->sub_h,canvas->sub_w,canvas->sub_h,TFTMatrix::m(i,j));
							}
						}
					}
				};

				void move(TFTCanvas *canvas, int step_X,int step_Y){
					TFTMatrix::setOrigin( TFTMatrix::X+step_X*dir_x, TFTMatrix::Y-step_Y*dir_y);	
				};

				void clean(TFTCanvas *canvas , bool oldMatrix = true ){
					TFTMatrix::cleanSubMatrix(canvas,0,0,TFTMatrix::m.Rows(),TFTMatrix::m.Cols(), oldMatrix);
				};

				void cleanSubMatrix(TFTCanvas *canvas, int row, int col, int rangeRow, int rangeCol, bool oldMatrix = true){
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

					if (oldMatrix){
						for (int i=row; i< maxRow; i++){
							for (int j=col; j< maxCol; j++){
								if( ((canvas->bckg_color-TFTMatrix::old_m(i,j)) != canvas->bckg_color)){
									canvas->fillRect(canvas->offset_x+(TFTMatrix::oldX+j)*canvas->sub_w,canvas->offset_y+(TFTMatrix::oldY+i)*canvas->sub_h,canvas->sub_w,canvas->sub_h,canvas->bckg_color);
								}
							}
						}
					}else{
						for (int i=row; i< maxRow; i++){
							for (int j=col; j< maxCol; j++){
								if( ((canvas->bckg_color-TFTMatrix::m(i,j)) != canvas->bckg_color)){
									canvas->fillRect(canvas->offset_x+(TFTMatrix::X+j)*canvas->sub_w,canvas->offset_y+(TFTMatrix::Y+i)*canvas->sub_h,canvas->sub_w,canvas->sub_h,canvas->bckg_color);
								}
							}
						}
					}
				};

				void drawSubMatrix(TFTCanvas *canvas, int d_x, int d_y, int row, int col, int rangeRow, int rangeCol){
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
							if((canvas->bckg_color-TFTMatrix::m(i,j)) != canvas->bckg_color){
								//Negative sum for d_y if we want to dispose up direction like positive number
								canvas->fillRect(canvas->offset_x+(TFTMatrix::X+j+d_x)*canvas->sub_w,canvas->offset_y+(TFTMatrix::Y+i-d_y)*canvas->sub_h,canvas->sub_w,canvas->sub_h,TFTMatrix::m(i,j));
							}
						}
					}
				}

				void drawPixels(TFTCanvas *canvas){
					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){

						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							//Problem with dark color because is equal to zero. Everything initialize with 0.

							if( ((canvas->bckg_color-TFTMatrix::m(i,j)) != canvas->bckg_color)){

							//The orientation i is for Y-Axis and j is for X_axis 
								canvas->drawPixel(canvas->offset_x+TFTMatrix::CG_x+j,canvas->offset_y+TFTMatrix::CG_y+i,TFTMatrix::m(i,j)); 
								DUMP("Row: ",i);
								DUMP(" Column: ",j);
								DUMP(" Color: ",TFTMatrix::m(i,j));
								DUMP(" Canvas ColorDiff: ",canvas->bckg_color-TFTMatrix::m(i,j));
							}
						}
					}
				};

				/*void drawHexel(TFTCanvas *canvas, int row, int col, uint16_t color){
					int d_w=canvas->sub_w ;
					int d_h=canvas->sub_h; 
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

				void drawHexel(TFTCanvas *canvas, int row, int col, uint16_t color){

					int d_w=canvas->sub_w ;
					int d_h=canvas->sub_h; 
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
					

					if((canvas->bckg_color-color) != canvas->bckg_color){
						//Draw Hexel Triangle
						canvas->fillTriangle( canvas->offset_x +  x1 + plus_w, canvas->offset_y + y1 + plus_h, canvas->offset_x + x2 + plus_w, canvas->offset_x + y2 + plus_h, canvas->offset_x + x3 + plus_w, canvas->offset_x + y3 + plus_h, color);
						
						/*#if (debug)
							Serial << "Col, Row: " << col%3 << " : " << row%2 << " P1: " << x1 << " , " << y1 << "P2: " << x2 << " , " << y2 << "P3: " << x3 << " , " << y3 << "\n";
							Serial << "d: " << 2*(col%3)+row%2 << " d2: " << (col%3+2*(row%2))%4 << "\n";
						#endif*/
					}
				};

				void drawHexels(TFTCanvas *canvas){

					for(int i = 0; i < TFTMatrix::m.Rows(); ++i){
						for(int j = 0; j < TFTMatrix::m.Cols(); ++j){
							//Desplazamiento en cuadriculas
							//TFTMatrix::drawHexel(canvas, j + (TFTMatrix::X)*2 , i + (TFTMatrix::Y)*6,TFTMatrix::m(i,j));
							TFTMatrix::drawHexel(canvas, j + (TFTMatrix::X) , i + (TFTMatrix::Y),TFTMatrix::m(i,j));
						}
					}
				};

				void movedraw(TFTCanvas *canvas, int step_X,int step_Y){
					step_X = step_X*dir_x;
					step_Y = step_Y*dir_y;
					bool d_x = (step_X) >= 0;
					bool d_y = (step_Y) >= 0;
					if ( (abs(step_X) >= TFTMatrix::m.Cols()) || (abs(step_Y) >= TFTMatrix::m.Rows()) ){
						
						//Borrar y dibujar completamente
						TFTMatrix::clean(canvas);
						TFTMatrix::draw(canvas);
						//TFTMatrix::setOrigin( TFTMatrix::X+step_X, TFTMatrix::Y-step_Y);
						
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
									if((canvas->bckg_color-TFTMatrix::m(i,j) ) == canvas->bckg_color ){
										canvas->fillRect(canvas->offset_x+(TFTMatrix::X+j+step_X)*canvas->sub_w,canvas->offset_y+(TFTMatrix::Y+i-step_Y)*canvas->sub_h,canvas->sub_w,canvas->sub_h,canvas->bckg_color);
									}else{
										canvas->fillRect(canvas->offset_x+(TFTMatrix::X+j+step_X)*canvas->sub_w,canvas->offset_y+(TFTMatrix::Y+i-step_Y)*canvas->sub_h,canvas->sub_w,canvas->sub_h,TFTMatrix::m(i,j));
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

		template<int rows, int cols , int nsprites, class ElemT = uint16_t, class MemT = Array<rows,cols,ElemT> >
		class TFTSprite : public TFTMatrix<rows,cols> {
			public:
				unsigned long timeline[nsprites];
				unsigned long current_time, time_offset, endtime;
				int currentsprite = 0;
				Matrix<rows,cols, uint16_t, MemT> sprites[nsprites];
				bool loop = false;
				bool rel = false;
				int add_counter = 0;

				TFTSprite(){

				};

				void setLoop(bool mode = true){TFTSprite::loop = mode;};
				void setTimeOffset(){TFTSprite::time_offset = millis();};
				void setTimeOffset(unsigned long t ){TFTSprite::time_offset = t;};
				void setEndTime(unsigned long t ){TFTSprite::endtime = t;};
				
				void addSprite ( Matrix<rows,cols> sprite , unsigned long timestamp ){
					DUMP(" Counter ", add_counter);
					DUMPPRINTLN();
					if (add_counter >= nsprites){
						DUMP("Overload adding sprites. Insert sprite defined with time ", timestamp );
						DUMPPRINTLN();
						TFTSprite::BubbleSortSprite( timestamp, sprite );
					}else{
						TFTSprite::setSprite( 0, sprite );
						TFTSprite::setSprite( 0, timestamp );
						TFTSprite::BubbleSortAsc(  timeline , nsprites );
					}
					
					add_counter++;
					
					for (int i = 0;  i< nsprites; i++){
						DUMP(" Sprite", i );
						DUMP(" ", timeline[i] );
						DUMPPRINTLN();

						/*for (int j = 0;  j< rows; j++){
							for (int k = 0;  k< cols; k++){
								DUMP(" ",sprites[i](j,k) );
							}
							DUMPPRINTLN();
						}*/

					}
					DUMPPRINTLN();
				};

				void addSprite ( void *f() , unsigned long timestamp );

				void setSprite ( int ID , Matrix<rows,cols> sprite ,  unsigned long timestamp ){
					TFTSprite::setSprite( ID, sprite);
					TFTSprite::setSprite( ID, timestamp);
				};

				void setSprite ( int ID , Matrix<rows,cols> sprite ){
					TFTSprite::sprites[ ID ] = sprite;
				};

				void setSprite ( int ID , unsigned long timestamp ){
					TFTSprite::timeline[ ID ] = timestamp;
					if (add_counter > nsprites){
						BubbleSortAsc( timeline , nsprites);
					}else{

					}
					/*for (int i = 0;  i< nsprites; i++){
						DUMP(" Sprite", i );
						DUMP(" ", timeline[i] );
						DUMPPRINTLN();
					}*/
				};
				
				void animate( TFTCanvas *canvas ){
					if( millis() - time_offset > timeline[ currentsprite ]){
				
						DUMP(" Time: ", millis() - time_offset );
						DUMP("\t TimeLine: ",timeline[ currentsprite ]);
						if (currentsprite < nsprites){
							
							DUMP(" Current: ",currentsprite);
							DUMPPRINTLN();
							current_time = timeline[currentsprite ];
							TFTSprite::m = TFTSprite::sprites[ currentsprite ] ;
							currentsprite++;
							TFTSprite::update(canvas);
							
						}else{
							if (TFTSprite::loop){
								for (int i = 0; i < nsprites; ++i){
									TFTSprite::timeline[ i ] += TFTSprite::timeline[ i ];
								}
								currentsprite = 0 ;
							}else{
								current_time = 4294000000UL;
							}
							
						}
					}
					
				};
				
			private:
				void BubbleSortAsc(unsigned long* values, int length){
					Matrix<rows,cols, uint16_t> sprite_temp;
					unsigned long tmp;
					for(int i=0; i<length; i++){
						for( int j=i+1; j<length; j++){
							if( *(values+i) > *(values+j)){
								tmp = *(values+i);
								*(values+i) = *(values+j);
								*(values+j) = tmp;

								memcpy(&sprite_temp, &sprites[i], sizeof( Matrix <rows, cols, uint16_t> ));
								memcpy(&sprites[i], &sprites[j], sizeof( Matrix <rows, cols, uint16_t> ));
								memcpy(&sprites[j], &sprite_temp, sizeof( Matrix <rows, cols, uint16_t> ));

							}
						}
					} 
				}

				void BubbleSortSprite( unsigned long time, Matrix<rows,cols> sprite ){
					
					if ( time > timeline[nsprites-1] ){
						DUMPS(" NO INSERT , TIME OVER LAST TIMESTAP");
					}else{
						for(int i=0; i<nsprites; i++){
							if (timeline[i] == time){
								timeline[i] = time;
								sprites[i] = sprite;
								break;
							}else if( timeline[i] > time ){
								
								movesprites (i);

								timeline[i] = time;
								sprites[i] = sprite;
								break;
							}
						} 
					}
				}

				void movesprites(int from){
					for( int j= nsprites-1; j > from ; j--){
						timeline[j] = timeline[j-1];
						memcpy(&sprites[j], &sprites[j-1], sizeof( Matrix <rows, cols, uint16_t> ));
					}
				}
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

						hexel.drawHexel(this,  column, row, TFTCanvas::Color565(R,G,B));
						//hexel.drawHexel(*this, row, column, color);
					} 

					CSVFile.close();
					
				}else {
					// if the file isn't open, pop up an error:
					
					DUMP("Error opening CSV File -->  ",TFTCanvas::_fileRoot);
					
				}

			}
		#endif


		#if defined(_PDQST7735H_)

			void TFTCanvas::fillCircleSector(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color){
					coord_t f	= 1 - r;
					coord_t ddF_x	= 1;
					coord_t ddF_y	= -2 * r;
					coord_t x	= 0;
					coord_t y	= r;

					while (x < y)
					{
						if (f >= 0)
						{
							y--;
							ddF_y += 2;
							f += ddF_y;
						}
						x++;
						ddF_x += 2;
						f += ddF_x;
					if (cornername & 0x4)
					{
						TFTCanvas::drawFastVLine(x0+x-1, y0, y, color);
						TFTCanvas::drawFastVLine(x0+y-1, y0, x, color);
					}
					if (cornername & 0x2)
					{
						TFTCanvas::drawFastVLine(x0+x-1, y0 - y, y, color);
						TFTCanvas::drawFastVLine(x0+y-1, y0 - x, x, color);
					}
					if (cornername & 0x8)
					{
						TFTCanvas::drawFastVLine(x0-y, y0, x, color);
						TFTCanvas::drawFastVLine(x0-x, y0, y, color);
					}
					if (cornername & 0x1)
					{
						TFTCanvas::drawFastVLine(x0-x, y0 - y, y, color);
						TFTCanvas::drawFastVLine(x0-y, y0 - x, x, color);
					}
				}
			}

			void TFTCanvas::fillCircleXector(coord_t x0, coord_t y0, coord_t r, uint8_t cornername, color_t color){
					coord_t f	= 1 - r;
					coord_t ddF_x	= 1;
					coord_t ddF_y	= -2 * r;
					coord_t x	= 0;
					coord_t y	= r;

					while (x <= y + 1)
					{
						if (f >= 0)
						{
							y--;
							ddF_y += 2;
							f += ddF_y;
						}
						x++;
						ddF_x += 2;
						f += ddF_x;
					if (cornername & 0x4)
					{
						TFTCanvas::drawFastVLine(x0+x-1, y0 + y, r-y , color);
						TFTCanvas::drawFastVLine(x0+y-1, y0 + x, r-x, color);
					}
					if (cornername & 0x2)
					{
						TFTCanvas::drawFastVLine(x0+x-1, y0 - r , r-y , color);
						TFTCanvas::drawFastVLine(x0+y-1, y0 - r , r-x-1, color);
					}
					if (cornername & 0x8)
					{
						TFTCanvas::drawFastVLine(x0-x, y0 + y , r-y, color);
						TFTCanvas::drawFastVLine(x0-y, y0 + x , r-x, color);
					}
					if (cornername & 0x1)
					{
						TFTCanvas::drawFastVLine(x0-x, y0 - r , r-y, color);
						TFTCanvas::drawFastVLine(x0-y, y0 - r , r-x-1, color);
					}
				}
			}

			#ifndef Robo3Duino_h

				#ifdef _LUT_
					const float ang2rad =180/PI;
					Matrix<4,4, float> trotx( float angle ){
						float arrayRot[4][4] = {{1,0,0,0},{0,cos_lut(angle),-sin_lut(angle),0},{0,sin_lut(angle),cos_lut(angle),0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> troty( float angle ){	
						float arrayRot[4][4] = {{cos_lut(angle),0,sin_lut(angle),0},{0,1,0,0},{-sin_lut(angle),0,cos_lut(angle),0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> trotz( float angle ){
						float arrayRot[4][4] = {{cos_lut(angle),-sin_lut(angle),0,0},{sin_lut(angle),cos_lut(angle),0},{0,0,1,0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> transl( float px, float py, float pz ){
						float arrayRot[4][4] = { {0,0,0,px}, {0,0,0,py}, {0,0,0,pz}, {0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}
				#else

					const float ang2rad =180/PI;
					Matrix<4,4, float> trotx( float angle ){
						float arrayRot[4][4] = {{1,0,0,0},{0,cos(angle/ang2rad),-sin(angle/ang2rad),0},{0,sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> troty( float angle ){	
						float arrayRot[4][4] = {{cos(angle/ang2rad),0,sin(angle/ang2rad),0},{0,1,0,0},{-sin(angle/ang2rad),0,cos(angle/ang2rad),0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> trotz( float angle ){
						float arrayRot[4][4] = {{cos(angle/ang2rad),-sin(angle/ang2rad),0,0},{sin(angle/ang2rad),cos(angle/ang2rad),0},{0,0,1,0},{0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}

					Matrix<4,4, float> transl( float px, float py, float pz ){
						float arrayRot[4][4] = { {0,0,0,px}, {0,0,0,py}, {0,0,0,pz}, {0,0,0,1}};
						Matrix <4, 4, float> m (arrayRot);
						return m;
					}
				#endif
			#endif

			#ifdef MESH_H

				int shoelace( const int (*n)[2], const unsigned char index){
					unsigned char t = 0;
					int surface = 0;

					for (; t<3; t++) {
					// (x1y2 - y1x2) + (x2y3 - y2x3) ...
					surface += (n[EDGE(index,t)][0]           * n[EDGE(index,(t<2?t+1:0))][1]) -
						(n[EDGE(index,(t<2?t+1:0))][0] * n[EDGE(index,t)][1]);
					}
					return surface * 0.5;
				};

				bool is_hidden(const int (*n)[2], const unsigned char index){
					return ( ( (n[EDGE(index,0)][0] * n[EDGE(index,1)][1]) -
						(n[EDGE(index,1)][0] * n[EDGE(index,0)][1])   ) +
						( (n[EDGE(index,1)][0] * n[EDGE(index,2)][1]) -
						(n[EDGE(index,2)][0] * n[EDGE(index,1)][1])   ) +
						( (n[EDGE(index,2)][0] * n[EDGE(index,0)][1]) -
						(n[EDGE(index,0)][0] * n[EDGE(index,2)][1])   ) ) < 0 ? false : true;
				};

				void draw_vertex(TFTCanvas *canvas, const int (*n)[2], const uint16_t color){
					int i = NODECOUNT-1;
					do {
						canvas->drawPixel(n[i][0],n[i][1], color);
					} while(i--);
				};

				void draw_wireframe(TFTCanvas *canvas, const int (*n)[2], const uint16_t color){
					int i = TRICOUNT-1;
					do {
						// don't draw triangle with negative surface value
						if (!is_hidden(n, i)) {
							// draw triangle edges - 0 -> 1 -> 2 -> 0
							canvas->drawLine(n[EDGE(i,0)][0], n[EDGE(i,0)][1], n[EDGE(i,1)][0], n[EDGE(i,1)][1], color);
							canvas->drawLine(n[EDGE(i,1)][0], n[EDGE(i,1)][1], n[EDGE(i,2)][0], n[EDGE(i,2)][1], color);
							canvas->drawLine(n[EDGE(i,2)][0], n[EDGE(i,2)][1], n[EDGE(i,0)][0], n[EDGE(i,0)][1], color);
						}
					} while(i--);
				};

				void draw_flat_color( TFTCanvas *canvas, const int (*n)[2], uint16_t color){
					int i = TRICOUNT-1;
					int surface;
					uint16_t col = color;
					do {
						// draw only triangles facing us
						if ((surface=shoelace(n, i)) < 0) {
							// this is an ugly hack but it 'somehow' fakes shading
							// depending on the size of the surface of the triangle
							// change the color toward brighter/darker
							color = col * (surface * 0.001);

							canvas->fillTriangle(n[EDGE(i,0)][0], n[EDGE(i,0)][1],
								n[EDGE(i,1)][0], n[EDGE(i,1)][1],
								n[EDGE(i,2)][0], n[EDGE(i,2)][1],
								color);
						}
					} while(i--);
				};

				//-----------------------------------------------------------------------------------//
				//------------------------------ Update Mesh ----------------------------------------//
				//--- paramater f is a function for scale,rotation, translation management of mesh---//

				void update_mesh( void *f()  ){
					loops = 0;
					while( millis() > next_tick && loops < MAX_FRAMESKIP) {
						f();

						for (int i=0; i<NODECOUNT; i++) {
							/*DUMP (" X ", NODEFLOAT(i,0) );
							DUMP (" Y ", NODEFLOAT(i,1) );
							DUMP (" Z ", NODEFLOAT(i,2) );
							DUMPPRINTLN();*/
							float arrayNODES[4][1] = {NODEFLOAT(i,0),NODEFLOAT(i,1),NODEFLOAT(i,2), 0};
							Matrix <4, 1, float> m_mesh (arrayNODES);

							Matrix <4, 1, float> res = m_world*m_mesh;

					 		long x =  res(0,0);
							long y =  res(1,0);
							long z =  res(2,0);
							
							// store projected node // Siempre me da el mismo valor ¿?¿?¿?
							proj_nodes[i][0] = (FOV * x) / (FOV + z) + HALFW;
							proj_nodes[i][1] = (FOV * y) / (FOV + z) + HALFH;

							/*DUMP("Node: ", i);
							DUMP (" dX ", x);DUMP (" dY ", y); DUMP (" dZ ", z);
							DUMPPRINTLN();
							DUMP (" X ", ((FOV * x) / (FOV + z) + HALFW));
							DUMP (" Y ", ((FOV * y) / (FOV + z) + HALFH));
							DUMPPRINTLN();*/
						}
						next_tick += SKIP_TICKS;
						loops++;
					}
				}

				void TFTCanvas::draw_mesh( int (*old_listnodes)[2], int (*proj_listnodes)[2], uint16_t newcolor, uint16_t oldcolor, int draw_type  ){
					
					if (memcmp(old_listnodes, proj_listnodes, sizeof(proj_listnodes))) {
						// render frame
						//int draw_type = 0;
						switch(draw_type) {
						  case 0: 
						  	//draw_vertex( this, *old_listnodes, oldcolor);
							//draw_vertex( this, proj_listnodes, newcolor);
							break;
						  case 1: if (TRICOUNT > 32) {
						  		//clear_dirty( this, old_listnodes);
						  	}
							else {
								//draw_wireframe( this, old_listnodes, oldcolor);
								//draw_wireframe( this, proj_listnodes, newcolor);
							}
							break;
						  case 2: //clear_dirty( this, old_listnodes);
							//draw_flat_color( this, proj_listnodes, newcolor);
							break;
						  case 3: //draw_flat_color( this, proj_nodes, newcolor);
							//draw_wireframe( this, proj_listnodes, oldcolor);
							break;
						  case 4: //draw_flat_color( this, proj_listnodes, newcolor);
							break;
						}
						// copy projected nodes to old_nodes to check if we need to redraw next frame
						for (int i=0; i<NODECOUNT; i++) {
							DUMP (" proj node X ", proj_listnodes[i][0] );
							DUMP (" proj node Y ", proj_listnodes[i][1] );
							DUMP (" proj node Z ", proj_listnodes[i][2] );
							DUMPPRINTLN();
			            }
			            for (int i=0; i<NODECOUNT; i++) {
							DUMP (" old node X ", old_listnodes[i][0] );
							DUMP (" old node Y ", old_listnodes[i][1] );
							DUMP (" old node Z ", old_listnodes[i][2] );
							DUMPPRINTLN();
			            }
						//memcpy(&old_listnodes, &proj_listnodes, sizeof(proj_listnodes));
						memcpy(&old_listnodes, &proj_listnodes, sizeof(float)*NODECOUNT*2);
						for (int i=0; i<NODECOUNT; i++) {
						  DUMP (" old node post X ", old_listnodes[i][0] );
						  DUMP (" old node post Y ", old_listnodes[i][1] );
						  DUMP (" old node post Z ", old_listnodes[i][2] );
						  DUMPPRINTLN();
						}
						
					}
				}
			#else

				//-----------------------------------------------------------------------------------//
				//------------------------------ Mesh Functions ----------------------------------------//

				int shoelace( mesh *model, const unsigned char index){
					unsigned char t = 0;
					int surface = 0;
					
					for (; t<3; t++) {
					// (x1y2 - y1x2) + (x2y3 - y2x3) ...
					surface += (  model->nodes[ model->faces[index][t] ][0] * model->nodes[ model->faces[index][(t<2?t+1:0)] ][1] ) -
						( model->nodes[ model->faces[index][(t<2?t+1:0)] ][0] * model->nodes[ model->faces[index][t] ][1] );
					}
					return surface * 0.5;
				};

				bool is_hidden(mesh *model, const unsigned char index){
					return ( ( (model->nodes[ model->faces[index][0] ][0] * model->nodes[ model->faces[index][1] ][1]) -
						(model->nodes[model->faces[index][1] ][0] * model->nodes[model->faces[index][0] ][1])   ) +
						( (model->nodes[model->faces[index][1] ][0] * model->nodes[model->faces[index][2] ][1]) -
						(model->nodes[model->faces[index][2] ][0] * model->nodes[model->faces[index][1] ][1])   ) +
						( (model->nodes[model->faces[index][2] ][0] * model->nodes[model->faces[index][0] ][1]) -
						(model->nodes[model->faces[index][0] ][0] * model->nodes[model->faces[index][2] ][1])   ) ) < 0 ? false : true;
				};

				void draw_vertex(TFTCanvas *canvas, const int (*n)[2], const uint16_t color){
					int i = NODECOUNT-1;
					  do {
					    canvas->drawPixel(n[i][0],n[i][1], color);
					  } while(i--);
				};

				void draw_wireframe(TFTCanvas *canvas,  mesh *model, const uint16_t color){
					int i = TRICOUNT-1;
					do {
						// don't draw triangle with negative surface value
						if (!is_hidden( model, i )) {
							// draw triangle edges - 0 -> 1 -> 2 -> 0
							canvas->drawLine(model->nodes[model->faces[i][0]][0], model->nodes[model->faces[i][0]][1], model->nodes[model->faces[i][1]][0], model->nodes[model->faces[i][1]][1], color);
							canvas->drawLine(model->nodes[model->faces[i][1]][0], model->nodes[model->faces[i][1]][1], model->nodes[model->faces[i][2]][0], model->nodes[model->faces[i][2]][1], color);
							canvas->drawLine(model->nodes[model->faces[i][2]][0], model->nodes[model->faces[i][2]][1], model->nodes[model->faces[i][0]][0], model->nodes[model->faces[i][0]][1], color);
						}
					} while(i--);
				};

				void draw_flat_color( TFTCanvas *canvas, mesh *model, uint16_t color){
					int i = TRICOUNT-1;
					int surface;
					uint16_t col = color;
					do {
						// draw only triangles facing us
						if ((surface=shoelace(model, i)) < 0) {
							// this is an ugly hack but it 'somehow' fakes shading
							// depending on the size of the surface of the triangle
							// change the color toward brighter/darker
							color = col * (surface * 0.001);

							canvas->fillTriangle(model->nodes[model->faces[i][0]][0], model->nodes[model->faces[i][0]][1],
								model->nodes[model->faces[i][1]][0], model->nodes[model->faces[i][1]][1],
								model->nodes[model->faces[i][2]][0], model->nodes[model->faces[i][2]][1],
								color);
						}
					} while(i--);
				};

				//------------------------------------------------------------------------------------------//
				//------------------------------ Class Mesh Methods ----------------------------------------//
				int mesh::shoelace( const unsigned char index, boolean projnodes ){
					unsigned char t = 0;
					int surface = 0;
					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}

					for (; t<3; t++) {
					// (x1y2 - y1x2) + (x2y3 - y2x3) ...
					surface += (  matrix_ptr[ mesh::faces[index][t] ][0] * matrix_ptr[ mesh::faces[index][(t<2?t+1:0)] ][1] ) -
						( matrix_ptr[ mesh::faces[index][(t<2?t+1:0)] ][0] * matrix_ptr[ mesh::faces[index][t] ][1] );
					}
					return surface * 0.5;
				};

				bool mesh::is_hidden( const unsigned char index, boolean projnodes ){
					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}

					return ( ( (matrix_ptr[ mesh::faces[index][0] ][0] * matrix_ptr[ mesh::faces[index][1] ][1]) -
						(matrix_ptr[mesh::faces[index][1] ][0] * matrix_ptr[mesh::faces[index][0] ][1])   ) +
						( (matrix_ptr[mesh::faces[index][1] ][0] * matrix_ptr[mesh::faces[index][2] ][1]) -
						(matrix_ptr[mesh::faces[index][2] ][0] * matrix_ptr[mesh::faces[index][1] ][1])   ) +
						( (matrix_ptr[mesh::faces[index][2] ][0] * matrix_ptr[mesh::faces[index][0] ][1]) -
						(matrix_ptr[mesh::faces[index][0] ][0] * matrix_ptr[mesh::faces[index][2] ][1])   ) ) < 0 ? false : true;
				};

				void mesh::draw_vertex(TFTCanvas *canvas, const uint16_t color, boolean projnodes ){
					int i = NODECOUNT-1;

					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}

					do {
						canvas->drawPixel( matrix_ptr[i][0], matrix_ptr[i][1], color);
					} while(i--);
					
				};

				void mesh::draw_wireframe(TFTCanvas *canvas, const uint16_t color, boolean projnodes ){
					int i = TRICOUNT-1;
					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}

					do {
						// don't draw triangle with negative surface value
						if (!mesh::is_hidden( i , projnodes)) {
							// draw triangle edges - 0 -> 1 -> 2 -> 0
							canvas->drawLine(matrix_ptr[mesh::faces[i][0]][0], matrix_ptr[mesh::faces[i][0]][1], matrix_ptr[mesh::faces[i][1]][0], matrix_ptr[mesh::faces[i][1]][1], color);
							canvas->drawLine(matrix_ptr[mesh::faces[i][1]][0], matrix_ptr[mesh::faces[i][1]][1], matrix_ptr[mesh::faces[i][2]][0], matrix_ptr[mesh::faces[i][2]][1], color);
							canvas->drawLine(matrix_ptr[mesh::faces[i][2]][0], matrix_ptr[mesh::faces[i][2]][1], matrix_ptr[mesh::faces[i][0]][0], matrix_ptr[mesh::faces[i][0]][1], color);
						}
					} while(i--);

				};

				void mesh::draw_flat_color( TFTCanvas *canvas, uint16_t color, boolean projnodes ){
					int i = TRICOUNT-1;
					int surface;
					uint16_t col = color;

					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}

					do {
						// draw only triangles facing us
						if ((surface=mesh::shoelace( i, projnodes )) < 0) {
							// this is an ugly hack but it 'somehow' fakes shading
							// depending on the size of the surface of the triangle
							// change the color toward brighter/darker
							color = col * (surface * 0.001);

							canvas->fillTriangle(matrix_ptr[mesh::faces[i][0]][0], matrix_ptr[mesh::faces[i][0]][1],
								matrix_ptr[mesh::faces[i][1]][0], matrix_ptr[mesh::faces[i][1]][1],
								matrix_ptr[mesh::faces[i][2]][0], matrix_ptr[mesh::faces[i][2]][1],
								color);
						}
					} while(i--);

				};

				void mesh::clear_dirty(TFTCanvas *canvas, uint16_t color = WHITE , boolean projnodes ){
					unsigned char x0=canvas->width(), y0=canvas->height(), x1=0, y1=0, c, w, h;
					
					int (*matrix_ptr)[2];
					if (projnodes){
						matrix_ptr = mesh::proj_nodes;
					}else{
						matrix_ptr = mesh::old_nodes;
					}
					// get bounding box of mesh

					for (c=0; c<NODECOUNT; c++) {
						if (matrix_ptr[c][0] < x0) x0 = matrix_ptr[c][0];
						if (matrix_ptr[c][0] > x1) x1 = matrix_ptr[c][0];
						if (matrix_ptr[c][1] < y0) y0 = matrix_ptr[c][1];
						if (matrix_ptr[c][1] > y1) y1 = matrix_ptr[c][1];
					}

					// clear area
					canvas->spi_begin();
					canvas->setAddrWindow_(x0, y0, x1, y1);
					h = (y1-y0);
					w = (x1-x0)+1;
					do {
						canvas->spiWrite16(color, w);
					} while (h--);
					canvas->spi_end();
				};
				
				void mesh::update( Matrix<4, 4, float> *f() ){
					loops = 0;
					while( millis() > next_tick && loops < mesh::MAX_FRAMESKIP) {
						f();
						//Matrix <4, 4, float> *m_rot ( f() );

						for (int i=0; i<NODECOUNT; i++) {
				
							float arrayNODES[4][1] = {mesh::nodes[i][0] ,mesh::nodes[i][1],mesh::nodes[i][2], 0};
							Matrix <4, 1, float> m_mesh (arrayNODES);

							Matrix <4, 1, float> res = m_world*m_mesh;

					 		long x =  res(0,0);
							long y =  res(1,0);
							long z =  res(2,0);
							
							// store projected node // Siempre me da el mismo valor ¿?¿?¿?
							mesh::proj_nodes[i][0] = (FOV * x) / (FOV + z) + HALFW;
							mesh::proj_nodes[i][1] = (FOV * y) / (FOV + z) + HALFH;
						}
						mesh::next_tick += mesh::SKIP_TICKS;
						loops++;
					}
				}

				void mesh::update( int rotx, int roty, int rotz ){
					loops = 0;

					Matrix <4, 4, float> m_rot ( trotx(rotx)*troty(roty)*trotz(rotz) );

					while( millis() > next_tick && loops < mesh::MAX_FRAMESKIP) {
						
						for (int i=0; i<NODECOUNT; i++) {
				
							float arrayNODES[4][1] = {mesh::nodes[i][0] ,mesh::nodes[i][1],mesh::nodes[i][2], 0};
							
							Matrix <4, 1, float> m_mesh (arrayNODES);
							Matrix <4, 1, float> res = m_rot*m_mesh;

					 		long x =  res(0,0);
							long y =  res(1,0);
							long z =  res(2,0);
							
							// store projected node // Siempre me da el mismo valor ¿?¿?¿?
							mesh::proj_nodes[i][0] = (FOV * x) / (FOV + z) + HALFW;
							mesh::proj_nodes[i][1] = (FOV * y) / (FOV + z) + HALFH;
						}
						mesh::next_tick += mesh::SKIP_TICKS;
						loops++;
					}
				}

				void mesh::draw(TFTCanvas *canvas, uint16_t timer){
					//( (millis() - mesh::next_tick) > timer)
					if (memcmp(mesh::old_nodes, mesh::proj_nodes, sizeof(mesh::proj_nodes)) ) {
					// render frame

						switch(mesh::draw_type) {
							case 0: 
							mesh::draw_vertex( canvas, YELLOW,0);
							mesh::draw_vertex( canvas, BLACK,1);
						break;
							case 1: if (TRICOUNT > 32) {
							mesh::clear_dirty( canvas, mesh::old_nodes);
						}
						else {
							mesh::draw_wireframe( canvas, YELLOW,0);
							mesh::draw_wireframe( canvas, BLACK,1);
						}
						break;
						case 2: mesh::clear_dirty( canvas, WHITE, 0);
							mesh::draw_flat_color( canvas, GREEN,1);
						break;
						case 3: mesh::draw_flat_color( canvas, GREEN,1);
							mesh::draw_wireframe( canvas, YELLOW,1);
						break;
						case 4: mesh::draw_flat_color( canvas, GREEN,1);
						break;
						}
						// copy projected nodes to old_nodes to check if we need to redraw next frame

						memcpy(mesh::old_nodes, mesh::proj_nodes, sizeof(mesh::proj_nodes));
						
					}
				}
				//-----------------------------------------------------------------------------------//
				//------------------------------ Update Mesh ----------------------------------------//
				//--- paramater f is a function for scale,rotation, translation management of mesh---//

				void update_mesh( Matrix<4, 4, float> *f(), mesh *model ){
					loops = 0;
					while( millis() > next_tick && loops < model->MAX_FRAMESKIP) {
						f();

						for (int i=0; i<NODECOUNT; i++) {
				
							float arrayNODES[4][1] = {model->nodes[i][0] ,model->nodes[i][1],model->nodes[i][2], 0};
							Matrix <4, 1, float> m_mesh (arrayNODES);

							Matrix <4, 1, float> res = m_world*m_mesh;

					 		long x =  res(0,0);
							long y =  res(1,0);
							long z =  res(2,0);
							
							// store projected node // Siempre me da el mismo valor ¿?¿?¿?
							model->proj_nodes[i][0] = (FOV * x) / (FOV + z) + HALFW;
							model->proj_nodes[i][1] = (FOV * y) / (FOV + z) + HALFH;
						}
						model->next_tick += model->SKIP_TICKS;
						loops++;
					}
				}

				//-----------------------------------------------------------------------------------//
				//------------------------------ TFTCanvas Draw Mesh --------------------------------//
				//--- paramater f is a function for scale,rotation, translation management of mesh---//

				void TFTCanvas::draw_mesh( int (*old_listnodes)[2], int (*proj_listnodes)[2], uint16_t newcolor, uint16_t oldcolor, int draw_type  ){
					
					if (memcmp(old_listnodes, proj_listnodes, sizeof(proj_listnodes))) {
						// render frame
						//int draw_type = 0;
						switch(draw_type) {
						  case 0: 
						  	//draw_vertex( this, *old_listnodes, oldcolor);
							//draw_vertex( this, proj_listnodes, newcolor);
							break;
						  case 1: if (TRICOUNT > 32) {
						  		//clear_dirty( this, old_listnodes);
						  	}
							else {
								//draw_wireframe( this, old_listnodes, oldcolor);
								//draw_wireframe( this, proj_listnodes, newcolor);
							}
							break;
						  case 2: //clear_dirty( this, old_listnodes);
							//draw_flat_color( this, proj_listnodes, newcolor);
							break;
						  case 3: //draw_flat_color( this, proj_nodes, newcolor);
							//draw_wireframe( this, proj_listnodes, oldcolor);
							break;
						  case 4: //draw_flat_color( this, proj_listnodes, newcolor);
							break;
						}
						// copy projected nodes to old_nodes to check if we need to redraw next frame
						for (int i=0; i<NODECOUNT; i++) {
							DUMP (" proj node X ", proj_listnodes[i][0] );
							DUMP (" proj node Y ", proj_listnodes[i][1] );
							DUMP (" proj node Z ", proj_listnodes[i][2] );
							DUMPPRINTLN();
			            }
			            for (int i=0; i<NODECOUNT; i++) {
							DUMP (" old node X ", old_listnodes[i][0] );
							DUMP (" old node Y ", old_listnodes[i][1] );
							DUMP (" old node Z ", old_listnodes[i][2] );
							DUMPPRINTLN();
			            }
						//memcpy(&old_listnodes, &proj_listnodes, sizeof(proj_listnodes));
						memcpy(&old_listnodes, &proj_listnodes, sizeof(float)*NODECOUNT*2);
						for (int i=0; i<NODECOUNT; i++) {
						  DUMP (" old node post X ", old_listnodes[i][0] );
						  DUMP (" old node post Y ", old_listnodes[i][1] );
						  DUMP (" old node post Z ", old_listnodes[i][2] );
						  DUMPPRINTLN();
						}
						
					}
				}
			#endif

				void clear_dirty(TFTCanvas *canvas, const int (*n)[2] , uint16_t color = WHITE ){
					unsigned char x0=canvas->width(), y0=canvas->height(), x1=0, y1=0, c, w, h;
					// get bounding box of mesh
					for (c=0; c<NODECOUNT; c++) {
						if (n[c][0] < x0) x0 = n[c][0];
						if (n[c][0] > x1) x1 = n[c][0];
						if (n[c][1] < y0) y0 = n[c][1];
						if (n[c][1] > y1) y1 = n[c][1];
					}
					// clear area
					canvas->spi_begin();
					canvas->setAddrWindow_(x0, y0, x1, y1);
					h = (y1-y0);
					w = (x1-x0)+1;
					do {
						canvas->spiWrite16(color, w);
					} while (h--);
					canvas->spi_end();
				};

				void draw_print(TFTCanvas *canvas, const int16_t color, int draw_type) {
					canvas->setCursor(0, 2);
					canvas->setTextColor(color);
					canvas->setTextSize(0);
					switch(draw_type) {
						case 0: canvas->println(F(" vertex"));
							canvas->print(F(" count: "));
							canvas->print(NODECOUNT);
							break;
						case 1: canvas->println(F(" wireframe"));
							canvas->print(F(" triangles: "));
							canvas->println(TRICOUNT);
							break;
						case 2: canvas->println(F(" flat color"));
							canvas->println(F(" mask clear"));
							canvas->print(F(" triangles: "));
							canvas->println(TRICOUNT);
							break;
						case 3: canvas->println(F(" flat color"));
							canvas->println(F(" wireframe clear"));
							canvas->print(F(" triangles: "));
							canvas->println(TRICOUNT);
							break;
						case 4: canvas->println(F(" flat color"));
							canvas->println(F(" no clearscreen"));
							canvas->print(F(" triangles: "));
							canvas->println(TRICOUNT);
							break;
					}
				}
		#endif

#endif
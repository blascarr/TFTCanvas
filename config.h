/*
  Design and created by Blascarr

  TFTCanvas
  Name    : Blascarr
  Description: config.h
  version : 1.0

 *  TFTCanvas.h Library for TFTCanvas Matrix Integration
 *
 *	This Library gives some helpful classes to manipulate Matrix Frames on a TFT Screen
 *	This library is part of a educational course to learn C++ in practice in http://www.blascarr.com/ webpage
 *	
 *	With TFTCanvas we extend some dependency issues for use a variety of TFTScreens with Adafruit TFT library .
 *	Nowadays is supported
 *
 *		· ST7735 			https://github.com/adafruit/Adafruit-ST7735-Library
 *		· SPFD5408 			https://github.com/adafruit/TFTLCD-Library
 *		· ILI9341 			https://github.com/adafruit/TFTLCD-Library
 *
 *	BasicLinearAlgebra is needed
 *
 *	https://github.com/tomstewart89/BasicLinearAlgebra
 *
 *  Written by Adrian for Blascarr
 */

#define BUFFPIXEL 20

/*#ifdef _ADAFRUIT_ST7735H_
	#define _ST7735H_			//-------Uncomment this line to use ST7735 TFT with Adafruit-ST7735-Library-------//
#endif

#ifdef _ADAFRUIT_ILI9341H_
	#define _ILI9341_			//-------Uncomment this line to use ILI9341 TFT with Adafruit-ILI9341-Library-------//
#endif

#ifdef _ADAFRUIT_TFTLCD_H_
	#define _SPFD5408_		//-------Uncomment this line to use SPFD5408 TFT with TFTLCD-Library-------//
#endif*/

//#define _ST7735H_		//-------Uncomment this line to use ST7735 TFT with Adafruit-ST7735-Library-------//
#define _ILI9341_ 		//-------Uncomment this line to use ILI9341 TFT with Adafruit-ILI9341-Library-------//
//#define _SPFD5408_		//-------Uncomment this line to use SPFD5408 TFT with TFTLCD-Library-------//

/*--------------------------------------------------------------*/


#ifdef _ST7735H_

	//#include <Adafruit_ST7735.h>

	#define TFTLib Adafruit_ST7735
	//#define mosi 16
	#define TFT_CS    7 //Chip Select
	#define TFT_DC   0  //Data/Command 
	#define TFT_RST  1
	#define TFT_SD_CS 5

	#define BLACK 0x000001
	#define WHITE 0xFFFFFF
	#define BLUE 0x0000FF
	#define YELLOW 0x00FF00
	#define ROSE 0x99AA66


#endif


/*--------------------------------------------------------------*/

#ifdef _ILI9341_
	#define TFTLib Adafruit_ILI9341
	#define TFT_CS   7 //Chip Select
	#define TFT_DC   6  //Data/Command 
 	#define TFT_SD_CS 8

	#define BLACK 0x000001
	#define WHITE 0xFFFFFF
	#define YELLOW 0x00FF00
#endif					


/*--------------------------------------------------------------*/

/*-------SPFD5408 also use an identifier for TFT Chip -----//

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
#ifdef _SPFD5408_

	#define TFTLib Adafruit_TFTLCD
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
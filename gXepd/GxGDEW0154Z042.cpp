
#include "GxGDEW0154Z042.h"

//#define DISABLE_DIAGNOSTIC_OUTPUT

#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif

#include "ico_l1.h"

#include "ico_l2.h"
#include "battery.h"
#include "cal.h"
#include "clock.h"
#include "comp.h"
#include "distance.h"
#include "up.h"
#include "down.h"
#include "humidity.h"
#include "mountains.h"
#include "temp.h"
#include "statusTabInactive.h"
#include "statusTabActive.h"

#include "newFont.h"
#include "newFontSmall.h"

#define GxGDEW0154Z04_PU_DELAY 300
// the physical number of pixels (for controller parameter)
#define GxGDEW0154Z04_X_PIXELS 200
#define GxGDEW0154Z04_Y_PIXELS 200

//#define GxGDEW0154Z04_WIDTH  GxGDEW0154Z04_X_PIXELS
//#define GxGDEW0154Z04_HEIGHT GxGDEW0154Z04_Y_PIXELS

const uint8_t LUTDefault_full[] PROGMEM =
{
  0x32,  // command
  0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t LUTDefault_part[] PROGMEM =
{
  0x32,	// command
  0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t GxGDEW0154Z04::lut_vcom0[] PROGMEM = {  0x0E  , 0x14 , 0x01 , 0x0A , 0x06 , 0x04 , 0x0A , 0x0A , 0x0F , 0x03 , 0x03 , 0x0C , 0x06 , 0x0A , 0x00 };
const uint8_t GxGDEW0154Z04::lut_w[] PROGMEM = {  0x0E  , 0x14 , 0x01 , 0x0A , 0x46 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x86 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04::lut_b[] PROGMEM = {  0x0E  , 0x14 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x4A , 0x04 };
const uint8_t GxGDEW0154Z04::lut_g1[] PROGMEM = { 0x8E  , 0x94 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04::lut_g2[] PROGMEM = { 0x8E  , 0x94 , 0x01 , 0x8A , 0x06 , 0x04 , 0x8A , 0x4A , 0x0F , 0x83 , 0x43 , 0x0C , 0x06 , 0x0A , 0x04 };
const uint8_t GxGDEW0154Z04::lut_vcom1[] PROGMEM = {  0x03  , 0x1D , 0x01 , 0x01 , 0x08 , 0x23 , 0x37 , 0x37 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };
const uint8_t GxGDEW0154Z04::lut_red0[] PROGMEM = { 0x83  , 0x5D , 0x01 , 0x81 , 0x48 , 0x23 , 0x77 , 0x77 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };
const uint8_t GxGDEW0154Z04::lut_red1[] PROGMEM = { 0x03  , 0x1D , 0x01 , 0x01 , 0x08 , 0x23 , 0x37 , 0x37 , 0x01 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 , 0x00 };

const uint8_t GDOControl[] = {0x01, (GxGDEW0154Z04_Y_PIXELS - 1) % 256, (GxGDEW0154Z04_Y_PIXELS - 1) / 256, 0x00}; //for 1.54inch
const uint8_t softstart[] = {0x0c, 0xd7, 0xd6, 0x9d};
const uint8_t VCOMVol[] = {0x2c, 0x9b}; // VCOM 7c
const uint8_t DummyLine[] = {0x3a, 0x1a}; // 4 dummy line per gate
const uint8_t Gatetime[] = {0x3b, 0x08}; // 2us per line

const uint8_t GxGDEW0154Z04::bw2grey[] =
{
  0b00000000, 0b00000011, 0b00001100, 0b00001111,
  0b00110000, 0b00110011, 0b00111100, 0b00111111,
  0b11000000, 0b11000011, 0b11001100, 0b11001111,
  0b11110000, 0b11110011, 0b11111100, 0b11111111,
};

GxGDEW0154Z04::GxGDEW0154Z04(GxIO& io, int8_t rst, int8_t busy)
  : GxEPD(GxGDEW0154Z04_WIDTH, GxGDEW0154Z04_HEIGHT), IO(io),
  _current_page(-1), _diag_enabled(false),
  _rst(rst), _busy(busy) 
{
}
/* ------------------------------------------------------------------*/
/* ------------------------------------------------------------------*/

void GxGDEW0154Z04::initScreen()
{
	int textSize = 10;

	int iconPosY[2] = { 1, (rowWidth + iconEdgeSpace) };
	int iconPosX[4] = { iconEdgeSpace, (xIconSpace + iconEdgeSpace), (xIconSpace * 2 + iconEdgeSpace), (xIconSpace * 3 + iconEdgeSpace) };



	//Column 1
	screen.scDate = { iconPosX[0] + textSize  , 28 };
	screen.scTime = { iconPosX[0] + textSize + 20, 35 };
	screen.humid = { iconPosX[1] + 20, 30 };
	screen.curTemp = { iconPosX[2] + 20, 20 };
	screen.tempH = { iconPosX[2] + 10, 80 };
	screen.tempL = { iconPosX[2] + 30, 80 };
	screen.comp = { iconPosX[3] + textSize + 2, 35 };
	screen.avgSpd = { iconPosX[3] + textSize + 20, 35 };


	// Column 2
	screen.curAlt = { iconPosX[0] + 20, iconPosY[1] + 25 };
	screen.altGain = { iconPosX[1]     , iconPosY[1] + 35 };
	screen.altLow = { iconPosX[1] + 20, iconPosY[1] + 30 };
	screen.distance = { iconPosX[2] + 10, iconPosY[1] + 30 };
	screen.tripTime = { iconPosX[2] + 30, iconPosY[1] + 30 };
	screen.battV = { iconPosX[3] + 15, iconPosY[1] + 30 };

	//Column 1
	screen.iconDT = { iconPosX[0]   , iconPosY[0] };
	screen.iconHum = { iconPosX[1]   , iconPosY[0] };
	screen.iconTemp = { iconPosX[2] + 5   , iconPosY[0] - 5 };
	screen.iconTempH = { iconPosX[2] - 2   , iconPosY[0] + 65 };
	screen.iconTempL = { iconPosX[2] + 18, iconPosY[0] + 65 };
	screen.iconComp = { iconPosX[3] + 5   , iconPosY[0] };
	screen.iconSpd = { iconPosX[3]   , iconPosY[0] };

	// Column 2       
	screen.iconAlt = { iconPosX[0], iconPosY[1] };
	screen.iconDis = { iconPosX[2] - 5, iconPosY[1] };
	//  screen.iconTempH = {iconPosX[2], iconPosY[1]+30};
	//  screen.iconTempL = {iconPosX[2]+10, iconPosY[1]+30};
	screen.iconAltH = { iconPosX[0] + 30, iconPosY[1] + 10 };
	screen.iconAltL = { iconPosX[0] + 45, iconPosY[1] + 10 };
	screen.iconBatt = { iconPosX[3], iconPosY[1] };

	//Status bar
	screen.iconStatusBar1 = { yScreenSize   , 0 };
	screen.iconStatusBar2 = { yScreenSize   , 40 };
	screen.iconStatusBar3 = { yScreenSize   , 80 };
	screen.iconStatusBar4 = { yScreenSize   , 120 };
	screen.iconStatusBar5 = { yScreenSize   , 160 };


}

void GxGDEW0154Z04::dispLayout()
{
	setRotation(0);
	fillScreen(GxEPD_WHITE);

	//Vertical line
	fillRect(screenSize / numCols, 0, lineWidth, screenSize - statusBarSize, GxEPD_BLACK);
	//Horz
	fillRect(0, rowHeight, screenSize / 2, lineWidth, GxEPD_BLACK);
	fillRect(0, rowHeight * 2, screenSize / 2, lineWidth, GxEPD_BLACK);
	fillRect(screenSize / 2, rowHeight * 2 - 10, screenSize, lineWidth, GxEPD_BLACK);
	fillRect(0, rowHeight * 3, screenSize, lineWidth, GxEPD_BLACK);
	fillRect(0, screenSize - statusBarSize, screenSize, lineWidth, GxEPD_BLACK);


	// Draw Icons
	drawBitmap(gImage_cal, screen.iconDT.Y, screen.iconDT.X, iconSize, iconSize, GxEPD_BLACK, 1);
	drawBitmap(gImage_humidity, screen.iconHum.Y, screen.iconHum.X, iconSize, iconSize, GxEPD_BLACK, 0);
	drawBitmap(gImage_temp, screen.iconTemp.Y, screen.iconTemp.X, iconSize, iconSize, GxEPD_BLACK);
	drawBitmap(gImage_up, screen.iconTempH.Y, screen.iconTempH.X, iconSizeSmall, iconSizeSmall, GxEPD_BLACK);
	drawBitmap(gImage_down, screen.iconTempL.Y, screen.iconTempL.X, iconSizeSmall, iconSizeSmall, GxEPD_BLACK);
	drawBitmap(gImage_comp, screen.iconComp.Y, screen.iconComp.X, iconSize, iconSize, GxEPD_BLACK);
	drawBitmap(gImage_mountains, screen.iconAlt.Y, screen.iconAlt.X, iconSize, iconSize, GxEPD_BLACK);
	drawBitmap(gImage_distance, screen.iconDis.Y, screen.iconDis.X, iconSize, iconSize, GxEPD_BLACK);
	drawBitmap(gImage_up, screen.iconAltH.Y, screen.iconAltH.X, iconSizeSmall, iconSizeSmall, GxEPD_BLACK);
	drawBitmap(gImage_down, screen.iconAltL.Y, screen.iconAltL.X, iconSizeSmall, iconSizeSmall, GxEPD_BLACK);
	drawBitmap(gImage_battery, screen.iconBatt.Y, screen.iconBatt.X, iconSize, iconSize, GxEPD_BLACK);


	fillRect(0, yScreenSize, screenSize, statusBarSize, GxEPD_BLACK);
	//drawBitmap(gImage_statusTabInactive      , screen.iconStatusBar1.Y+1 , screen.iconStatusBar1.X , statusBarSize     , 40     , GxEPD_BLACK);
	//drawBitmap(gImage_statusTabInactive      , screen.iconStatusBar2.Y+1 , screen.iconStatusBar2.X , statusBarSize     , 40     , GxEPD_BLACK);
	///drawBitmap(gImage_statusTabInactive      , screen.iconStatusBar3.Y+1, screen.iconStatusBar3.X , statusBarSize     , 40     , GxEPD_BLACK);
	//drawBitmap(gImage_statusTabInactive      , screen.iconStatusBar4.Y+1, screen.iconStatusBar4.X , statusBarSize     , 40     , GxEPD_BLACK);
	//drawBitmap(gImage_statusTabInactive      , screen.iconStatusBar5.Y+1, screen.iconStatusBar5.X , statusBarSize     , 40     , GxEPD_BLACK);

	//update();

	setStatus( false, 0);
	setStatus( false, 1);
	setStatus( false, 2);
	setStatus( false, 3);
	setStatus(false, 4);

	//drawBitmap(gImage_ico_l1, 0,0, 200, 200, GxEPD_BLACK);
	// drawBitmap(gImage_ico_l2, 0,0, 200, 200, GxEPD_BLACK);

}


void GxGDEW0154Z04::setDate(char* dateIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.scDate.Y, screen.scDate.X);
	setTextColor(GxEPD_RED);
	print(dateIN);
}

void GxGDEW0154Z04::setTime(char* timeIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.scTime.Y, screen.scTime.X);
	setTextColor(GxEPD_RED);
	print(timeIN);
}



void GxGDEW0154Z04::setHumidity(float humidityIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.humid.Y, screen.humid.X);
	setTextColor(GxEPD_RED);
	print(humidityIN, 1);
	setTextColor(GxEPD_BLACK);
	print(" %");
}

void GxGDEW0154Z04::setTemp(float tempIN, float tempHIN, float tempLIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.curTemp.Y, screen.curTemp.X);
	setTextColor(GxEPD_RED);
	print(tempIN, 1);
	setTextColor(GxEPD_BLACK);
	drawCircle(screen.curTemp.Y + 35, screen.curTemp.X - 13, 2, GxEPD_BLACK);
	f = &SansSerif_bold_13;
	setFont(f);
	print("C");


	f = &SansSerif_bold_13;
	setFont(f);

	setCursor(screen.tempH.Y, screen.tempH.X);
	setTextColor(GxEPD_RED);
	print(tempHIN, 0);
	setTextColor(GxEPD_BLACK);
	print((char)223);
	//print("C");

	setCursor(screen.tempL.Y, screen.tempL.X);
	setTextColor(GxEPD_RED);
	print(tempLIN, 0);
	setTextColor(GxEPD_BLACK);
	print((char)223);
	//print("C");

}

void GxGDEW0154Z04::setCompass(float directionIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.comp.Y, screen.comp.X);
	setTextColor(GxEPD_RED);
	print(directionIN, 0);
	drawCircle(screen.comp.Y + 35, screen.comp.X - 12, 2, GxEPD_BLACK);
	print((char)223);
}
void GxGDEW0154Z04::setSpeed(float speedIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.avgSpd.Y, screen.avgSpd.X);
	setTextColor(GxEPD_RED);
	print(speedIN, 1);
	f = &SansSerif_bold_13;
	setFont(f);

	setTextColor(GxEPD_BLACK);
	print(" mph");
}
void GxGDEW0154Z04::setAlt(float altIN, float altHIN, float altLIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.curAlt.Y, screen.curAlt.X);
	setTextColor(GxEPD_RED);
	print(altIN, 0);
	setTextColor(GxEPD_BLACK);
	print(" ft");

	setCursor(screen.altGain.Y, screen.altGain.X);
	setTextColor(GxEPD_RED);
	print(altHIN, 0);
	setTextColor(GxEPD_BLACK);
	print(" ft");

	setCursor(screen.altLow.Y, screen.altLow.X - 1);
	setTextColor(GxEPD_RED);
	print(altLIN, 0);
	setTextColor(GxEPD_BLACK);
	print(" ft");
}
void GxGDEW0154Z04::setDistance(float distanceIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.distance.Y, screen.distance.X);
	setTextColor(GxEPD_RED);
	print(distanceIN, 2);
	setTextColor(GxEPD_BLACK);
	print(" mi");

	setCursor(screen.distance.Y, screen.distance.X + 20);
	setTextColor(GxEPD_RED);
	print("002:15");
	setTextColor(GxEPD_BLACK);
	//print(" {H:M:S)");
}

void GxGDEW0154Z04::setBattery(float batteryIN)
{
	const GFXfont* f = &FreeSansBold9pt7b;
	setFont(f);
	setCursor(screen.battV.Y, screen.battV.X);
	setTextColor(GxEPD_RED);
	print(batteryIN, 2);
	setTextColor(GxEPD_BLACK);
	print(" V");

	f = &SansSerif_bold_11;
	setFont(f);
	setCursor(screen.battV.Y - 25, screen.battV.X + 22);
	setTextColor(GxEPD_RED);
	print("01  14  53");
	setTextColor(GxEPD_BLACK);
	setCursor(screen.battV.Y - 23, screen.battV.X + 22);
	print("   d    h    m");
}

void GxGDEW0154Z04::setStatus(bool statusIN, int statusNum)
{

	int vertStart = 40 * (statusNum + 1) - 3;
	const GFXfont* f = &SansSerif_bold_8;
	int statusTxtBoarderOffset, statusTxtNewLineOffset;
	setFont(f);

	if (!statusIN) {
		// Draw Black background
		drawBitmap(gImage_statusTabActive, 40 * statusNum, screen.iconStatusBar1.X, 40, 30, GxEPD_WHITE);
		setTextColor(GxEPD_BLACK);
	}
	else {
		// Draw background behind red tab
		drawBitmap(gImage_statusTabActive, 40 * statusNum, screen.iconStatusBar1.X, 40, statusBarSize, GxEPD_RED);
		fillRect(40 * (statusNum), yScreenSize, 40, 3, GxEPD_BLACK);
		fillRect(40 * (statusNum + 1) - 2, yScreenSize, 2, 30, GxEPD_BLACK);

		fillRect(40 * (statusNum + 1) - 3, yScreenSize, 1, 8, GxEPD_BLACK);
		fillRect(40 * (statusNum + 1) - 7, yScreenSize + 3, 5, 1, GxEPD_BLACK);
		drawPixel(40 * (statusNum + 1) - 5, yScreenSize + 4, GxEPD_BLACK);
		drawPixel(40 * (statusNum + 1) - 4, yScreenSize + 4, GxEPD_BLACK);
		drawPixel(40 * (statusNum + 1) - 4, yScreenSize + 5, GxEPD_BLACK);
	}

	statusTxtBoarderOffset = 15;
	statusTxtNewLineOffset = statusTxtBoarderOffset + 10;
	if (statusNum == 0) {
		setCursor(40 * statusNum, yScreenSize + 15);
		if (statusIN) {
			setTextColor(GxEPD_WHITE);
			print("  GPS \n  Acq");
		}
		else {
			setTextColor(GxEPD_BLACK);
			print("  GPS \n  Off");
		}

	}
	else if (statusNum == 1)
	{
		if (statusIN) {
			setTextColor(GxEPD_WHITE);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("  GPS");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print(" Track");
		}
		else {
			setTextColor(GxEPD_BLACK);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("  GPS");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print(" Track");
		}
	}

	else if (statusNum == 2)
	{
		if (statusIN) {
			setTextColor(GxEPD_WHITE);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("  GPS");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print("  On");
		}
		else {
			setTextColor(GxEPD_BLACK);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("  GPS");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print("   Off");
		}
	}
	else if (statusNum == 3)
	{
		if (statusIN) {
			setTextColor(GxEPD_WHITE);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("   BT");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print("   On");
		}
		else {
			setTextColor(GxEPD_BLACK);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("   BT");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print("   Off");
		}

	}
	else if (statusNum == 4)
	{
		if (statusIN) {
			setTextColor(GxEPD_WHITE);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print(" SD");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print(" OFF");
		}
		else {
			setTextColor(GxEPD_BLACK);
			setCursor(40 * statusNum, yScreenSize + statusTxtBoarderOffset);
			print("   SD");
			setCursor(40 * statusNum, yScreenSize + statusTxtNewLineOffset);
			print(" Write");
		}
	}
}




/* ------------------------------------------------------------------*/
/* ------------------------------------------------------------------*/
void GxGDEW0154Z04::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) return;

  // check rotation, move pixel around if necessary
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      x = GxGDEW0154Z04_WIDTH - x - 1;
      break;
    case 2:
      x = GxGDEW0154Z04_WIDTH - x - 1;
      y = GxGDEW0154Z04_HEIGHT - y - 1;
      break;
    case 3:
      swap(x, y);
      y = GxGDEW0154Z04_HEIGHT - y - 1;
      break;
  }
  uint16_t i = x / 8 + y * GxGDEW0154Z04_WIDTH / 8;
  if (_current_page < 1)
  {
    if (i >= sizeof(_black_buffer)) return;
  }
  else
  {
    if (i < GxGDEW0154Z04_PAGE_SIZE * _current_page) return;
    if (i >= GxGDEW0154Z04_PAGE_SIZE * (_current_page + 1)) return;
    i -= GxGDEW0154Z04_PAGE_SIZE * _current_page;
  }

  _black_buffer[i] = (_black_buffer[i] & (0xFF ^ (1 << (7 - x % 8)))); // white
  _red_buffer[i] = (_red_buffer[i] & (0xFF ^ (1 << (7 - x % 8)))); // white
  if (color == GxEPD_WHITE) return;
  else if (color == GxEPD_BLACK) _black_buffer[i] = (_black_buffer[i] | (1 << (7 - x % 8)));
  else if (color == GxEPD_RED) _red_buffer[i] = (_red_buffer[i] | (1 << (7 - x % 8)));
  else
  {
    if ((color & 0xF100) > (0xF100 / 2)) _red_buffer[i] = (_red_buffer[i] | (1 << (7 - x % 8)));
    else if ((((color & 0xF100) >> 11) + ((color & 0x07E0) >> 5) + (color & 0x001F)) < 3 * 255 / 2)
    {
      _black_buffer[i] = (_black_buffer[i] | (1 << (7 - x % 8)));
    }
  }
}


void GxGDEW0154Z04::init(uint32_t serial_diag_bitrate)
{
  if (serial_diag_bitrate > 0)
  {
    Serial.begin(serial_diag_bitrate);
    _diag_enabled = true;
  }
  IO.init();
  IO.setFrequency(4000000); // 4MHz
  if (_rst >= 0)
  {
    digitalWrite(_rst, HIGH);
    pinMode(_rst, OUTPUT);
  }
  pinMode(_busy, INPUT);
  fillScreen(GxEPD_WHITE);
  _current_page = -1;
}

void GxGDEW0154Z04::fillScreen(uint16_t color)
{
  uint8_t black = 0x00;
  uint8_t red = 0x00;
  if (color == GxEPD_WHITE);
  else if (color == GxEPD_BLACK) black = 0xFF;
  else if (color == GxEPD_RED) red = 0xFF;
  else if ((color & 0xF100) > (0xF100 / 2))  red = 0xFF;
  else if ((((color & 0xF100) >> 11) + ((color & 0x07E0) >> 5) + (color & 0x001F)) < 3 * 255 / 2) black = 0xFF;
  for (uint16_t x = 0; x < sizeof(_black_buffer); x++)
  {
    _black_buffer[x] = black;
    _red_buffer[x] = red;
  }
}

void GxGDEW0154Z04::update(void)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    uint8_t data = (i < sizeof(_black_buffer)) ? ~_black_buffer[i] : 0xFF;
    _writeData(bw2grey[(data & 0xF0) >> 4]);
    _writeData(bw2grey[data & 0x0F]);
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    _writeData((i < sizeof(_red_buffer)) ? ~_red_buffer[i] : 0xFF);
  }
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("update");
  _sleep();
}

void  GxGDEW0154Z04::drawBitmap(const uint8_t *bitmap, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color, int16_t mode)
{
  if (mode & bm_default) mode |= bm_normal; // no change
  drawBitmapBM(bitmap, x, y, w, h, color, mode);
}

void GxGDEW0154Z04::drawExamplePicture(const uint8_t* black_bitmap, const uint8_t* red_bitmap, uint32_t black_size, uint32_t red_size)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE * 2; i++)
  {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
    _writeData((i < black_size) ? pgm_read_byte(&black_bitmap[i]) : 0x00);
#else
    _writeData((i < black_size) ? black_bitmap[i] : 0x00);
#endif
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
    _writeData((i < red_size) ? pgm_read_byte(&red_bitmap[i]) : 0x00);
#else
    _writeData((i < red_size) ? red_bitmap[i] : 0x00);
#endif
  }
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPicture");
  _sleep();
}

void GxGDEW0154Z04::drawPicture(const uint8_t* black_bitmap, const uint8_t* red_bitmap, uint32_t black_size, uint32_t red_size, int16_t mode)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    uint8_t data = 0xFF; // white is 0xFF on device
    if (i < black_size)
    {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
      data = pgm_read_byte(&black_bitmap[i]);
#else
      data = black_bitmap[i];
#endif
      if (mode & bm_invert) data = ~data;
    }
    _writeData(bw2grey[(data & 0xF0) >> 4]);
    _writeData(bw2grey[data & 0x0F]);
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    uint8_t data = 0xFF; // white is 0xFF on device
    if (i < red_size)
    {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
      data = pgm_read_byte(&red_bitmap[i]);
#else
      data = red_bitmap[i];
#endif
      if (mode & bm_invert_red) data = ~data;
    }
    _writeData(data);
  }
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPicture");
  _sleep();
}

void GxGDEW0154Z04::drawBitmap(const uint8_t* bitmap, uint32_t size, int16_t mode)
{
  if (_current_page != -1) return;
  if (mode & bm_default) mode |= bm_normal; // no change
  uint8_t mask = 0xFF; // black
  //uint8_t mask = 0b0101010101010101; // (light) grey
  //uint8_t mask = 0b1010101010101010; // (dark) grey, same grey
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
#if defined(__AVR) || defined(ESP8266) || defined(ESP32)
    uint8_t data = (i < size) ? pgm_read_byte(&bitmap[i]) : 0x00;
#else
    uint8_t data = (i < size) ? bitmap[i] : 0x00;
#endif
    if (mode & bm_invert) data = ~data;
    _writeData(~(bw2grey[(data & 0xF0) >> 4] & mask));
    _writeData(~(bw2grey[data & 0x0F] & mask));
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    _writeData(0xFF);
  }
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawBitmap");
  _sleep();
}

void GxGDEW0154Z04::eraseDisplay(bool using_partial_update)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE * 2; i++)
  {
    _writeData(0xFF); // white is 0xFF on device
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    _writeData(0xFF); // white is 0xFF on device
  }
  _writeCommand(0x12);      //display refresh
  _waitWhileBusy("eraseDisplay");
  _sleep();
}

void GxGDEW0154Z04::_writeCommand(uint8_t command)
{
  IO.writeCommandTransaction(command);
}

void GxGDEW0154Z04::_writeData(uint8_t data)
{
  IO.writeDataTransaction(data);
}

void GxGDEW0154Z04::_waitWhileBusy(const char* comment)
{
  unsigned long start = micros();
  while (1)
  {
    if (digitalRead(_busy) == 1) break;
    delay(1);
    if (micros() - start > 10000000)
    {
      if (_diag_enabled) Serial.println("Busy Timeout!");
      break;
    }
  }
  if (comment)
  {
#if !defined(DISABLE_DIAGNOSTIC_OUTPUT)
    if (_diag_enabled)
    {
      unsigned long elapsed = micros() - start;
      Serial.print(comment);
      Serial.print(" : ");
      Serial.println(elapsed);
    }
#endif
  }
  (void) start;
}

void GxGDEW0154Z04::_wakeUp()
{
  // reset required for wakeup
  if (_rst >= 0)
  {
    digitalWrite(_rst, 0);
    delay(10);
    digitalWrite(_rst, 1);
    delay(10);
  }

  _writeCommand(0x01);
  _writeData(0x07);
  _writeData(0x00);
  _writeData(0x08);
  _writeData(0x00);
  _writeCommand(0x06);
  _writeData(0x07);
  _writeData(0x07);
  _writeData(0x07);
  _writeCommand(0x04);
  _waitWhileBusy("Power On");
  _writeCommand(0X00);
  _writeData(0xcf);
  _writeCommand(0X50);
  _writeData(0x37);
  _writeCommand(0x30);
  _writeData(0x39);
  _writeCommand(0x61);
  _writeData(0xC8);
  _writeData(0x00);
  _writeData(0xC8);
  _writeCommand(0x82);
  _writeData(0x0E);
  _writeLUT();
}

void GxGDEW0154Z04::_sleep(void)
{
  _writeCommand(0X50);
  _writeData(0x17);    //BD floating
  _writeCommand(0x82);     //to solve Vcom drop
  _writeData(0x00);
  _writeCommand(0x01);     //power setting
  _writeData(0x02);    //gate switch to external
  _writeData(0x00);
  _writeData(0x00);
  _writeData(0x00);
  delay(1500);     //delay 1.5S
  _writeCommand(0X02);     //power off
}

#if defined(__AVR) || defined(ESP8266) || defined(ESP32)

void GxGDEW0154Z04::_writeLUT(void)
{
  unsigned int count;
  // lut_bw
  _writeCommand(0x20);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_vcom0[count]));
  }

  _writeCommand(0x21);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_w[count]));
  }

  _writeCommand(0x22);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_b[count]));
  }

  _writeCommand(0x23);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_g1[count]));
  }

  _writeCommand(0x24);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_g2[count]));
  }
  // lut_red
  _writeCommand(0x25);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_vcom1[count]));
  }

  _writeCommand(0x26);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_red0[count]));
  }

  _writeCommand(0x27);
  for (count = 0; count < 15; count++)
  {
    _writeData(pgm_read_byte(&lut_red1[count]));
  }
}

#else

void GxGDEW0154Z04::_writeLUT(void)
{
  unsigned int count;
  // lut_bw
  _writeCommand(0x20);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_vcom0[count]);
  }

  _writeCommand(0x21);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_w[count]);
  }

  _writeCommand(0x22);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_b[count]);
  }

  _writeCommand(0x23);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_g1[count]);
  }

  _writeCommand(0x24);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_g2[count]);
  }
  // lut_red
  _writeCommand(0x25);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_vcom1[count]);
  }

  _writeCommand(0x26);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_red0[count]);
  }

  _writeCommand(0x27);
  for (count = 0; count < 15; count++)
  {
    _writeData(lut_red1[count]);
  }
}

#endif

void GxGDEW0154Z04::drawPaged(void (*drawCallback)(void))
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback();
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00;
        _writeData(~bw2grey[(data & 0xF0) >> 4]);
        _writeData(~bw2grey[data & 0x0F]);
      }
    }
  }
  _writeCommand(0x13);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback();
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00;
        _writeData(~data);
      }
    }
  }
  _current_page = -1;
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPaged");
  _sleep();
}

void GxGDEW0154Z04::drawPaged(void (*drawCallback)(uint32_t), uint32_t p)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00;
        _writeData(~bw2grey[(data & 0xF0) >> 4]);
        _writeData(~bw2grey[data & 0x0F]);
      }
    }
  }
  _writeCommand(0x13);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00;
        _writeData(~data);
      }
    }
  }
  _current_page = -1;
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPaged");
  _sleep();
}

void GxGDEW0154Z04::drawPaged(void (*drawCallback)(const void*), const void* p)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00;
        _writeData(~bw2grey[(data & 0xF0) >> 4]);
        _writeData(~bw2grey[data & 0x0F]);
      }
    }
  }
  _writeCommand(0x13);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00;
        _writeData(~data);
      }
    }
  }
  _current_page = -1;
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPaged");
  _sleep();
}

void GxGDEW0154Z04::drawPaged(void (*drawCallback)(const void*, const void*), const void* p1, const void* p2)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p1, p2);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00;
        _writeData(~bw2grey[(data & 0xF0) >> 4]);
        _writeData(~bw2grey[data & 0x0F]);
      }
    }
  }
  _writeCommand(0x13);
  for (_current_page = 0; _current_page < GxGDEW0154Z04_PAGES; _current_page++)
  {
    fillScreen(GxEPD_WHITE);
    drawCallback(p1, p2);
    for (int16_t y1 = 0; y1 < GxGDEW0154Z04_PAGE_HEIGHT; y1++)
    {
      for (int16_t x1 = 0; x1 < GxGDEW0154Z04_WIDTH / 8; x1++)
      {
        uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
        uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00;
        _writeData(~data);
      }
    }
  }
  _current_page = -1;
  _writeCommand(0x12); //display refresh
  _waitWhileBusy("drawPaged");
  _sleep();
}

void GxGDEW0154Z04::drawCornerTest(uint8_t em)
{
  if (_current_page != -1) return;
  _wakeUp();
  _writeCommand(0x10);
  for (uint32_t y = 0; y < GxGDEW0154Z04_HEIGHT; y++)
  {
    for (uint32_t x = 0; x < GxGDEW0154Z04_WIDTH / 8; x++)
    {
      uint8_t data = 0xFF;
      if ((x < 1) && (y < 8)) data = 0x00;
      if ((x > GxGDEW0154Z04_WIDTH / 8 - 3) && (y < 16)) data = 0x00;
      if ((x > GxGDEW0154Z04_WIDTH / 8 - 4) && (y > GxGDEW0154Z04_HEIGHT - 25)) data = 0x00;
      if ((x < 4) && (y > GxGDEW0154Z04_HEIGHT - 33)) data = 0x00;
      _writeData(data);
      _writeData(data);
    }
  }
  _writeCommand(0x13);
  for (uint32_t i = 0; i < GxGDEW0154Z04_BUFFER_SIZE; i++)
  {
    _writeData(0xFF);
  }
  _writeCommand(0x12);      //display refresh
  _waitWhileBusy("drawCornerTest");
  _sleep();
}

void GxGDEW0154Z04::updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation)
{
  if (_current_page != -1) return;
  if (using_rotation)
  {
    switch (getRotation())
    {
      case 1:
        swap(x, y);
        swap(w, h);
        x = GxGDEW0154Z04_WIDTH - x - w - 1;
        break;
      case 2:
        x = GxGDEW0154Z04_WIDTH - x - w - 1;
        y = GxGDEW0154Z04_HEIGHT - y - h - 1;
        break;
      case 3:
        swap(x, y);
        swap(w, h);
        y = GxGDEW0154Z04_HEIGHT - y  - h - 1;
        break;
    }
  }
  if (x >= GxGDEW0154Z04_WIDTH) return;
  if (y >= GxGDEW0154Z04_HEIGHT) return;
  // x &= 0xFFF8; // byte boundary, not here, use encompassing rectangle
  uint16_t xe = gx_uint16_min(GxGDEW0154Z04_WIDTH, x + w) - 1;
  uint16_t ye = gx_uint16_min(GxGDEW0154Z04_HEIGHT, y + h) - 1;
  // x &= 0xFFF8; // byte boundary, not needed here
  uint16_t xs_bx = x / 8;
  uint16_t xe_bx = (xe + 7) / 8;
  _using_partial_mode = true;
  if (!_using_partial_mode) _wakeUp();
  _using_partial_mode = true;
  IO.writeCommandTransaction(0x91); // partial in
  _setPartialRamArea(x, y, xe, ye);
  IO.writeCommandTransaction(0x10);
  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_bx; x1 < xe_bx; x1++)
    {
      uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00; // white is 0x00 in buffer
      IO.writeDataTransaction(~data); // white is 0xFF on device
    }
  }
  IO.writeCommandTransaction(0x13);
  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_bx; x1 < xe_bx; x1++)
    {
      uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00; // white is 0x00 in buffer
      IO.writeDataTransaction(~data); // white is 0xFF on device
    }
  }
  IO.writeCommandTransaction(0x12);      //display refresh
  _waitWhileBusy("updateWindow");
  IO.writeCommandTransaction(0x92); // partial out
  delay(GxGDEW0154Z04_PU_DELAY); // don't stress this display
}
uint16_t GxGDEW0154Z04::_setPartialRamArea(uint16_t x, uint16_t y, uint16_t xe, uint16_t ye)
{
  x &= 0xFFF8; // byte boundary
  xe = (xe - 1) | 0x0007; // byte boundary - 1
  IO.writeCommandTransaction(0x90); // partial window
  //IO.writeDataTransaction(x / 256);
  IO.writeDataTransaction(x % 256);
  //IO.writeDataTransaction(xe / 256);
  IO.writeDataTransaction(xe % 256);
  IO.writeDataTransaction(y / 256);
  IO.writeDataTransaction(y % 256);
  IO.writeDataTransaction(ye / 256);
  IO.writeDataTransaction(ye % 256);
  IO.writeDataTransaction(0x01); // don't see any difference
  //IO.writeDataTransaction(0x00); // don't see any difference
  return (7 + xe - x) / 8; // number of bytes to transfer per line
}

/*
void GxGDEW0154Z04::updateWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, bool using_rotation)
{
  if (_current_page != -1) return;
  if (using_rotation) _rotate(x, y, w, h);
  if (x >= GxGDEW0154Z04_WIDTH) return;
  if (y >= GxGDEW0154Z04_HEIGHT) return;
  uint16_t xe = gx_uint16_min(GxGDEW0154Z04_WIDTH, x + w) - 1;
  uint16_t ye = gx_uint16_min(GxGDEW0154Z04_HEIGHT, y + h) - 1;
  uint16_t xs_d8 = x / 8;
  uint16_t xe_d8 = xe / 8;
 // _Init_Part(0x03);
  _SetRamArea(xs_d8, xe_d8, y % 256, y / 256, ye % 256, ye / 256); // X-source area,Y-gate area
  _SetRamPointer(xs_d8, y % 256, y / 256); // set ram
  _waitWhileBusy();
  _writeCommand(0x24);
  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_d8; x1 <= xe_d8; x1++)
    {
      uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_black_buffer)) ? _black_buffer[idx] : 0x00;
      _writeData(~data);
    }
  }
  _Update_Part();
  delay(GxGDEW0154Z04_PU_DELAY);
  // update erase buffer
  _SetRamArea(xs_d8, xe_d8, y % 256, y / 256, ye % 256, ye / 256); // X-source area,Y-gate area
  _SetRamPointer(xs_d8, y % 256, y / 256); // set ram
  _waitWhileBusy();
  _writeCommand(0x24);
  for (int16_t y1 = y; y1 <= ye; y1++)
  {
    for (int16_t x1 = xs_d8; x1 <= xe_d8; x1++)
    {
      uint16_t idx = y1 * (GxGDEW0154Z04_WIDTH / 8) + x1;
      uint8_t data = (idx < sizeof(_red_buffer)) ? _red_buffer[idx] : 0x00;
      _writeData(~data);
    }
  }
  delay(GxGDEW0154Z04_PU_DELAY);
}

void GxGDEW0154Z04::_Init_Part(uint8_t em)
{
  _InitDisplay(em);
  _writeCommandData(LUTDefault_part, sizeof(LUTDefault_part));
  _PowerOn();
}

void GxGDEW0154Z04::_SetRamArea(uint8_t Xstart, uint8_t Xend, uint8_t Ystart, uint8_t Ystart1, uint8_t Yend, uint8_t Yend1)
{
  _writeCommand(0x44);
  _writeData(Xstart);
  _writeData(Xend);
  _writeCommand(0x45);
  _writeData(Ystart);
  _writeData(Ystart1);
  _writeData(Yend);
  _writeData(Yend1);
}

void GxGDEW0154Z04::_SetRamPointer(uint8_t addrX, uint8_t addrY, uint8_t addrY1)
{
  _writeCommand(0x4e);
  _writeData(addrX);
  _writeCommand(0x4f);
  _writeData(addrY);
  _writeData(addrY1);
}

void GxGDEW0154Z04::_PowerOn(void)
{
  _writeCommand(0x22);
  _writeData(0xc0);
  _writeCommand(0x20);
  _waitWhileBusy("_PowerOn");
}

void GxGDEW0154Z04::_PowerOff(void)
{
  _writeCommand(0x22);
  _writeData(0xc3);
  _writeCommand(0x20);
  _waitWhileBusy("_PowerOff");
}

void GxGDEW0154Z04::_writeCommandData(const uint8_t* pCommandData, uint8_t datalen)
{
  if (digitalRead(_busy))
  {
    String str = String("command 0x") + String(pCommandData[0], HEX);
    _waitWhileBusy(str.c_str());
  }
  IO.startTransaction();
  IO.writeCommand(*pCommandData++);
  for (uint8_t i = 0; i < datalen - 1; i++)  // sub the command
  {
    IO.writeData(*pCommandData++);
  }
  IO.endTransaction();

}
void GxGDEW0154Z04::_rotate(uint16_t& x, uint16_t& y, uint16_t& w, uint16_t& h)
{
  switch (getRotation())
  {
    case 1:
      swap(x, y);
      swap(w, h);
      x = GxGDEW0154Z04_WIDTH - x - w - 1;
      break;
    case 2:
      x = GxGDEW0154Z04_WIDTH - x - w - 1;
      y = GxGDEW0154Z04_HEIGHT - y - h - 1;
      break;
    case 3:
      swap(x, y);
      swap(w, h);
      y = GxGDEW0154Z04_HEIGHT - y - h - 1;
      break;
  }
}


void GxGDEW0154Z04::_InitDisplay(uint8_t em)
{
  _writeCommandData(GDOControl, sizeof(GDOControl));  // Pannel configuration, Gate selection
  _writeCommandData(softstart, sizeof(softstart));  // X decrease, Y decrease
  _writeCommandData(VCOMVol, sizeof(VCOMVol));    // VCOM setting
  _writeCommandData(DummyLine, sizeof(DummyLine));  // dummy line per gate
  _writeCommandData(Gatetime, sizeof(Gatetime));    // Gate time setting
  _setRamDataEntryMode(em);
}
void GxGDEW0154Z04::_setRamDataEntryMode(uint8_t em)
{
  const uint16_t xPixelsPar = GxGDEW0154Z04_X_PIXELS - 1;
  const uint16_t yPixelsPar = GxGDEW0154Z04_Y_PIXELS - 1;
  em = gx_uint16_min(em, 0x03);
  _writeCommand(0x11);
  _writeData(em);
  switch (em)
  {
    case 0x00: // x decrease, y decrease
      _SetRamArea(xPixelsPar / 8, 0x00, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00);  // X-source area,Y-gate area
      _SetRamPointer(xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256); // set ram
      break;
    case 0x01: // x increase, y decrease : as in demo code
      _SetRamArea(0x00, xPixelsPar / 8, yPixelsPar % 256, yPixelsPar / 256, 0x00, 0x00);  // X-source area,Y-gate area
      _SetRamPointer(0x00, yPixelsPar % 256, yPixelsPar / 256); // set ram
      break;
    case 0x02: // x decrease, y increase
      _SetRamArea(xPixelsPar / 8, 0x00, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256);  // X-source area,Y-gate area
      _SetRamPointer(xPixelsPar / 8, 0x00, 0x00); // set ram
      break;
    case 0x03: // x increase, y increase : normal mode
      _SetRamArea(0x00, xPixelsPar / 8, 0x00, 0x00, yPixelsPar % 256, yPixelsPar / 256);  // X-source area,Y-gate area
      _SetRamPointer(0x00, 0x00, 0x00); // set ram
      break;
  }
}
/*
void GxGDEW0154Z04::_Init_Full(uint8_t em)
{
  _InitDisplay(em);
  _writeCommandData(LUTDefault_full, sizeof(LUTDefault_full));
  _PowerOn();
}


void GxGDEW0154Z04::_Update_Full(void)
{
  _writeCommand(0x22);
  _writeData(0xc4);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Full");
  _writeCommand(0xff);
}

void GxGDEW0154Z04::_Update_Part(void)
{
  _writeCommand(0x22);
  _writeData(0x04);
  _writeCommand(0x20);
  _waitWhileBusy("_Update_Part");
  _writeCommand(0xff);
}
*/
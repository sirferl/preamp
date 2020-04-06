#include "ferlsOLED.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that its in that state when a sketch starts (and the
// LiquidCrystal constructor is called).

/********************
 * void softSpiTransfer(uint16_t shOut)
 * Software SPI
 * Transfer 10 bit with Bit banging
 ********************/
void softSpiTransfer(uint16_t shOut)
{
  digitalWrite (SS_Pin,LOW);

  for(uint8_t i = 0; i < 10; i++)
  {
    digitalWrite (MOSI_Pin,( (shOut & 0x200) ? 1 : 0));

    delayMicroseconds(1);   //warten damit Datensignale korrekt anliegen
    digitalWrite (CLK_Pin,1);
    delayMicroseconds(1);  //steigen Flanke einlesen
    digitalWrite (CLK_Pin,0);
    delayMicroseconds(1); //warten für nächsten Clockpuls

    shOut <<= 1;
  }
  digitalWrite (SS_Pin,HIGH);
}

/********************
 * void OLED_str(char *str)
 * Show string at current cursor position
 * The string needs to have a 0x00 at the end
 ********************/
void OLED_str(char *str)
{
  while(*str)
    OLED_data(*str++);
}

/********************
 * void OLED_pos (uint8_t line, uint8_t offset)
 * Set cursor position
 * Home Position: line=1, offset=1
 ********************/
void OLEDCrystal::setCursor ( uint8_t offset, uint8_t line)
{
  uint8_t pos = 0;
  line++;
  offset++;
  if(line == 2)
    pos = 0x40;
  else if(line == 3)
    pos = 0x14;
  else if (line == 4)
    pos = 0x54;

  pos += offset-1;
  
  OLED_cmd(0x80+pos);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void OLEDCrystal::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  OLED_cmd(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    //write(byte(charmap[i]));
	OLED_data(byte(charmap[i]));
  }
}

/********************
 * void OLED_clear(void)
 * Clear dislpay
 * Clears whole display and sets cursor to home postion
 ********************/
void OLEDCrystal::OLED_clear(void)
{
  OLED_cmd(0x01);
  delay(2); 
  OLED_cmd(0x02);
  delay(2);
}


/*********** mid level commands, for sending data/cmds */

void OLEDCrystal::command(uint8_t value) {
  softSpiTransfer((uint16_t)value);
}
void OLEDCrystal::write(char *str) {
  while(*str)
		OLED_data(*str++);
}

/********************
 * void OLED_cmd(uint8_t cmd)
 * Command to display
 ********************/
void OLED_cmd(uint8_t cmd)
{
  softSpiTransfer((uint16_t)cmd);
}


/********************
 * void OLED_dat(uint8_t data)
 * Data to display
 ********************/
void OLEDCrystal::OLED_dat(uint8_t data)
{
  softSpiTransfer(0x0200 + (uint16_t)data);
}

void OLED_data(uint8_t data)
{
  softSpiTransfer(0x0200 + (uint16_t)data);
}

// Constructor - just copy the chip-select pin value 
OLEDCrystal::OLEDCrystal(uint8_t ssPin)
{
	_ssPin = ssPin;
}

  
 void OLEDCrystal::begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS)
 {
  pinMode (_ssPin,OUTPUT);
  pinMode (MOSI_Pin, OUTPUT);
  pinMode (CLK_Pin, OUTPUT);
  digitalWrite (_ssPin,1);
  digitalWrite (CLK_Pin,0);
	 
  OLED_cmd(0x39);  //Function set with font selection (european char set)
  OLED_cmd(0x08);  // cursor mode
  OLED_cmd(0x06);  // Entry mode set
  OLED_cmd(0x17);  // Cursor display shift character mode
  OLED_cmd(0x01);  //  reset display
  delay(10); 
  OLED_cmd(0x02);   // Return Home
  delay(10); 
  OLED_cmd(0x0C);  //Display on, cursor off, blink off
}
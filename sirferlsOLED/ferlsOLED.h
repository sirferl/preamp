#include "Arduino.h"
#ifndef OLED_h
#define OLED_h

#include <inttypes.h>
//#include "Print.h"

// Definition of Portpins for Software SPI
#define SS_Pin 9
#define MOSI_Pin 10
#define MISO_Pin 11
#define CLK_Pin  12

// flags for function set
#define LCD_5x8DOTS 0x00


//command set 
#define LCD_SETCGRAMADDR 0x40

void softSpiTransfer(uint16_t shOut);

//class OLEDCrystal : public Print {
class OLEDCrystal {
public:
    OLEDCrystal(uint8_t ssPin); //SPI  Select pin ##########
	void OLED_clear(void);                        //clear Display an return Home
/*	void initSPI(uint8_t _ssPin);*/

	void OLED_dat(uint8_t data);
	
	void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);
	void setCursor ( uint8_t offset, uint8_t line);
	void createChar(uint8_t, uint8_t[]);
	// virtual size_t write(uint8_t);
	void write(char *str);
	void command(uint8_t);
	
private:
    uint8_t _ssPin;
	
};

void softSpiTransfer(uint16_t shOut);
void OLED_data(uint8_t data);

void OLED_str(char *str);                     //Output string at current cursor position
void OLED_pos (uint8_t line, uint8_t offset); //set cursor, home position = 1,1


void OLED_cmd(uint8_t cmd);

//void OLED_init(void);                         //Init OLED (2 LINES)




#endif

// include the library code:

#include<sirferlsOLED.h>
#include <IRremote.h>

//startup message
const char start_message[16] = "Pre-amp V5.1";

// define PGA-Board pins
#define PGA_CLK PORT2
#define PGA_DATA PORT3
// Macros for setting the Pins FAST
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))


// Define LED Pins
#define RED 6

// Define variables for Source Switch
// we read the switch every half second
unsigned long startTime;
unsigned long endTime;
word switchVal;
char cChannel[50];
byte switchPosition,currentChannel;
const char channel[5][6]={
  "PHONO\x0",
  "DAC  \x0",
  "AUX1 \x0",
  "AUX2 \x0",
  "---- \x0"
};


// define LCD Pins
OLEDCrystal lcd(9);  //* SS PIN 10 - rest gos to ISP Pins
// definition of special characters for LCD
uint8_t bararray[7][8] = {
  {B00000, B00000, B00000, B00000, B11111, B00000, B00000, B00000},
  {B00000, B10000, B10000, B10000, B11111, B10000, B10000, B10000},
  {B00000, B11000, B11000, B11000, B11111, B11000, B11000, B11000},
  {B00000, B11100, B11100, B11100, B11111, B11100, B11100, B11100},
  {B00000, B11110, B11110, B11110, B11111, B11110, B11110, B11110},
  {B00000, B11111, B11111, B11111, B11111, B11111, B11111, B11111},
  {B00000, B00000, B00000, B00000, B11111, B00000, B00000, B00000}
};


// initialize the IR REMOTE library with the numbers of the interface pins
/* The IR-Receiver connections :
 *  IR signal pin to digital pin 6
 */
int RECV_PIN = 8; //IR input pin on Arduino
IRrecv irrecv(RECV_PIN);
decode_results results;
// keycodes for remote
const int R_PWR =      0xFFA25D;
const int R_MUTE =     0xFFE21D;
const int R_VOL_UP =   0xFF629D;
const int R_VOL_DOWN = 0xFFA857;

unsigned int lastcode;
int code_count;
int increment;

// states of the preamp
typedef enum { STDBY, NORMAL, MUTED} state_t ;

state_t state = STDBY;

bool mute = false;
bool standby = true;
int nGain =1;

/* 
 *  
 *  Function set Volume Bar
 *  convert gain to a decibal string, and write to the lcd
 *  also set bar graph to this value
 *  
 */
void setVolumeBar(int dGain) {
  char cGain[17];
  float iGain;
  int ii, ival;
  unsigned int fullz, rest;
  iGain = 31.5 - 0.5 * (255 - dGain);
  // lcd.clear();
  // ("Volume: " + gain + " dB").toCharArray(cGain, 17);
  ival = iGain *10;
  sprintf(cGain, "%5s   %+3d.%d dB",channel[currentChannel], ival/10, abs(ival%10));
  //sprintf(cGain, "Volume: %2d %2d  ",dGain , ((dGain % 16) - 2) / 3 );
  lcd.setCursor(0, 0);  //first line
  lcd.write(cGain);

  // show a block at every  16 dGain step
  fullz = dGain / 16;
  // rest of division by 16 - 2 divided by 3
  rest = ((dGain % 16) - 2) / 3;
  
  ii = 0;
  if (fullz > 0) {
    while (ii <= fullz){
       cGain[ii] = byte(5);
       ii++;
    } 
    cGain[fullz] = ((rest==0) ? byte(6) : byte(rest));
    for ( ii=fullz + 1 ; ii < 16; ii++){
      cGain[ii] = byte(6);
    }
  }
  else {
    cGain[0] = ((rest==0) ? byte(6) : byte(rest));
    for ( ii =  1 ; ii < 16; ii++){
        cGain[ii] = byte(6);
    }  
  }
  lcd.setCursor(0, 1);  //second line
  lcd.write(cGain);
}

/* 
 *  
 *  Function setGain
 *  Set the preamp-board to the given level
 *  
 */
void setGain(byte nGain) {
  noInterrupts();
  PGA2311_byteout(nGain); // rght channel
  PGA2311_byteout(nGain); // left channel
  interrupts();
}

/* 
 *  
 *  Function PGA2311_byteout
 *  Write one byte to the PGA2311
 *  CS is not needed, becuase PGA2311 is always selected
 *  
 */
void PGA2311_byteout(byte vol){
 byte mask = 0x80;
  for (int i = 0; i < 8; i++) {
    if (vol & mask)  PORTD |= B00001000;
    else PORTD &= B11110111;
    delayMicroseconds(4);
    PORTD |= B00000100; // CLK high
    delayMicroseconds(4);
    PORTD &= B11110111; // DATA low
    delayMicroseconds(4);
    PORTD &= B11111011; // CLK low
    delayMicroseconds(4);
    // Data is pin 3 , clk is pin 2
    /*
    digitalWrite( PGA_DATA, ((vol & mask) ? 1 : 0));
    digitalWrite(PGA_CLK, HIGH);
    delayMicroseconds(2);
    digitalWrite(PGA_DATA, LOW); //set data to low regardless
    digitalWrite(PGA_CLK, LOW);
    */
    mask >>= 1;                      //delay and prepare for next bit
    
  }
}

/*
* **************************
*  Setting up
* **************************
*/
void setup() {
  // save starttime for channelswitch check
  startTime = millis();
  // initilaize serial line for debugging 
  Serial.begin(9600);
  // initialize LCD and set up the number of columns and rows:
  lcd.begin(16, 2);
  // load volume bar characters to LCD module
  for (int y = 1; y < 8; y++) {
    lcd.createChar(y, bararray[y]);
  }
  
  // start IR receiver
  irrecv.enableIRIn(); 

 // PGA Pins as OUTPUT
 pinMode(PGA_CLK, OUTPUT);
 digitalWrite(PGA_CLK, 0);
 pinMode(PGA_DATA, OUTPUT);
 digitalWrite(PGA_DATA, 0);

  // init the Power LED
  pinMode(RED, OUTPUT);
  analogWrite(RED,2 );

  // start the A0 pin as a pullup source
  pinMode(A0, INPUT_PULLUP);
  currentChannel=10; //unknown channel to force display
}

/*
* **************************
* main loop
* ***************************
*/
void loop() {
  #define delayTime 10

 endTime = millis();
  if ((endTime - startTime) >= 500){
     switchVal = analogRead(A0);  //check switch
     if ( switchVal < 70) {
       switchPosition = 0;
     }  
     else if (switchVal >= 200 && switchVal < 330) {
       switchPosition = 1;
     }  
     else if (switchVal >= 350 && switchVal < 430) {
       switchPosition = 2;
     }
     else if (switchVal >= 450 && switchVal < 500) {
       switchPosition = 3;
     }  
     else switchPosition = 4;
     if ((switchPosition != currentChannel) and ( state == NORMAL)) {
      currentChannel = switchPosition;
      lcd.setCursor(0, 0);  //first line
      lcd.write(channel[currentChannel]);
      
     }
     //sprintf(cChannel, "Reading: %+5d Position: %d Channel: %s", switchVal, switchPosition , channel[switchPosition]);
     //Serial.println (cChannel);
     startTime = endTime;
  //delay (1000);
  }
  
  // Wait for code from remote control
  if (irrecv.decode(&results)) {
     Serial.println(results.value, HEX);
     
      // remote has a repeat function
     if (results.value != 0xFFFFFFFF) {
        lastcode = (results.value);
        code_count = 1;
     }
     else {code_count++;}

     switch (code_count){
       case 1: 
         increment = 1;
         break;
       case 2 ... 4: 
         increment = 2;
         break;
       default: increment = 5;
     } 

     // Switching states
     switch(state) {
      case (STDBY): 
        if (lastcode  == R_PWR) {
          //light up power led
          analogWrite(RED,10 );
          //write proud startup message
          lcd.setCursor(0, 0);  //first line
          lcd.write(start_message);
          delay(2000);
          setGain(nGain);
          setVolumeBar(nGain);
          state = NORMAL;
        }
      break; // end of STDBY section 
      
      case (NORMAL):
      switch (lastcode){
        case R_MUTE:
          lcd.setCursor(0, 0);
          lcd.write("      Muted     ");
          lcd.setCursor(0, 1);
          lcd.write("                     ");
          setGain(1);
          state=MUTED;
          delay(500);
        break; // end of key R_MUTE

        case R_VOL_UP:
          nGain = nGain + increment;
          if (nGain > 255) {nGain = 255;}
          setGain(nGain);
          setVolumeBar(nGain);
        break; // end of key R_VOL_UP
        
        case R_VOL_DOWN:
          nGain = nGain - increment;
          if (nGain < 1) {nGain = 1;}
          setGain(nGain);
          setVolumeBar(nGain);
        break; //end of key R_VOL_DOWN

        case (R_PWR):
          lcd.setCursor(0, 0);
          lcd.write("                ");
          lcd.setCursor(0, 1);
          lcd.write("                ");
          analogWrite(RED,2 );
          setGain(1);
          delay(500);
          state = STDBY;
        break; // end of key R_PWR
      }
      break; //end of NORMAL section 

      case (MUTED):
       if ((lastcode == R_MUTE) || (lastcode == R_VOL_UP)){
          setGain(nGain);
          setVolumeBar(nGain);
          state = NORMAL;
          delay(500);
       }
      break; // end of section MUTED 
     }
     irrecv.resume(); // Receive the next value
  }

}

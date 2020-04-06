# preamp
arduino based remote control for sure PGA2311 preamp module:
http://store.sure-electronics.com/product/AA-AB41148

Before Building you have to import the sirferlsOLED library into the arduino IDE.

In the file define the two ports which are connected to the PGA2311-board remote connector.

#define PGA_CLK PORT2
#define PGA_DATA PORT3
Be sure to connect the GND-Pin as well.

I used this display in serial mode, some soldering need:
https://secure.reichelt.at/display-oled-2x16-80x36mm-weiss-ea-w162-x3lw-p113317.html

There is a channel display as well, because I have a casing that came wit a source switch.



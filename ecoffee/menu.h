#ifndef _MENU_H_
#define _MENU_H_

#include "defaults.h"
#include "Arduino.h"
#ifdef LCD_I2C
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
#else
#include <LiquidCrystal.h>
#endif

//#ifdef LCD_I2C
extern LiquidCrystal_I2C lcd;
//LiquidCrystal_I2C lcd(0x3F, 20, 4);
//#else
//LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
//extern LiquidCrystal lcd;
//#endif

void writeLine(int line, String message = "", int offsetLeft = 0);
void lcd_clearxy(int x, int y,int pos=1);
void lcd_selxy(int x, int y);
void check_encoder();
void display_lcd ();
void init_display();

void PinA();
void PinB();

class idMenu {
	public:
		idMenu(){}
		idMenu(const uint8_t &menu_scr);
		~idMenu(){}
   void lcd_selxy(int x, int y);
		void check_encoder(); //should be put in loop function
	
	private:
		void display_lcd();
    void clear_n_sel(int menu);
		void PinA();
		void PinB();
    
    byte max_sel,min_sel; //According to current selection
    unsigned long lastButtonPress;
    int curr_sel, old_curr_sel;
    byte encoderPos; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
    byte oldEncPos; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
    bool show_changed_options; //Only for display
    bool update_options;
    char tempstr[5],tempstr2[5];
    byte menu_number;

    bool isitem_sel;
    byte old_menu_pos;

};

#endif

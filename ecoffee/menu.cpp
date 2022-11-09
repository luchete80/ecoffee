#include <Arduino.h>
#include "pinout.h"
#include "menu.h"

//Encoder from https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/
int pinA = PIN_ENC_CL; // Our first hardware interrupt pin is digital pin 2
int pinB = PIN_ENC_DIR; // Our second hardware interrupt pin is digital pin 3
byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent

byte max_sel, min_sel; //According to current selection

//
void PinA() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB() {
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


static bool clear_all_display;

void init_display()
{
  #ifdef LCD_I2C
  lcd.begin(16, 2);  //I2C
#else
  lcd.begin(16, 2); //NO I2C
#endif
  //lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
}

void writeLine(int line, String message = "", int offsetLeft = 0) {
  lcd.setCursor(offsetLeft, line);
  lcd.print(message);
}

void lcd_clearxy(int x, int y,int pos=1) {
  for (int i=0;i<pos;i++) {
      lcd.setCursor(x+i, y);
      lcd.print(" ");
  }
}
void idMenu::lcd_selxy(int x, int y) {
  lcd.setCursor(x, y);
  if (!isitem_sel)
      lcd.print(">");
  else 
      lcd.write(byte(0));
}

void idMenu::check_encoder ( ) {
    byte btnState = digitalRead(PIN_ENC_SW);
    if (btnState == LOW) { //SELECTION: Nothing(0),VENT_MODE(1)/BMP(2)/I:E(3)/VOL(4)/PIP(5)/PEEP(6) 
    if (millis() - lastButtonPress > 150) {
      Serial.println("Boton presionado");
      isitem_sel=!isitem_sel;
      if (!isitem_sel)
          curr_sel=encoderPos=old_curr_sel;          
      lastButtonPress = millis();
    }// if time > last button press
       
      if (isitem_sel) {      
            switch (curr_sel){
              case 1: 
               if ( menu_number == 0 ) {
                      min_sel=1;max_sel=2;
                      //encoderPos=oldEncPos=vent_mode;
                  } else if ( menu_number == 1 ) {
                      min_sel=20;max_sel=50;
                      //encoderPos=oldEncPos=alarm_max_pressure;            
               } 
              break;
      
            }
      
          show_changed_options = true;
          update_options = true;
        }//Switch button
    }
  if (oldEncPos != encoderPos) {
    show_changed_options = true;
    
//    if (menu_number==2)
//      change_pid_params=true;
    if (!isitem_sel) { //Selecting position
          curr_sel=encoderPos;
          encoderPos=oldEncPos=curr_sel;
          Serial.print("curr sel: ");Serial.println(curr_sel);
          Serial.print("Encoder pos: ");Serial.println(encoderPos);
		  
      if ( menu_number == 0 ) {
        if (curr_sel > 5) {
          curr_sel=1;
          menu_number+=1;
          clear_all_display=true;
          display_lcd();
        } 
      } else if (menu_number == 1) {
         if (curr_sel > 5) {
          curr_sel=0;
          menu_number=2; 
          clear_all_display=true;
          display_lcd();         
         }
      }
        else if (menu_number == 2) {
         if (curr_sel > 3) {
          curr_sel=0;
          menu_number=0; 
          clear_all_display=true;
          display_lcd();         
         }
      }
    } else {//inside a particular selection
	

      if ( encoderPos > max_sel ) {
         encoderPos=oldEncPos=max_sel; 
      } else if ( encoderPos < min_sel ) {
          encoderPos=oldEncPos=min_sel;
        } else {
       
			oldEncPos = encoderPos;
			oldEncPos = encoderPos;
			switch (curr_sel) {
			  case 1:

				break;
			  case 2:
	//            if ( menu_number == 0 ) options.respiratoryRate = encoderPos;
	//            else                    alarm_peep_pressure     = encoderPos;
				break;
			}
			show_changed_options = true;
			update_options=true;
		}//Valid range
  
    }
  }//oldEncPos != encoderPos and valid between range
    old_curr_sel = curr_sel;
}

void idMenu::clear_n_sel(int menu){
    if (menu==0) {  
        lcd_clearxy(0,0);
        lcd_clearxy(0,1);lcd_clearxy(9,0);
        lcd_clearxy(0,2);lcd_clearxy(8,1);
         switch(curr_sel){
              case 1: 
                lcd_selxy(0,0);break;
              case 2: 
                lcd_selxy(0,1);break;
              case 3:
                lcd_selxy(0,2);break;
              case 4: 
                lcd_selxy(9,0);break;
              case 5: 
                lcd_selxy(8,1);break;
            }
     } else if (menu==1){  
      lcd_clearxy(0,0);
      lcd_clearxy(0,1);lcd_clearxy(10,2);
      lcd_clearxy(0,2);lcd_clearxy(0,3);
      switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;//PIP
          case 2: 
            lcd_selxy(0,1);break;//PEEP
          case 3:
            lcd_selxy(10,1);break;
          case 4: 
            lcd_selxy(0,2);break;
          case 5: 
            lcd_selxy(0,3);break;
      }
    } else if (menu==2) {  
      lcd_clearxy(0,0);
      lcd_clearxy(0,1);lcd_clearxy(10,2);
      lcd_clearxy(0,2);lcd_clearxy(0,3);
      switch(curr_sel){
          case 1: 
            lcd_selxy(0,0);break;//PIP
          case 2: 
            lcd_selxy(6,1);break;//PEEP
          case 3:
            lcd_selxy(12,1);break;
          case 4: 
            lcd_selxy(0,2);break;
          case 5: 
            lcd_selxy(0,3);break;
      }
    }//menu number 


}

void idMenu::display_lcd ( ) {
    if (clear_all_display)
        lcd.clear();        
  clear_n_sel(menu_number);
  if (menu_number==0) {  
    lcd_clearxy(5,1,3); lcd_clearxy(12,0,4);
    lcd_clearxy(5,2,2); lcd_clearxy(14,1,2);
                        lcd_clearxy(13,2,2);
  


      
  } else if (menu_number ==1 ){//OTHER SETTINGS
                        lcd_clearxy(12,0,8);
    lcd_clearxy(8,1,2); lcd_clearxy(16,1,3);
                        lcd_clearxy(13,6,3);
        

  } else if (menu_number ==2 ){//PID
 

  }//menu_number
  
  clear_all_display=false;

}

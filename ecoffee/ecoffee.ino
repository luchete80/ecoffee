#include "pinout.h"
#include "defaults.h"
#include "pid_still.h"
//#include "acs712.h"
#include "menu.h"
#include "wifi.h"

//#include "src/Filters/Filters.h"
#include "src/LiquidCrystal_I2C/LiquidCrystal_I2C.h"
//#define LCD_I2C 1
//#ifdef LCD_I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);
//#else
//LiquidCrystal lcd(PIN_LCD_RS, PIN_LCD_EN, PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
//#endif
float temp;
char tempstr[5];
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Stepper.h>
idWifi wifi;

unsigned long previousMillis = 0;
unsigned long printPeriod = 500;

float Sensibilidad=0.100; //sensibilidad en Voltios/Amperio para sensor de 5A
double corr_rms;

int t_wait_reheat = 20;  //one reached maximum temp
unsigned long time_stop_heat;
//Object definition 
//Stepper           stepper(STEPS, PIN_STEPPER_IN1, PIN_STEPPER_IN2, PIN_STEPPER_IN3, PIN_STEPPER_IN4);
pid_still         pidstill;
idMenu            Menu;
OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);

//ROM = 28 AA 8 55 3D 14 1 AD ROM = 28 75 B4 86 13 19 1 43
DeviceAddress sensor1 = { 0x28, 0xAA, 0x8, 0x55, 0x3D, 0x14, 0x1, 0xAD };
DeviceAddress sensor2 = { 0x28, 0x75, 0xB4, 0x86, 0x13, 0x19, 0x1, 0x43 };

unsigned long temp_readnshowtime;
unsigned long time_sendwifi;
unsigned long curr_time;
//RunningStatistics inputStats;                 // create statistics to look at the raw test signal
float temphead,prev_temphead;


float init_heat_timecount;
float min_dTdt_toheatup;      //Once maximum dTdt has been reached
unsigned long init_heat_time;
bool is_heating;
int analog_zero[]={0,0};
bool ready_to_heat;
bool is_dT_rising_slow;
float dT_max;//dTdt
float dT, prev_dT;//dTdt
int pumpsw_state;

void setup() {
    
    time_stop_heat = 0;
    //wifi.Init();
    pinMode(PIN_PUMPSW,INPUT_PULLUP); 
    //pinMode(PIN_ACS_712,INPUT);  //Define the pin mode
    pinMode(PIN_BREWHEAT,OUTPUT); 
    pinMode(PIN_PUMP,OUTPUT); 
    
    digitalWrite(PIN_PUMP, HIGH); //Inverted,low
    //wifi.Init();
    sensors.begin();
    //init_display();
    lcd.init();
    lcd.backlight();
    //lcd.clear();
    //writeLine(0, "eCoffee v0.1", 2);
    Serial.begin(115200);

    //lcd.clear();
    //lcd.setCursor(0, 0);

    //inputStats.setWindowSecs( windowLength );     //Set the window length
    temp_readnshowtime = time_sendwifi= curr_time = millis();
    lcd.print("eCoffee v0.1.0");
    Serial.println("ecoffee");
    delay(1500);
    writeLine(0, "T:     T:", 0); 
    writeLine(1, "      dT:", 0); 
    
    for (int i=0;i<20;i++)
      analog_zero[0]+=analogRead(PIN_ACS_712);
   analog_zero[0]/=20;

    is_heating = true;
   //Calculate initial heat up time in seconds
   temphead = prev_temphead = sensors.getTempC(sensor1);
   init_heat_timecount = 1.2 * 4184.0 * (95.0 - temphead)/1500.0;

   min_dTdt_toheatup = 0.05;
   init_heat_time=millis();
   is_heating = true;
   digitalWrite(PIN_BREWHEAT, LOW); //inverted
   ready_to_heat = false;
   is_dT_rising_slow = false;

   dT_max = 0.;
   dT = -10.;
}
	
void loop() {

  float rem_time;
  curr_time=millis(); 
  pumpsw_state = digitalRead(PIN_PUMPSW);
  // Keep in mind the pull-up means the pushbutton's logic is inverted. It goes
  // HIGH when it's open, and LOW when it's pressed. 
  if (pumpsw_state == LOW) { //pressed
    digitalWrite(PIN_PUMP, LOW); // RELAY IS INVERTED
  } else {
    digitalWrite(PIN_PUMP, HIGH);
  }
  
  //Not calculate each time     
    if (is_heating) {
      rem_time = init_heat_timecount - (float)(curr_time - init_heat_time)/1000.0;
      Serial.println("rem_time"+String(rem_time));
      if (rem_time < 0.0){
        is_heating = false;
        digitalWrite(PIN_BREWHEAT, HIGH); //inverted
        time_stop_heat = millis();
        }      
    }
    
    if ( pidstill.getState() == WARM_UP ) {
    }
 

   if (is_heating)  writeLine(0, "H", 15);
   else             writeLine(0, " ", 15);

    if ( curr_time > temp_readnshowtime + TIME_READNSHOW_TEMPS ) {   
        sensors.requestTemperatures();
        //Serial.print("Sensor 1(*C): ");
        temp = sensors.getTempC(sensor2); 
        prev_temphead = temphead;
        temphead=sensors.getTempC(sensor1); //THIS IS THE MAIN
        dtostrf(temphead, 2, 1, tempstr);
        writeLine(0, tempstr, 2);

        dtostrf(temp, 2, 1, tempstr);
        //Serial.print(sensors.getTempC(sensor1)); 
        writeLine(0, tempstr, 9);
        //Serial.print("Sensor 2(*C): ");
        //Serial.print(sensors.getTempC(sensor2));    

        prev_dT = dT;
        dT = (temphead - prev_temphead)*1000./TIME_READNSHOW_TEMPS;
        dtostrf(dT, 4, 2, tempstr);
        writeLine(1, tempstr, 9);

        if (is_heating){
          dtostrf(rem_time, 2, 1, tempstr);
          writeLine(1, tempstr, 0);
          } else {
          dtostrf(dT, 2, 0, tempstr);
          writeLine(1, tempstr, 0);            
            }
          
        if (dT>dT_max) dT_max = dT;
        if (dT < prev_dT ) { //slowing rising
          writeLine(1, "S", 15);
          if (dT < 0.05){
            if (temphead < 95.0){
                if (millis()> time_stop_heat + t_wait_reheat * 1000) { //Otherwise cut and start continuosly
                  is_heating = true;
                  digitalWrite(PIN_BREWHEAT, LOW); //inverted
                  init_heat_timecount = 1.2 * 4184.0 * (92.0 - temphead)/1500.0;
                  init_heat_time = millis();
                }
              }
            }
        } else {
          writeLine(0, " ", 15);
        }

        
        temp_readnshowtime = curr_time;
    }


    //        if (alarm_state > 0) {
    //
    //              if (!buzzmuted) {
    //                  if (millis() > timebuzz + TIME_BUZZER) {
    //                      timebuzz=millis();
    //                      isbuzzeron=!isbuzzeron;
    //                      if (isbuzzeron){
    //                          digitalWrite(PIN_BUZZER,BUZZER_LOW);
    //                      }
    //                      else {
    //                          digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //                      }
    //                  }
    //              } else {  //buzz muted
    //                  digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //              }
    //        } else {//state > 0
    //          digitalWrite(PIN_BUZZER,!BUZZER_LOW);
    //          isbuzzeron=true;        //Inverted logic
    //        }

//    if (curr_time > time_sendwifi + TIME_SEND_WIFI) {
//        wifi.sendFloat(temphead);
//        time_sendwifi = curr_time;
//    }
    //lcd.print("T:     Tr:");
//    stepper.setSpeed(1); // 1 rpm
//    stepper.step(2038); // do 2038 steps -- corresponds to one revolution in one minute

//    stepper.setSpeed(6); // 6 rpm
//    stepper.step(-2038); // do 2038 steps in the other direction with faster speed -- corresponds to one revolution in 10 seconds
 
//          
      
      if((unsigned long)(millis() - previousMillis) >= printPeriod) { //every second we do the calculation
        previousMillis = millis();   // update time

        float min=1000;

        for (int i=0;i<1;i++){
          //ACS_Value = analogRead(PIN_ACS_712);  // read the analog in value:
          //inputStats.input(ACS_Value);  // log to Stats function
          //Serial.print(String(ACS_Value) + " ");

          //min = intercept + slope * inputStats.sigma();
          //if (min<Amps_TRMS)
          //  Amps_TRMS=min;
        }
        //Amps_TRMS/=10.;

        corr_rms=ac_read(analog_zero[0]);
        //dtostrf(Amps_TRMS, 2, 3, tempstr);
        //Serial.print( "\t Amps: " ); 
        //Serial.println( tempstr );
        //Serial.println("Sigma val (4 cal):" + String(inputStats.sigma()));
        //Serial.println("Sigma: " + String(inputStats.sigma()) + " Analog:" + String (ACS_Value)+ " RMS(sig): "+ String(Amps_TRMS) + " Corr_rms_dir: "+corr_rms);
      }
      //Serial.println(ACS_Value);

}//loop


double ac_read(int rZero) {
  int rVal = 0;
  int sampleDuration = 25;       // 100ms
  int sampleCount = 0;
  unsigned long rSquaredSum = 0;
  //int rZero = 511;                // For illustrative purposes only - should be measured to calibrate sensor.

  uint32_t startTime = millis();  // take samples for 100ms
  while((millis()-startTime) < sampleDuration)
  {
    rVal = analogRead(A0) - rZero;
    rSquaredSum += (rVal * rVal);//HACERL LO DEL FACTOR A NIVEL DE BITS!
    sampleCount++;
  }

  double voltRMS = 5.0 * sqrt(rSquaredSum / sampleCount) / 1024.0;

  // x 1000 to convert volts to millivolts
  // divide by the number of millivolts per amp to determine amps measured
  // the 20A module 100 mv/A (so in this case ampsRMS = 10 * voltRMS
  double ampsRMS = voltRMS * 10.0;
  return ampsRMS;
  Serial.println(ampsRMS);
}

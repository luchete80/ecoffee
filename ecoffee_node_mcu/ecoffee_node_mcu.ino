//https://github.com/mathworks/thingspeak-arduino

#include "src/ThingSpeak/ThingSpeak.h"
//#include <ESP8266WiFi.h>
#include "src/ESP8266WiFi/src/ESP8266WiFi.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include "defaults.h"

#define PIN_TEMP                  14    //GPIO14, PIN D5
#define PIN_HEAT                  12    //GPIO14, PIN D6
#define PIN_COOL                  13    //GPIO14, PIN D7
#define PIN_MIXER                  D8    //...

int  time_mixer_duration,time_mixer_interval,time_last_mix;
bool mixing;

//#include <TM1637Display.h>

// Define the connections pins:
#define CLK D2
#define DIO D3
// Create array that turns all segments on:
const uint8_t data[] = {0xff, 0xff, 0xff, 0xff};
#define TIME_SHOW_TEMP   3000
unsigned long last_show_temp;

// Create display object of type TM1637Display:
//TM1637Display display = TM1637Display(CLK, DIO);

OneWire           oneWire(PIN_TEMP);
DallasTemperature sensors(&oneWire);

//----------------  Fill in your credentails   ---------------------
char ssid[] = "Telecentro-81ff";             // your network SSID (name) 
char pass[] = "NJNXETMMTHNW";         // your network password
unsigned long myChannelNumber = 1;  // Replace the 0 with your channel number
const char * myWriteAPIKey = "MYRJ65U4RW6CDH3A";    // Paste your ThingSpeak Write API Key between the quotes 

unsigned long myReadChannel = 851892;
const char * myReadAPIKey = "K4CV8BK1NYCTCNKT";    // Paste your ThingSpeak Write API Key between the quotes 

//------------------------------------------------------------------
int x;
unsigned long showtime;
unsigned long current_time,readswitch_time;

unsigned short mode;
float set_temp;
WiFiClient  client;
bool heating,cooling;
float temp = 0;

void check_conn(){
    // Connect or reconnect to WiFi
    if(WiFi.status() != WL_CONNECTED){
        Serial.print("Attempting to connect to SSID: ");
        //Serial.println(SECRET_SSID);
        while(WiFi.status() != WL_CONNECTED){
            WiFi.begin(ssid, pass);
            Serial.print(".");
            delay(5000);     
        } 
          Serial.println("\nConnected.");
    }
  }
void setup() {
    //display.setBrightness(7);
  // All segments on:
  //display.setSegments(data);
  //display.clear();
  
	  //Initialize serial and wait for port to open:
	  Serial.begin(9600);
   Serial.println("HELLO");
	  while (!Serial) {
		; // wait for serial port to connect. Needed for native USB port only
	  }

	  WiFi.mode(WIFI_STA);
	  ThingSpeak.begin(client); 

    sensors.begin();
    pinMode(PIN_HEAT, OUTPUT);
    pinMode(PIN_MIXER, OUTPUT);
    pinMode(PIN_COOL, OUTPUT);
    digitalWrite(PIN_HEAT, HIGH);
    digitalWrite(PIN_COOL, HIGH);
    digitalWrite(PIN_MIXER,LOW);
    heating=cooling=false;
    current_time= readswitch_time = time_last_mix = millis();
    
    time_mixer_duration = 10000;
    time_mixer_interval = 500000;

    mixing = true;
    last_show_temp=millis();
}

void loop() {

  current_time = millis();
    if (current_time > readswitch_time + TIME_READING_SWITCHES){ //FIRST THING IS TO READ SWITCHES
        check_conn();
        //float readFloatField(unsigned long channelNumber, unsigned int field, const char * readAPIKey)
        set_temp=ThingSpeak.readFloatField(myReadChannel, 1, myReadAPIKey);
        readswitch_time = current_time;
      } else {
            if ( current_time > showtime + TIME_READNSHOW_TEMPS) {
                check_conn();

            	  // Write to ThingSpeak. There are up to 8 fields in a channel, allowing you to store up to 8 different
            	  // pieces of information in a channel.  Here, we write to field 1.
            	  //x = ThingSpeak.writeField(myChannelNumber, 1, sensors.getTempCByIndex(0), myWriteAPIKey);
                
                ThingSpeak.setField(1, sensors.getTempCByIndex(0));
                int field = 0;
                if      (heating) field  = 1;
                else if (cooling) field  =-1;
                ThingSpeak.setField(2, field);
                x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
               
                // Check the return code
                if(x == 200){
                    Serial.println("Channel update successful.");
                    }
                else{
                    Serial.println("Problem updating channel. HTTP error code " + String(x));
                }
        
               Serial.print("Temp: "); Serial.println(temp);

                showtime=millis();
            }//READNSHOW TEMPS
      }
	 if (temp > (set_temp - 0.3) && heating) {
        heating=false;
	      digitalWrite(PIN_HEAT, HIGH);
	    }

   if (temp < (set_temp - 0.3) && !heating) {
        heating=true;
        digitalWrite(PIN_HEAT, LOW);  //ACTIVATE  
    }

   if (temp < (set_temp + 0.3)) {
        if (cooling ){
          cooling=false;
          digitalWrite(PIN_COOL, HIGH);
        }
   } else { //TEMP < COOLING THRESHOLD 
        if (!cooling) {
            cooling=true;
            digitalWrite(PIN_COOL, LOW);  //ACTIVATE  
    }    
   }


    
    if (!mixing) {
        //Serial.print(" No mix");
    if (current_time > time_last_mix + (unsigned long)time_mixer_interval + (unsigned long)time_mixer_duration){
        time_last_mix = current_time;
        digitalWrite(PIN_MIXER,LOW);
        mixing=true;
        //Serial.println("Start");
        }
    } else { //ìf mixing
      //Serial.print ("Current time: ");Serial.printSerial.println(time_last_mix + (unsigned long) time_mixer_duration);
        if (current_time > time_last_mix + (unsigned long) time_mixer_duration) {
            digitalWrite(PIN_MIXER,HIGH);
            mixing=false;
          //Serial.println("Stop");
      }//time
    }//max time reached stop

    if (current_time > last_show_temp + TIME_SHOW_TEMP){
        sensors.requestTemperatures();
        temp=sensors.getTempCByIndex(0);
        //display.showNumberDecEx(temp*10, 0b11100000, false, 4, 0);
        //int pos=2;
        //display.showNumberDecEx(int (temp*10), (0x80 >> pos), false);
           Serial.println(temp);
        last_show_temp=current_time;
      }
	 
}

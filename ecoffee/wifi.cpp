//https://www.electronicshub.org/connect-esp8266-to-thingspeak
#include "wifi.h"

//#include <SoftwareSerial.h>       //Software Serial library
//SoftwareSerial Serial1(2, 3);   //Pin 2 and 3 act as RX and TX. Connect them to TX and RX of ESP8266      
#define DEBUG true


void idWifi::Init() {
  Serial1.begin(115200);
  
  espData("AT+RST", 3000, DEBUG);                      //Reset the ESP8266 module
  espData("AT+CWMODE=1", 3000, DEBUG);                 //Set the ESP mode as station mode
  espData("AT+CWJAP=\""+ mySSID +"\",\""+ myPWD +"\"", 3000, DEBUG);   //Connect to WiFi network
  /*while(!esp.find("OK")) 
  {          
      //Wait for connection
  }*/
  delay(1000);
  
}

void idWifi::Reconnect() {
	espData("AT+RST", 3000, DEBUG);                      //Reset the ESP8266 module
	espData("AT+CWMODE=1", 3000, DEBUG);                 //Set the ESP mode as station mode
	espData("AT+CWJAP=\""+ mySSID +"\",\""+ myPWD +"\"", 3000, DEBUG);   //Connect to WiFi network

}

// void loop()
// {
    // /* Here, I'm using the function random(range) to send a random value to the 
     // ThingSpeak API. You can change this value to any sensor data
     // so that the API will show the sensor data  
    // */
    
    // sendVal = random(1000); // Send a random number between 1 and 1000
    // String sendData = "GET /update?api_key="+ myAPI +"&"+ myFIELD +"="+String(sendVal);
    // espData("AT+CIPMUX=1", 5000, DEBUG);       //Allow multiple connections
    // espData("AT+CIPSTART=0,\"TCP\",\""+ myHOST +"\","+ myPORT, 5000, DEBUG);
    // espData("AT+CIPSEND=0," +String(sendData.length()+4),5000,DEBUG);  
    // Serial1.find(">"); 
    // Serial1.println(sendData);
    // Serial.print("Value to be sent: ");
    // Serial.println(sendVal);
     
    // espData("AT+CIPCLOSE=0",1000,DEBUG);
    // delay(20000);
// }

void idWifi::sendInt(const int &val){

    String sendData = "GET /update?api_key="+ myAPI +"&"+ myFIELD +"="+String(val);
    espData("AT+CIPMUX=1", 5000, DEBUG);       //Allow multiple connections
    espData("AT+CIPSTART=0,\"TCP\",\""+ myHOST +"\","+ myPORT, 5000, DEBUG);
    espData("AT+CIPSEND=0," +String(sendData.length()+4),5000,DEBUG);  
    Serial1.find(">"); 
    Serial1.println(sendData);
    Serial.print("Value to be sent: ");
    Serial.println(sendVal);
     
    espData("AT+CIPCLOSE=0",1000,DEBUG);
    delay(20000);
}

void idWifi::sendFloat(const float &val){

    String sendData = "GET /update?api_key="+ myAPI +"&"+ myFIELD +"="+String(val);
    espData("AT+CIPMUX=1", 5000, DEBUG);       //Allow multiple connections
    espData("AT+CIPSTART=0,\"TCP\",\""+ myHOST +"\","+ myPORT, 5000, DEBUG);
    espData("AT+CIPSEND=0," +String(sendData.length()+4),5000,DEBUG);  
    Serial1.find(">"); 
    Serial1.println(sendData);
    Serial.print("Value to be sent: ");
    Serial.println(sendVal);
     
    espData("AT+CIPCLOSE=0",50,DEBUG);
}

//void idWifi::sendFloatsfromField1(const float val[]){
//
//    String sendData = "GET /update?api_key="+ myAPI +"&"+ "field1" +"=";
//    for (int i=0;i<sizeof(val);i++)
//      sendData+=String(val[i]);
//    espData("AT+CIPMUX=1", 5000, DEBUG);       //Allow multiple connections
//    espData("AT+CIPSTART=0,\"TCP\",\""+ myHOST +"\","+ myPORT, 5000, DEBUG);
//    espData("AT+CIPSEND=0," +String(sendData.length()+4),5000,DEBUG);  
//    Serial1.find(">"); 
//    Serial1.println(sendData);
//    Serial.print("Value to be sent: ");
//    Serial.println(sendVal);
//     
//    espData("AT+CIPCLOSE=0",50,DEBUG);
//
//}

//String postStr = "api_key="+apiKeyChannel+"&field1="+str_sensor+"&field2="+str_sensor2;

String idWifi::espData(String command, const int timeout, boolean debug) {
	Serial.print("AT Command ==> ");
	Serial.print(command);
	Serial.println("     ");
	  
	String response = "";
	Serial1.println(command);
	long int time = millis();
	while ( (time + timeout) > millis() ) {
		while ( Serial1.available() ) {
		  char c = Serial1.read();
		  response += c;
		}
	  }
	if (debug) {
	Serial.print(response);
	}
	return response;
}

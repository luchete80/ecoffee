#ifndef _WIFI_H_
#define _WIFI_H_
#include <Arduino.h>

class idWifi {
	public:
		idWifi(){};
		~idWifi(){};
    void Init();
    void idWifi::Reconnect();
    String espData(String command, const int timeout, boolean debug);
    void sendFloat(const float &val);
    void sendInt(const int &val);

    private:
    String mySSID = "Telecentro-4300";       // WiFi SSID
    String myPWD = "tele-2848559"; // WiFi Password
    String myAPI = "P5MHE1FPIBNYJOHQ";   // WRITE API Key
    String myHOST = "api.thingspeak.com";
    String myPORT = "80";
    String myFIELD = "field1"; 
    int sendVal;

};
	

#endif

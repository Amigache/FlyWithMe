#ifndef Web_h
#define Web_h

#include "config.h"

// Load Wi-Fi library
#include <WiFi.h>

#include "FWM.h"

#include <esp_system.h>

class FWM;

class Web
{
public:
    Web(FWM *fwm);
    void begin();
    void startAP();
    void run();
    
    String getPostParam(String body, String param);
    String urlDecode(String input);

    bool server_up = false;

    IPAddress host_ip;
 
private:
    FWM *fwm;
};
#endif
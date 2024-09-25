#ifndef Screen_h
#define Screen_h

#include "config.h"

// Display libs
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "FWM.h"

class FWM;

class Screen
{
public:
    Screen(FWM *fwm);
    void begin();
    void run();
    void showCenterText(const char *text);
    void showServerData(const char* ssid, const char* password, IPAddress ip);

    boolean connected_screen = false;
    
private:
    Params_t params_data;
    FWM *fwm;
};
#endif
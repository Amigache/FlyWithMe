#ifndef FWM_H
#define FWM_H

#include "config.h"

#include "Comm.h"
#include "Telem.h"
#include "Web.h"
#include "Screen.h"

#include <Preferences.h>

#include <Ticker.h>

class Screen;
class Web;
class Comm;
class Telem;

class FWM
{
public:
    // FWM
    FWM();
    void begin();
    void run();
    void bridgeRun();

    static FWM* self;

    // Instances
    Comm *comm;
    Telem *mav;
    Screen *screen;
    Web *web;

    // Follow
    int stage_follow = STAGE_IDLE;
    int follow_mode = FOLL_MODE;
    void changeFollowMode(uint8_t mode);

    // Params
    void resetParams();
    bool existParams();
    void saveParams();
    void loadParams();

    // Variables
    Params_t params;

private:
    Preferences preferences;
    
    Ticker send_packet_ticker;
    static void send_packet_ticker_callback();

};

#endif // FWM_H

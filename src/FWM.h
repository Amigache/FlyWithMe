#ifndef FWM_H
#define FWM_H

#include "config.h"

#include "Comm.h"
#include "Telem.h"
#include "Web.h"
#include "Screen.h"

#include <Preferences.h>
#include <Ticker.h>
#include <esp_task_wdt.h>  // FASE 1: Watchdog

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

    // FASE 1: State Machine
    SystemState currentState = STATE_INIT;
    SystemState previousState = STATE_INIT;
    uint32_t stateEntryTime = 0; // A2: momento de entrada al estado actual (histéresis)
    void transitionState(SystemState newState);
    bool isValidStateTransition(SystemState from, SystemState to);
    void onStateEntry(SystemState state);
    const char* getStateName(SystemState state);
    
    // FASE 2: Control de flujo adaptativo y logging
    void updateTransmissionRate();
    float getDistanceToFollower();
    float getLinkDistance(); // A4: distancia al peer (-1 si no hay beacon)
    Logger* logger = nullptr;

    // Params
    void resetParams();
    bool existParams();
    void saveParams();
    void loadParams();

    // Config runtime (persistente en NVS) desde la web
    void setFormation(uint8_t idx);
    void setPrediction(bool on);
    void setFilter(bool on);

    // AP/WiFi solo en tierra (configuracion); nunca en vuelo
    bool isOnGround();
    void updateApGate();

    // Fase 2: tabla de parametros FWM (fuente de verdad)
    int paramCount();
    const ParamDef_t* paramDefAt(int idx);
    int findParam(const char* key);
    float getParamByIndex(int idx);
    bool setParamByIndex(int idx, float value, bool persist = true);
    bool setParamByKey(const char* key, const char* valueStr, String &err);
    String paramsJson();

    // Variables
    Params_t params;

private:
    Preferences preferences;
    
    Ticker send_packet_ticker;
    static void send_packet_ticker_callback();

};

#endif // FWM_H

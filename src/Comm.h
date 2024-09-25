#ifndef Comm_h
#define Comm_h

#include "config.h"
#include <cstdint>
#include <cstring>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

#include <Ticker.h>

#include "FWM.h"

class FWM;

class Comm
{
public:
    Comm(FWM *fwm);
    void begin();
    void run();

    static void beacon_ticker_callback();

    CommData_t commData;

    void sendPacket(LoraPacket_t packet);
    bool validateChecksum(LoraPacket_t packet);
    uint8_t calChecksum(LoraPacket_t packet);

    unsigned long time_lora_send = 0;
    unsigned long time_beacon = 0;
    unsigned long last_beacon = 0;

    static Comm* self;            // Puntero a la instancia

private:

    Ticker beacon_ticker;

    FWM *fwm;

};
#endif
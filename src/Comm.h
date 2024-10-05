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
    
    static Comm* self;            // Puntero a la instancia

    void bridgeRun();
    void receive_mavlink_lora();
    void send_mavlink_lora(mavlink_message_t message);

    static void beacon_ticker_callback();

    CommData_t commData;

    void sendPacket(LoraPacket_t packet);
    bool validateChecksum(LoraPacket_t packet);
    uint8_t calChecksum(LoraPacket_t packet);

    unsigned long time_lora_send = 0;
    unsigned long time_beacon = 0;
    unsigned long last_beacon = 0;

private:

    Ticker beacon_ticker;

    FWM *fwm;

};
#endif
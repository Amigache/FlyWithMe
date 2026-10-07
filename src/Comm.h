#ifndef Comm_h
#define Comm_h

#include "config.h"
#include <cstdint>
#include <cstring>

//Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>


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

    void markBeaconReceived();

    CommData_t commData;

    void sendPacket(const LoraPacket_t &packet);
    bool validateChecksum(const LoraPacket_t &packet);
    uint8_t calChecksum(const LoraPacket_t &packet);
    bool validatePacket(const LoraPacket_t &packet); // FASE 1: Validación completa de paquete
    bool acceptSequence(uint16_t seq);
    void applyNetid();                        // v2: re-fija el sync word de radio desde params.netid
    void sendReplyPacket();                   // v2: REPLY/JOIN del seguidor (enlace de vuelta)
    uint16_t txSeq = 0;                        // v2: secuencia de TX (perdidas/duplicados)
    uint16_t lastRxSeq = 0;                    // v2: ultima secuencia RX valida
    bool lastRxSeqInitialized = false;
    bool replyPending = false;
    uint32_t replyDueMs = 0;
    volatile bool syncWordPending = false;
    
    // FASE 2: Compresión y optimización
    CompressedLoraPacket_t compressPacket(LoraPacket_t packet);
    LoraPacket_t decompressPacket(CompressedLoraPacket_t compressed);
    bool sendPacketWithRetry(const LoraPacket_t &packet, uint8_t maxRetries = MAX_LORA_RETRIES);
    uint8_t calChecksumCompressed(CompressedLoraPacket_t packet);
    bool validateChecksumCompressed(CompressedLoraPacket_t packet);
    
    // FASE 3: Calibración automática
    void autoCalibrate();
    int measureAverageRSSI(int samples = 10);
    
    // FASE 4: Simulación
    #if SIMULATION_MODE
    uint32_t lastSimulatedPacket = 0;
    #endif

    unsigned long time_lora_send = 0;
    unsigned long time_beacon = 0;
    unsigned long last_beacon = 0;

private:

    FWM *fwm;

};
#endif

#ifndef Web_h
#define Web_h

#include "config.h"

// Load Wi-Fi library
#include <WiFi.h>

#include "FWM.h"

#include <esp_system.h>

#if USE_WEB_SERVER
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#endif

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
    
    bool ap_info_shown = false;  // Flag para mostrar info del AP solo una vez
    
    #if USE_WEB_SERVER
    void setupWebServer();
    void setupWebSocket();
    void handleRoot();
    void handleAPI();
    void handleGetConfig();
    void handleSetConfig();
    void handleGetStats();
    void handleGetLogs();
    void sendTelemetryWebSocket();
    String generateHTML();
    String generateAPIResponse(bool success, const char* message);
    #endif
 
private:
    FWM *fwm;
    
    #if USE_WEB_SERVER
    AsyncWebServer* server = nullptr;
    AsyncWebSocket* ws = nullptr;
    uint32_t lastWSBroadcast = 0;
    #endif
    
    uint32_t lastScreenUpdate = 0;  // Control de actualización de pantalla
};
#endif
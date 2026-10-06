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
    void bridgeRun();
    void showCenterText(const char *text);
    void showServerData(const char* ssid, const char* password, IPAddress ip);

    boolean connected_screen = false;
    
    // FASE 4: Sistema de menú interactivo
    #if USE_INTERACTIVE_MENU
    void initMenu();
    void updateMenu();
    void handleButtonPress();
    void showMenu();
    void showFormationMenu();
    void showSettingsMenu();
    void showDiagnosticsMenu();
    void showStatsScreen();
    MenuState currentMenuState = MENU_MAIN;
    int selectedOption = 0;
    int menuOffset = 0;
    SystemStats getSystemStats();
    #endif
    
private:
    FWM *fwm;
    
    #if USE_INTERACTIVE_MENU
    // Botones
    uint32_t lastButtonPress[4] = {0, 0, 0, 0};
    bool buttonState[4] = {false, false, false, false};
    
    // Opciones de menú
    static const int MAX_MENU_ITEMS = 8;
    MenuOption mainMenuOptions[MAX_MENU_ITEMS];
    MenuOption formationMenuOptions[MAX_MENU_ITEMS];
    MenuOption settingsMenuOptions[MAX_MENU_ITEMS];
    int mainMenuSize;
    int formationMenuSize;
    int settingsMenuSize;
    
    // Helpers
    bool isButtonPressed(int pin, int buttonIndex);
    void executeMenuItem(MenuItem item);
    #endif
};
#endif
#include "Screen.h"

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

FlightModeInfo flightModes[] = {
    {0, "Manual"},
    {1, "CIRCLE"},
    {2, "STABILIZE"},
    {3, "TRAINING"},
    {4, "ACRO"},
    {5, "FBWA"},
    {6, "FBWB"},
    {7, "CRUISE"},
    {8, "AUTOTUNE"},
    {10, "Auto"},
    {11, "RTL"},
    {12, "Loiter"},
    {13, "TAKEOFF"},
    {14, "AVOID_ADSB"},
    {15, "Guided"},
    {17, "QSTABILIZE"},
    {18, "QHOVER"},
    {19, "QLOITER"},
    {20, "QLAND"},
    {21, "QRTL"},
    {22, "QAUTOTUNE"},
    {23, "QACRO"},
    {24, "THERMAL"},
    {25, "Loiter to QLand"}};

Screen::Screen(FWM *fwm)
{
  this->fwm = fwm;
  
  #if USE_INTERACTIVE_MENU
  // Inicializar menú principal
  mainMenuOptions[0] = {"Formacion", ITEM_FORMATION_TYPE};
  mainMenuOptions[1] = {"Configuracion", ITEM_FILTER_TOGGLE};
  mainMenuOptions[2] = {"Diagnostico", ITEM_VIEW_STATS};
  mainMenuOptions[3] = {"Calibrar LoRa", ITEM_CALIBRATE_LORA};
  mainMenuSize = 4;
  
  // Inicializar menú de formación
  formationMenuOptions[0] = {"Trail", ITEM_FORMATION_TYPE};
  formationMenuOptions[1] = {"Left", ITEM_FORMATION_TYPE};
  formationMenuOptions[2] = {"Right", ITEM_FORMATION_TYPE};
  formationMenuOptions[3] = {"Above", ITEM_FORMATION_TYPE};
  formationMenuOptions[4] = {"Below", ITEM_FORMATION_TYPE};
  formationMenuOptions[5] = {"Distancia", ITEM_FORMATION_DISTANCE};
  formationMenuOptions[6] = {"Volver", ITEM_BACK};
  formationMenuSize = 7;
  
  // Inicializar menú de configuración
  settingsMenuOptions[0] = {"Prediccion", ITEM_PREDICTION_TOGGLE};
  settingsMenuOptions[1] = {"Filtro", ITEM_FILTER_TOGGLE};
  settingsMenuOptions[2] = {"Alpha Filtro", ITEM_FILTER_ALPHA};
  settingsMenuOptions[3] = {"Tasa Adapt.", ITEM_ADAPTIVE_RATE};
  settingsMenuOptions[4] = {"Compresion", ITEM_COMPRESSION};
  settingsMenuOptions[5] = {"Volver", ITEM_BACK};
  settingsMenuSize = 6;
  #endif
}

void Screen::begin()
{

  // WIRE --------------------------------------------------------------------------------------------------
  Log.notice("Init I2C for Display" CR);
  Wire.begin(OLED_SDA, OLED_SCL);
  delay(1000);
  Log.notice("I2C for Display Ready" CR);

  // DISPLAY -----------------------------------------------------------------------------------------------
  Log.notice("Init Display" CR);

  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c, false, false))
  { // Address 0x3C for 128x32
    Log.error("SSD1306 allocation failed" CR);
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.display(); // Clear the display buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  showCenterText(VERSION);

  delay(2000);
  Log.notice("Display Ready" CR);
  
  #if USE_INTERACTIVE_MENU
  initMenu();
  #endif
}

#if USE_INTERACTIVE_MENU
void Screen::initMenu()
{
  // Inicializar pines de botones
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BACK_PIN, INPUT_PULLUP);
  
  Log.notice("Menu interactivo inicializado" CR);
}

bool Screen::isButtonPressed(int pin, int buttonIndex)
{
  bool currentState = (digitalRead(pin) == LOW);
  uint32_t now = millis();
  
  // Anti-rebote
  if (currentState && !buttonState[buttonIndex])
  {
    if (now - lastButtonPress[buttonIndex] > BUTTON_DEBOUNCE_MS)
    {
      lastButtonPress[buttonIndex] = now;
      buttonState[buttonIndex] = true;
      return true;
    }
  }
  else if (!currentState)
  {
    buttonState[buttonIndex] = false;
  }
  
  return false;
}

void Screen::handleButtonPress()
{
  // Botón UP
  if (isButtonPressed(BUTTON_UP_PIN, 0))
  {
    if (selectedOption > 0) {
      selectedOption--;
    }
  }
  
  // Botón DOWN
  if (isButtonPressed(BUTTON_DOWN_PIN, 1))
  {
    int maxOptions = mainMenuSize;
    if (currentMenuState == MENU_FORMATION) maxOptions = formationMenuSize;
    if (currentMenuState == MENU_SETTINGS) maxOptions = settingsMenuSize;
    
    if (selectedOption < maxOptions - 1) {
      selectedOption++;
    }
  }
  
  // Botón SELECT
  if (isButtonPressed(BUTTON_SELECT_PIN, 2))
  {
    if (currentMenuState == MENU_MAIN) {
      executeMenuItem(mainMenuOptions[selectedOption].item);
    }
    else if (currentMenuState == MENU_FORMATION) {
      executeMenuItem(formationMenuOptions[selectedOption].item);
    }
    else if (currentMenuState == MENU_SETTINGS) {
      executeMenuItem(settingsMenuOptions[selectedOption].item);
    }
  }
  
  // Botón BACK
  if (isButtonPressed(BUTTON_BACK_PIN, 3))
  {
    if (currentMenuState != MENU_MAIN) {
      currentMenuState = MENU_MAIN;
      selectedOption = 0;
    }
  }
}

void Screen::executeMenuItem(MenuItem item)
{
  switch(item) {
    case ITEM_FORMATION_TYPE:
      currentMenuState = MENU_FORMATION;
      selectedOption = 0;
      break;
      
    case ITEM_FILTER_TOGGLE:
      currentMenuState = MENU_SETTINGS;
      selectedOption = 0;
      break;
      
    case ITEM_VIEW_STATS:
      currentMenuState = MENU_DIAGNOSTICS;
      selectedOption = 0;
      break;
      
    case ITEM_CALIBRATE_LORA:
      #if AUTO_CALIBRATE_LORA
      showCenterText("Calibrando...");
      fwm->comm->autoCalibrate();
      delay(2000);
      #endif
      break;
      
    case ITEM_BACK:
      currentMenuState = MENU_MAIN;
      selectedOption = 0;
      break;
      
    default:
      break;
  }
}

void Screen::showMenu()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  // Título según menú actual
  const char* title = "MENU PRINCIPAL";
  if (currentMenuState == MENU_FORMATION) title = "FORMACION";
  if (currentMenuState == MENU_SETTINGS) title = "CONFIG";
  if (currentMenuState == MENU_DIAGNOSTICS) title = "DIAGNOSTICO";
  
  display.println(title);
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  // Mostrar opciones
  MenuOption* options = mainMenuOptions;
  int optionCount = mainMenuSize;
  
  if (currentMenuState == MENU_FORMATION) {
    options = formationMenuOptions;
    optionCount = formationMenuSize;
  } else if (currentMenuState == MENU_SETTINGS) {
    options = settingsMenuOptions;
    optionCount = settingsMenuSize;
  }
  
  // Calcular offset para scroll
  const int maxVisible = 3;
  if (selectedOption >= menuOffset + maxVisible) {
    menuOffset = selectedOption - maxVisible + 1;
  } else if (selectedOption < menuOffset) {
    menuOffset = selectedOption;
  }
  
  // Mostrar opciones visibles
  for (int i = 0; i < maxVisible && (menuOffset + i) < optionCount; i++) {
    int index = menuOffset + i;
    display.setCursor(5, 15 + (i * 10));
    
    if (index == selectedOption) {
      display.print(">");
    } else {
      display.print(" ");
    }
    
    display.print(options[index].label);
  }
  
  // Indicador de scroll
  if (menuOffset > 0) {
    display.setCursor(120, 15);
    display.print("^");
  }
  if (menuOffset + maxVisible < optionCount) {
    display.setCursor(120, 35);
    display.print("v");
  }
  
  display.display();
}

void Screen::showStatsScreen()
{
  SystemStats stats = getSystemStats();
  
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  display.println("ESTADISTICAS");
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  display.setCursor(0, 13);
  
  display.print("Uptime: ");
  display.print(stats.uptime / 1000);
  display.println("s");
  
  display.print("RX:");
  display.print(stats.totalPacketsRx);
  display.print(" TX:");
  display.println(stats.totalPacketsTx);
  
  display.print("Lost:");
  display.print(stats.packetsLost);
  display.print(" (");
  display.print(stats.packetLossRate, 1);
  display.println("%)");
  
  display.print("RSSI:");
  display.print(stats.avgRSSI);
  display.print(" SNR:");
  display.println(stats.avgSNR);
  
  display.print("Dist: ");
  if (stats.linkDistance < 0) {
    display.println("--");
  } else {
    display.print(stats.linkDistance);
    display.println(" m");
  }
  
  display.display();
}

SystemStats Screen::getSystemStats()
{
  SystemStats stats;
  stats.uptime = millis();
  stats.totalPacketsRx = fwm->comm->commData.rx_packet_counter;
  stats.totalPacketsTx = fwm->comm->commData.tx_packet_counter;
  stats.packetsLost = fwm->comm->commData.lost_packet_counter;
  
  if (stats.totalPacketsRx > 0) {
    stats.packetLossRate = (stats.packetsLost * 100.0f) / (stats.totalPacketsRx + stats.packetsLost);
  } else {
    stats.packetLossRate = 0.0f;
  }
  
  stats.avgRSSI = fwm->comm->commData.rssi;
  stats.avgSNR = fwm->comm->commData.snr;          // A4
  stats.linkDistance = (int)fwm->getLinkDistance(); // A4
  stats.totalDistance = 0; // TODO: calcular distancia total recorrida
  stats.stateChanges = 0;
  stats.safetyViolations = 0;
  
  return stats;
}

void Screen::updateMenu()
{
  handleButtonPress();
  
  if (currentMenuState == MENU_DIAGNOSTICS) {
    showStatsScreen();
  } else {
    showMenu();
  }
}
#endif

void Screen::run()
{
  #if USE_INTERACTIVE_MENU
  // Comprobar si algún botón está presionado para entrar al menú
  if (digitalRead(BUTTON_SELECT_PIN) == LOW && 
      millis() - lastButtonPress[2] > 2000) {
    // Mantener SELECT por 2s entra al menú
    while(true) {
      updateMenu();
      
      // Salir del menú si se mantiene BACK presionado
      if (digitalRead(BUTTON_BACK_PIN) == LOW) {
        delay(500);
        if (digitalRead(BUTTON_BACK_PIN) == LOW) {
          currentMenuState = MENU_MAIN;
          selectedOption = 0;
          break;
        }
      }
      
      delay(50);
    }
    lastButtonPress[2] = millis();
  }
  #endif
  
  // buscamos el modo de vuelo
  String flightModeName = "Unknown";
  for (int i = 0; i < sizeof(flightModes) / sizeof(flightModes[0]); i++)
  {
    if (flightModes[i].mode == fwm->mav->APdata.custom_mode)
    {
      flightModeName = flightModes[i].name.c_str();
      break;
    }
  }

  // Check if we are connected to the FC
  if (fwm->mav->link && !connected_screen)
  {
    showCenterText("FC Connected");
    connected_screen = true;
    delay(1000);
  }
  else if (fwm->mav->is_connecting)
  {
    showCenterText("Connecting to FC");
    connected_screen = false;
  }
  else if (connected_screen)
  {
    int cursorY = 8;     // Tamaño de la fuente predeterminada en altura es 8 píxeles
    int lineSpacing = 2; // Pequeño margen debajo del texto

    if (fwm->follow_mode != FOLL_MODE_OFF)
    {

      display.clearDisplay();
      display.setCursor(0, 0);

      if (fwm->follow_mode == FOLL_MODE_FOLLOWER)
      {
        // La banda va en la cabecera y no como linea mas: el follower ya llena la pantalla hasta
        // la ultima linea, y la banda es justo lo que hay que comparar entre dos placas cuando
        // no hay enlace por desajuste.
        display.print("MODE FOLLOWER ");
        display.println(loraBandLabel(fwm->params.band));
        display.drawLine(0, cursorY + lineSpacing, 128, cursorY + lineSpacing, SSD1306_WHITE);
        display.setCursor(0, cursorY + lineSpacing + 5); // Mover el cursor debajo de la línea
        display.print("Rx: ");
        display.println(fwm->comm->commData.rx_packet_counter);
        display.print("Lost: ");
        display.println(fwm->comm->commData.lost_packet_counter);
        display.print("RSSI: ");
        display.println(fwm->comm->commData.rssi);
        display.print("Size: ");
        display.println((unsigned int)fwm->comm->commData.lastValidPacketSize);
        display.print("Mode: ");
        display.println(flightModeName);
        display.print("Dist: ");
        display.println(fwm->mav->APdata.wp_dist);
      }

      if (fwm->follow_mode == FOLL_MODE_LEADER)
      {
        display.print("MODE LEADER ");
        display.println(loraBandLabel(fwm->params.band));
        display.drawLine(0, cursorY + lineSpacing, 128, cursorY + lineSpacing, SSD1306_WHITE);
        display.setCursor(0, cursorY + lineSpacing + 5); // Mover el cursor debajo de la línea
        display.print("Tx: ");
        display.println(fwm->comm->commData.tx_packet_counter);
        display.print("Mode: ");
        display.println(flightModeName);
      }

      display.display();
    }
    else
    {
      showCenterText("FOLLOW MODE OFF");
    }
  }
}

void Screen::bridgeRun()
{
  int cursorY = 8;     // Tamaño de la fuente predeterminada en altura es 8 píxeles
  int lineSpacing = 2; // Pequeño margen debajo del texto

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MODE BRIDGE");
  display.drawLine(0, cursorY + lineSpacing, 128, cursorY + lineSpacing, SSD1306_WHITE);
  display.setCursor(0, cursorY + lineSpacing + 5); // Mover el cursor debajo de la línea
  display.print("RSSI: ");
  display.println(fwm->comm->commData.rssi);
  display.print("SNR: ");
  display.println(fwm->comm->commData.snr);
  display.display();
}

void Screen::showCenterText(const char *text)
{
  display.clearDisplay();
  display.setCursor(0, 0);

  // Calculate the position to center the text
  int16_t x = (SCREEN_WIDTH - (strlen(text) * 6)) / 2; // Each character is approximately 6 pixels wide
  int16_t y = (SCREEN_HEIGHT - display.getCursorY()) / 2;

  display.setCursor(x, y);
  display.println(text);
  display.display();
}

void Screen::showServerData(const char *ssid, const char *password, IPAddress ip)
{
  int cursorY = 8;     // Tamaño de la fuente predeterminada en altura es 8 píxeles
  int lineSpacing = 2; // Pequeño margen debajo del texto

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("AP MODE");
  display.drawLine(0, cursorY + lineSpacing, 128, cursorY + lineSpacing, SSD1306_WHITE);
  display.setCursor(0, cursorY + lineSpacing + 5); // Mover el cursor debajo de la línea
  display.print("ssid: ");
  display.println(ssid);
  display.print("pass: ");
  display.println(password);
  display.print("ip:   ");
  display.println(ip);
  display.display();
}
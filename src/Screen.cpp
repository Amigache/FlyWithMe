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
}

void Screen::run()
{
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
        display.println("MODE FOLLOWER");
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
        display.println("MODE LEADER");
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
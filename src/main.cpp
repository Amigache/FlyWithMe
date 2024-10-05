#include "FWM.h"

// FWM
FWM fwm;

void setup()
{

  // LED ---------------------------------------------------------------------------------------------------
  pinMode(LED_BUILTIN, OUTPUT);

  // SERIALS -----------------------------------------------------------------------------------------------

  // Debug Serial
  Serial.begin(SERIAL_BAUD, SERIAL_8N1);

  delay(1000);
  Log.notice("Monitor Serial Ready" CR);

// Log helper
#ifdef DEBUG_MODE
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#else
  Log.begin(LOG_LEVEL_SILENT, &Serial);
#endif

  // FWM ---------------------------------------------------------------------------------------------------
  fwm.begin();
}

void loop()
{
  if (MAV_BRIDGE)
  {
    fwm.bridgeRun();
  }
  else
  {
    fwm.run();
  }
}

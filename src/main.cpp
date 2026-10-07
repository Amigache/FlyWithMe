#include "FWM.h"
#if FWM_SELFTEST
#include "selftest.h"
#endif

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
#if FWM_SELFTEST
  {
    char msg[96];
    int fails = protocolSelfTest(msg, sizeof(msg));
    Log.notice("SELFTEST %s: %s" CR, fails ? "FAIL" : "PASS", msg);
  }
#endif
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

#include "FWM.h"
#if FWM_SELFTEST
#include "selftest.h"
#endif
#if FWM_DUAL_CORE
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

// FWM
FWM fwm;

#if FWM_DUAL_CORE
// Tarea de UI/logging en el CORE 0 (no bloquea el camino critico de vuelo del core 1).
static void fwm_io_task(void *)
{
  for (;;)
  {
    fwm.runIo();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
#endif

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

#if FWM_DUAL_CORE && !MAV_BRIDGE
  // Core 0: UI/logging. El loop() de Arduino sigue en el core 1 con el camino critico.
  xTaskCreatePinnedToCore(fwm_io_task, "fwm_io", 8192, nullptr, 1, nullptr, 0);
  Log.notice("Dual-core: vuelo en core 1, UI/log en core 0" CR);
#endif
}

void loop()
{
  if (MAV_BRIDGE)
  {
    fwm.bridgeRun();
  }
#if FWM_DUAL_CORE
  else
  {
    fwm.runRt(); // core 1 (loopTask): LoRa + MAVLink + FSM
  }
#else
  else
  {
    fwm.run();
  }
#endif
}

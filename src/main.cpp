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

// Log helper
#ifdef DEBUG_MODE
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#else
  Log.begin(LOG_LEVEL_SILENT, &Serial);
#endif

  delay(1000);
  Log.notice("Monitor Serial Ready" CR);

  // FWM ---------------------------------------------------------------------------------------------------
  fwm.begin();

}

void loop()
{

  // Only work if we are on follower mode
  if (fwm.follow_mode == FOLL_MODE_FOLLOWER)
  {
    if (fwm.mav->APdata.custom_mode == MODE_GUIDED)
    {

      // Start approach we have beacon
      if (fwm.comm->commData.have_beacon && fwm.stage_follow == STAGE_IDLE)
      {

        Log.info("Beacon OK, start approach" CR);

        fwm.stage_follow = STAGE_APPROACH;

        // Stop approach beacuse beacon lost
      }
      else if (!fwm.comm->commData.have_beacon && fwm.stage_follow == STAGE_APPROACH)
      {

        Log.info("Beacon LOST, stop follow" CR);

        fwm.stage_follow = STAGE_IDLE;
      }

      // Stop approach beacuse mode changed
    }
    else if (fwm.mav->APdata.custom_mode != MODE_GUIDED && fwm.stage_follow == STAGE_APPROACH)
    {

      Log.info("Stop follow, mode changed" CR);

      fwm.stage_follow = STAGE_IDLE;
    }
    else
    {
      fwm.stage_follow = STAGE_IDLE;
    }
  }else{
    fwm.stage_follow = STAGE_IDLE;
  }

  fwm.run();

}

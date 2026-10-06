#ifndef Telem_h
#define Telem_h

#include "config.h"

#include "FWM.h"

#include <Ticker.h>

class FWM;

class Telem
{
public:
  Telem(FWM *fwm);
  void begin();
  void run();

  static Telem* self;

  void bridgeRun();
  void receive_mavlink_serial();

  void send_to_fc(mavlink_message_t msg);
  void heartbeat(uint8_t system_id, uint8_t component_id, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status);
  void status_text(const char * text);
  void request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop);
  void do_change_speed(uint16_t speed);
  void do_reposition(int32_t lat, int32_t lon, float alt, uint16_t hdg);
  void nav_waypoint(int32_t lat, int32_t lon, int32_t alt);
  void request_param_value(std::string param_name);
  void set_param_value(std::string param_name, float param_value);

  static void heartbeat_ticker_callback();

  uint16_t calculate_dynamic_speed(float leader_speed, float distance);
  
  // FASE 1: Límites de seguridad
  float calculateDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
  bool isSafeToFollow(LoraPacket_t leaderData);
  
  // FASE 3: Predicción y formaciones
  PredictedPosition predictLeaderPosition(LoraPacket_t current, uint32_t futureTime);
  void calculateFormationPosition(LoraPacket_t leader, FormationType formation, 
                                   int32_t &targetLat, int32_t &targetLon, int32_t &targetAlt);
  const char* getFormationName(FormationType formation);
  
  // FASE 3: Variables de formación y filtrado
  FormationType currentFormation = (FormationType)DEFAULT_FORMATION;
  PositionFilter positionFilter;
  PredictedPosition lastPrediction;
  
  // FASE 4: Simulación
  #if SIMULATION_MODE
  SimulatedData simulatedData;
  void initSimulation();
  void updateSimulation();
  LoraPacket_t getSimulatedPacket();
  #endif

  APdata_t APdata;
  boolean link = false;
  boolean led_status = false;
  int linkTryTime = 0; 
  boolean linkTimeout = false;
  boolean lock_ap = false;
  boolean is_connecting = false;

  HardwareSerial SerialPort;

private:
  void check_link();
  void init_setup();

  int listTimeOutPointer = 0;

  // unsigned long time_heartbeat = 0;
  unsigned long last_heartbeat = 0;
  // unsigned long timeout_count = 0;

  bool init_setup_done = false;

  Ticker heartbeat_ticker;

  FWM *fwm;

};
#endif
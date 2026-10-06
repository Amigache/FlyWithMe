#ifndef Config_H
#define Config_H

#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include <string>  // FlightModeInfo usa std::string

// For mavlink sha256 redefinition error
#ifdef F
#undef F
#endif

#include "../lib/mavlink/common/mavlink.h"

#define VERSION "FlyWithMe V1.0"

// DEBUG MODE
#define DEBUG_MODE // Comentar para desactivar debug

// Data Streams
#define MAV_DATA_STREAM_POSITION_RATE 0x02       ///< 2 Hz
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 0x02 ///< 2 Hz

// Params setup
#define AUTO_SET_FOLL_PARAMS 1 ///< 1 to automatically set params, 0 otherwise

// Intervals
#define HEARTBEAT_INTERVAL 1000 ///< ms 1 vez por segundo
#define BEACON_CHECK_INTERVAL 1000 ///< ms 1 vez por segundo
#define SEND_PACKET_INTERVAL 1000
#define LINK_METRICS_LOG_INTERVAL_MS 5000 ///< A4: periodo de log de métricas de enlace

// OTHER config ------------------------------------------------------------------------------------------

// Serial Bauds
#define SERIAL_BAUD 57600
#define SERIAL_BAUD_TELEM 57600

// Time to lost link
#define LOST_TIME 10        // s
#define LOST_TIME_BEACON 10 // s

// Flight modes
#define MODE_GUIDED 15

// Web Server
#define WEB_PORT 80

// MavBridge
#define MAV_BRIDGE 0

// BOARD config ------------------------------------------------------------------------------------------

// TTGO LORA32 V1.0

// Oled
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16
#define SCREEN_WIDTH 128 ///< OLED display width, in pixels
#define SCREEN_HEIGHT 64 ///< OLED display height, in pixels

// Serial
#define SERIAL1_RX 12
#define SERIAL1_TX 13

// Lora
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define LORA_SIGNAL_BANDWIDTH 125000 ///< 125kHz
#define LORA_SPREADING_FACTOR 12     ///< SF12
#define LORA_CODING_RATE 5           ///< 4/5
#define LORA_TX_POWER 20             ///< 20dBm
#define LORA_SYNC_WORD 0x34          ///< 0x34

// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define LORA_BAND 866E6

// Stages ------------------------------------------------------------------------------------------------
#define STAGE_IDLE 0
#define STAGE_APPROACH 1
#define STAGE_HOLDPOS 2

// Follow modes --------------------------------------------------------------------------------------------
#define FOLL_MODE_OFF 0
#define FOLL_MODE_FOLLOWER 1
#define FOLL_MODE_LEADER 2

// SAFETY LIMITS (FASE 1 - Seguridad Crítica) -----------------------------------------------------------
#define MAX_FOLLOW_DISTANCE 5000  // metros - 5km máximo de distancia de seguimiento
#define MAX_FOLLOW_SPEED 5000     // cm/s - 50 m/s máximo de velocidad
#ifndef MIN_SAFE_ALTITUDE
#define MIN_SAFE_ALTITUDE 50000   // mm - 50m mínimo sobre terreno
#endif
#define EMERGENCY_RECOVERY_MS 5000 // ms - espera antes de reintentar tras una emergencia (histéresis)
#define MAX_VALID_LATITUDE 900000000  // Lat máxima válida (* 1E7)
#define MAX_VALID_LONGITUDE 1800000000 // Lon máxima válida (* 1E7)
#define MIN_VALID_ALTITUDE -1000000    // Altitud mínima válida (mm)
#define MAX_VALID_ALTITUDE 50000000    // Altitud máxima válida (mm) - 50km
#define MAX_VALID_GROUND_SPEED 30000   // cm/s - 300 m/s máximo

// ADAPTIVE COMMUNICATION (FASE 2 - Optimización) -------------------------------------------------------
#define USE_COMPRESSED_PACKETS 1       // 1 = usar paquetes comprimidos, 0 = usar paquetes normales
#define ADAPTIVE_RATE 1                // 1 = tasa adaptativa, 0 = tasa fija
#define PACKET_RATE_CLOSE 2000         // ms - 0.5 Hz cuando está cerca (< 100m)
#define PACKET_RATE_MEDIUM 1000        // ms - 1 Hz distancia media (100-500m)
#define PACKET_RATE_FAR 500            // ms - 2 Hz cuando está lejos (> 500m)
#define DISTANCE_THRESHOLD_CLOSE 100   // metros
#define DISTANCE_THRESHOLD_MEDIUM 500  // metros
#define MAX_LORA_RETRIES 3             // Intentos máximos de retransmisión
#define LORA_RETRY_DELAY_MIN 10        // ms - Delay mínimo para retry
#define LORA_RETRY_DELAY_MAX 100       // ms - Delay máximo para retry

// ADVANCED FOLLOWING (FASE 3 - Mejoras de Seguimiento) -------------------------------------------------
#define USE_PREDICTION 1               // 1 = usar predicción de movimiento, 0 = desactivado
#define PREDICTION_TIME_MS 1000        // ms - Tiempo de predicción adelantado
#define USE_POSITION_FILTER 1          // 1 = usar filtro de posición, 0 = desactivado
#define POSITION_FILTER_ALPHA 0.7      // 0.0-1.0 - Factor de filtro (mayor = más rápido)
#define DEFAULT_FORMATION 0            // 0=TRAIL, 1=LEFT, 2=RIGHT, 3=ABOVE, 4=BELOW
#define FORMATION_LATERAL_OFFSET 50    // metros - Offset lateral para formaciones
#define FORMATION_VERTICAL_OFFSET 20   // metros - Offset vertical para formaciones
#define AUTO_CALIBRATE_LORA 0          // 1 = calibrar automáticamente al inicio, 0 = manual

// DEFAULT PARAMS
#define FOLL_ENABLE 1
#define FOLL_OFS_TYPE 1
#define FOLL_ALT_TYPE 0
#define LINK_TIMEOUT 10
#define DEFAULT_SSID "FWM AP 1"
#define DEFAULT_PASS "12345678"
#define FOLL_MODE FOLL_MODE_OFF
#define FOLL_MODE_CH 7
#define ALT_OFFSET 10 // m
#define SPEED_OFFSET 20 // %
#define DIST_OFFSET 100 //m

// Set by target
#ifdef MASTER_BUILD_FLAG
#ifndef TARGET_SYSID
#define TARGET_SYSID 1     ///< Pixhawk (or any other autopilot)
#endif
#define TARGET_COMPID 1    ///< Component
#ifndef SYSID
#define SYSID TARGET_SYSID ///< ID 20 for this airplane. 1 PX, 255 ground station
#endif
#define COMPID 158         ///< The component sending the message
#define DEFAULT_SSID "FWM AP 1"
#define DEFAULT_PASS "12345678"
#define FOLL_MODE FOLL_MODE_LEADER
#define FOLL_MODE_CH 0
#endif

#ifdef SLAVE_BUILD_FLAG
#ifndef TARGET_SYSID
#define TARGET_SYSID 2     ///< Pixhawk (or any other autopilot)
#endif
#define TARGET_COMPID 1    ///< Component
#ifndef SYSID
#define SYSID TARGET_SYSID ///< ID 20 for this airplane. 1 PX, 255 ground station
#endif
#define COMPID 158         ///< The component sending the message
#define DEFAULT_SSID "FWM AP 2"
#define DEFAULT_PASS "12345678"
#define FOLL_MODE FOLL_MODE_FOLLOWER
#define FOLL_MODE_CH 0
#endif

typedef struct
{
  int32_t foll_enable;
  int32_t foll_ofs_type;
  int32_t foll_alt_type;
  int32_t link_timeout;
  char ssid[11];
  char pass[11];
} Params_t;

typedef struct
{
  uint32_t custom_mode;    ///< A bitfield for use for autopilot-specific flags
  uint8_t type;            ///< Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.*/
  uint8_t autopilot;       ///< Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.*/
  uint8_t base_mode;       ///< System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
  uint8_t system_status;   ///< System status flag.*/
  uint8_t mavlink_version; ///< MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version*/
  int32_t lat;             ///< Latitude, expressed as * 1E7
  int32_t lon;             ///< Longitude, expressed as * 1E7
  int32_t alt;             ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
  int32_t relative_alt;    ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
  int16_t vx;              ///< [cm/s] Ground X Speed (Latitude, positive north)*/
  int16_t vy;              ///< [cm/s] Ground Y Speed (Longitude, positive east)*/
  int16_t vz;              ///< [cm/s] Ground Z Speed (Altitude, positive down)*/
  uint16_t ground_speed;   ///< [cm/s] Ground speed;
  uint16_t hdg;            ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
  uint16_t wp_dist;        ///< [cm] Distance to active waypoint, 0 if no active waypoint
  uint8_t armed;           ///< System armed status
} APdata_t;

typedef struct
{
  uint8_t sysid;         ///< ID System
  int32_t lat;           ///< Latitude, expressed as * 1E7
  int32_t lon;           ///< Longitude, expressed as * 1E7
  int32_t alt;           ///< Altitude in meters, expressed as * 1000 (millimeters), above MSL
  int32_t relative_alt;  ///< Altitude above ground in meters, expressed as * 1000 (millimeters)
  uint16_t ground_speed; ///< [cm/s] Ground speed;
  uint16_t hdg;          ///< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
  uint8_t checksum;      ///< Checksum
} LoraPacket_t;

// FASE 2: Estructura de paquete comprimido (15 bytes vs 27 bytes)
typedef struct __attribute__((packed))
{
  uint8_t sysid;              ///< ID System (1 byte)
  int8_t lat_deg;             ///< Latitude degrees -90 to 90 (1 byte)
  uint8_t lat_frac;           ///< Latitude fractional part 0-255 -> 0.0-0.9999 (1 byte)
  int16_t lon_deg;            ///< Longitude degrees -180 to 180 (2 bytes)
  uint8_t lon_frac;           ///< Longitude fractional part 0-255 -> 0.0-0.9999 (1 byte)
  uint16_t relative_alt_dm;   ///< Relative altitude in decimeters (2 bytes) -> 0-6553.5m
  uint16_t ground_speed;      ///< [cm/s] Ground speed (2 bytes)
  uint16_t hdg;               ///< Heading in degrees * 100 (2 bytes)
  uint8_t flags;              ///< Flags: bit 0=alt_valid, bits 1-7=reserved (1 byte)
  uint8_t checksum;           ///< Checksum (1 byte)
} CompressedLoraPacket_t;   // Total: 15 bytes

typedef struct
{
  unsigned long tx_packet_counter = 0;
  unsigned long rx_packet_counter = 0;
  unsigned long lost_packet_counter = 0;
  uint16_t lastValidPacketSize = 0;
  LoraPacket_t lastValidPacket;
  bool have_beacon = false;
  int rssi = 0;
  int snr = 0;
} CommData_t;

// Struct que contiene el modo de vuelo y su nombre asociado
struct FlightModeInfo
{
  int32_t mode;
  std::string name;
};

// FASE 3: Tipos de Formación -----------------------------------------------------------------
enum FormationType
{
  FORMATION_TRAIL = 0,     // Detrás del líder
  FORMATION_LEFT = 1,      // A la izquierda
  FORMATION_RIGHT = 2,     // A la derecha
  FORMATION_ABOVE = 3,     // Arriba
  FORMATION_BELOW = 4      // Abajo
};

// FASE 3: Estructura de Posición Predicha ----------------------------------------------------
struct PredictedPosition
{
  int32_t lat;           // Latitud predicha (* 1E7)
  int32_t lon;           // Longitud predicha (* 1E7)
  int32_t alt;           // Altitud predicha (mm)
  uint32_t timestamp;    // Timestamp de la predicción
  float confidence;      // Confianza de la predicción (0.0-1.0)
};

// FASE 3: Filtro de Posición (Paso Bajo) -----------------------------------------------------
class PositionFilter
{
private:
  float alpha;           // Factor de suavizado (0.0-1.0)
  int32_t filteredLat;   // Latitud filtrada
  int32_t filteredLon;   // Longitud filtrada
  int32_t filteredAlt;   // Altitud filtrada
  bool initialized;      // Si el filtro ha sido inicializado
  
public:
  PositionFilter(float filterAlpha = POSITION_FILTER_ALPHA) 
    : alpha(filterAlpha), filteredLat(0), filteredLon(0), filteredAlt(0), initialized(false)
  {
  }
  
  void init(int32_t lat, int32_t lon, int32_t alt)
  {
    filteredLat = lat;
    filteredLon = lon;
    filteredAlt = alt;
    initialized = true;
  }
  
  void update(int32_t newLat, int32_t newLon, int32_t newAlt)
  {
    if (!initialized)
    {
      init(newLat, newLon, newAlt);
      return;
    }
    
    // Filtro paso bajo: filtered = alpha * new + (1 - alpha) * old
    filteredLat = (int32_t)(alpha * newLat + (1.0 - alpha) * filteredLat);
    filteredLon = (int32_t)(alpha * newLon + (1.0 - alpha) * filteredLon);
    filteredAlt = (int32_t)(alpha * newAlt + (1.0 - alpha) * filteredAlt);
  }
  
  int32_t getLat() const { return filteredLat; }
  int32_t getLon() const { return filteredLon; }
  int32_t getAlt() const { return filteredAlt; }
  bool isInitialized() const { return initialized; }
  
  void reset()
  {
    initialized = false;
  }
  
  void setAlpha(float newAlpha)
  {
    if (newAlpha >= 0.0 && newAlpha <= 1.0)
    {
      alpha = newAlpha;
    }
  }
};

// FASE 2: Sistema de Logging Persistente ---------------------------------------------------------------
// FASE 4: Logging Mejorado con niveles y rotación
#include <SPIFFS.h>

// Definiciones de niveles de log (antes de la clase Logger)
#define FWM_LOG_LEVEL_TRACE 0
#define FWM_LOG_LEVEL_DEBUG 1
#define FWM_LOG_LEVEL_INFO 2
#define FWM_LOG_LEVEL_WARNING 3
#define FWM_LOG_LEVEL_ERROR 4
#define FWM_CURRENT_LOG_LEVEL FWM_LOG_LEVEL_INFO  // Nivel mínimo de logging
#define FWM_LOG_FILE_MAX_SIZE 512000      // 512KB por archivo
#define FWM_LOG_FILE_MAX_COUNT 5          // Máximo 5 archivos de log

class Logger
{
private:
  File logFile;
  uint32_t logCounter = 0;
  bool initialized = false;
  uint32_t lastFlushTime = 0;
  const uint32_t FLUSH_INTERVAL = 5000; // Flush cada 5 segundos
  int currentLogFile = 0;
  
  void rotateLogIfNeeded()
  {
    if (!initialized) return;
    
    size_t fileSize = logFile.size();
    if (fileSize < FWM_LOG_FILE_MAX_SIZE) return;
    
    // Cerrar archivo actual
    logFile.close();
    
    // Rotar archivos existentes
    for (int i = FWM_LOG_FILE_MAX_COUNT - 1; i > 0; i--)
    {
      char oldName[32], newName[32];
      snprintf(oldName, sizeof(oldName), "/flight%d.log", i - 1);
      snprintf(newName, sizeof(newName), "/flight%d.log", i);
      
      if (SPIFFS.exists(oldName))
      {
        SPIFFS.remove(newName);  // Eliminar el más viejo
        SPIFFS.rename(oldName, newName);
      }
    }
    
    // Renombrar actual a .0
    SPIFFS.rename("/flight.log", "/flight0.log");
    
    // Abrir nuevo archivo
    logFile = SPIFFS.open("/flight.log", "w");
    Log.notice("Log file rotated" CR);
  }
  
public:
  void init()
  {
    if (!SPIFFS.begin(true))
    {
      Log.error("SPIFFS Mount Failed" CR);
      return;
    }
    
    logFile = SPIFFS.open("/flight.log", "a");
    if (!logFile)
    {
      Log.error("Failed to open log file" CR);
      return;
    }
    
    initialized = true;
    Log.notice("Logger initialized" CR);
    logEvent(FWM_LOG_LEVEL_INFO, "SYSTEM_START", "FlyWithMe started");
  }
  
  void logEvent(int level, const char* event, const char* data = "")
  {
    if (!initialized || level < FWM_CURRENT_LOG_LEVEL) return;
    
    rotateLogIfNeeded();
    
    const char* levelStr = "INFO";
    switch(level) {
      case FWM_LOG_LEVEL_TRACE:   levelStr = "TRACE"; break;
      case FWM_LOG_LEVEL_DEBUG:   levelStr = "DEBUG"; break;
      case FWM_LOG_LEVEL_INFO:    levelStr = "INFO"; break;
      case FWM_LOG_LEVEL_WARNING: levelStr = "WARN"; break;
      case FWM_LOG_LEVEL_ERROR:   levelStr = "ERROR"; break;
    }
    
    char logEntry[256];
    snprintf(logEntry, sizeof(logEntry), "%lu,%s,%s,%s\n", 
             millis(), levelStr, event, data);
    
    logFile.print(logEntry);
    logCounter++;
    
    // Flush periódicamente
    if (millis() - lastFlushTime > FLUSH_INTERVAL)
    {
      logFile.flush();
      lastFlushTime = millis();
    }
    
    #ifdef DEBUG_MODE
    Serial.print(logEntry);
    #endif
  }
  
  // Métodos de conveniencia para diferentes niveles
  void trace(const char* event, const char* data = "") { 
    logEvent(FWM_LOG_LEVEL_TRACE, event, data); 
  }
  void debug(const char* event, const char* data = "") { 
    logEvent(FWM_LOG_LEVEL_DEBUG, event, data); 
  }
  void info(const char* event, const char* data = "") { 
    logEvent(FWM_LOG_LEVEL_INFO, event, data); 
  }
  void warning(const char* event, const char* data = "") { 
    logEvent(FWM_LOG_LEVEL_WARNING, event, data); 
  }
  void error(const char* event, const char* data = "") { 
    logEvent(FWM_LOG_LEVEL_ERROR, event, data); 
  }
  
  void logTelemetry(APdata_t data)
  {
    if (!initialized) return;
    
    char telemetryData[128];
    snprintf(telemetryData, sizeof(telemetryData), 
             "lat=%d,lon=%d,alt=%d,spd=%d", 
             data.lat, data.lon, data.relative_alt, data.ground_speed);
    logEvent(FWM_LOG_LEVEL_DEBUG, "TELEMETRY", telemetryData);
  }
  
  void logPacket(LoraPacket_t packet, int rssi, int snr)
  {
    if (!initialized) return;
    
    char packetData[128];
    snprintf(packetData, sizeof(packetData),
             "sysid=%d,rssi=%d,snr=%d,lat=%d,lon=%d",
             packet.sysid, rssi, snr, packet.lat, packet.lon);
    logEvent(FWM_LOG_LEVEL_DEBUG, "RX_PACKET", packetData);
  }
  
  String getLogStats()
  {
    String stats = "Log Stats:\n";
    stats += "Total entries: " + String(logCounter) + "\n";
    stats += "Current file size: " + String(logFile.size()) + " bytes\n";
    stats += "Files: ";
    
    for (int i = 0; i < FWM_LOG_FILE_MAX_COUNT; i++)
    {
      char filename[32];
      if (i == 0) {
        strcpy(filename, "/flight.log");
      } else {
        snprintf(filename, sizeof(filename), "/flight%d.log", i - 1);
      }
      
      if (SPIFFS.exists(filename))
      {
        File f = SPIFFS.open(filename, "r");
        stats += String(filename) + "(" + String(f.size()) + " bytes) ";
        f.close();
      }
    }
    
    return stats;
  }
  
  void close()
  {
    if (initialized && logFile)
    {
      logFile.flush();
      logFile.close();
      Log.notice("Logger closed. Total events: %lu" CR, logCounter);
    }
  }
  
  uint32_t getLogCount() { return logCounter; }
};

// ============================================================================
// FASE 4: INTERFAZ Y TESTING
// ============================================================================

// --- Sistema de Menú Interactivo ---
// NOTA: los pines del menú (12/13/14/15) chocan con UART MAVLink (12/13), LoRa RST (14) y
// OLED SCL (15). Desactivado hasta reasignar los botones a GPIOs libres.
#define USE_INTERACTIVE_MENU 0        // Activar menú OLED
#define BUTTON_UP_PIN 12              // Botón navegación arriba
#define BUTTON_DOWN_PIN 13            // Botón navegación abajo
#define BUTTON_SELECT_PIN 14          // Botón seleccionar
#define BUTTON_BACK_PIN 15            // Botón volver
#define BUTTON_DEBOUNCE_MS 50         // Tiempo anti-rebote botones

// --- Servidor Web y API ---
#define USE_WEB_SERVER 1              // Activar servidor web
#define WEB_SERVER_PORT 80            // Puerto del servidor
#define USE_WEBSOCKET 1               // Activar WebSocket para telemetría
#define WEBSOCKET_PORT 81             // Puerto WebSocket
#define WEB_START_AP_IMMEDIATELY 1    // Iniciar AP al arrancar (1) o solo cuando no hay FC (0)

// --- Modo Simulación ---
#define SIMULATION_MODE 0             // Activar modo simulación (sin hardware)
#define SIMULATION_UPDATE_RATE 100    // Actualización simulación (ms)
#define SIMULATION_NOISE_LEVEL 0.1f   // Nivel de ruido en simulación

// --- Enlace de FC para pruebas ---
// FC_EMULATION=1: el ESP32 sintetiza telemetría válida en APdata (sin UART). Modo banco.
// FC_LINK_USB=1 : el MAVLink del FC se lee/escribe por el USB (UART0) en lugar del UART1.
//                 Para pruebas con SITL (puente serie <-> TCP). NO usar junto a FC_EMULATION.
#ifndef FC_EMULATION
#define FC_EMULATION 1
#endif
#ifndef FC_LINK_USB
#define FC_LINK_USB 0
#endif

// --- Enumeraciones de Menú ---
enum MenuState {
  MENU_MAIN,
  MENU_FORMATION,
  MENU_SETTINGS,
  MENU_DIAGNOSTICS,
  MENU_LOGS
};

enum MenuItem {
  ITEM_FORMATION_TYPE,
  ITEM_FORMATION_DISTANCE,
  ITEM_PREDICTION_TOGGLE,
  ITEM_FILTER_TOGGLE,
  ITEM_FILTER_ALPHA,
  ITEM_ADAPTIVE_RATE,
  ITEM_COMPRESSION,
  ITEM_VIEW_STATS,
  ITEM_VIEW_LOGS,
  ITEM_CALIBRATE_LORA,
  ITEM_RESET_STATS,
  ITEM_BACK
};

// --- Estructura de Menú ---
struct MenuOption {
  const char* label;
  MenuItem item;
};

// --- Estadísticas del Sistema ---
struct SystemStats {
  uint32_t uptime;
  uint32_t totalPacketsRx;
  uint32_t totalPacketsTx;
  uint32_t packetsLost;
  float packetLossRate;
  int avgRSSI;
  int avgSNR;
  float totalDistance;
  int linkDistance;   // A4: distancia al peer en metros (-1 si no hay beacon)
  uint32_t stateChanges;
  uint32_t safetyViolations;
};

// --- Simulador ---
#if (SIMULATION_MODE || FC_EMULATION)
struct SimulatedData {
  float lat;
  float lon;
  float alt;
  float heading;
  float groundSpeed;
  float climb;
  uint32_t lastUpdate;
  
  void init() {
    lat = 40.4168;  // Madrid ejemplo
    lon = -3.7038;
    alt = 600.0f;
    heading = 45.0f;
    groundSpeed = 1500.0f;  // 15 m/s en cm/s
    climb = 0.0f;
    lastUpdate = millis();
  }
  
  void update() {
    uint32_t now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    
    // Movimiento circular simulado
    heading += 5.0f * dt;
    if (heading >= 360.0f) heading -= 360.0f;
    
    float headingRad = heading * M_PI / 180.0f;
    float distanceM = (groundSpeed / 100.0f) * dt;
    
    lat += (distanceM * cos(headingRad)) / 111320.0f;
    lon += (distanceM * sin(headingRad)) / (111320.0f * cos(lat * M_PI / 180.0f));
    
    // Agregar ruido
    lat += (random(-100, 100) / 1000000.0f) * SIMULATION_NOISE_LEVEL;
    lon += (random(-100, 100) / 1000000.0f) * SIMULATION_NOISE_LEVEL;
    alt += (random(-10, 10) / 10.0f) * SIMULATION_NOISE_LEVEL;
  }
};
#endif

// SYSTEM STATES (FASE 1 - Máquina de Estados) --------------------------------------------------------
enum SystemState
{
  STATE_INIT,          // Inicializando sistema
  STATE_SEARCHING,     // Buscando beacon del líder
  STATE_CONNECTING,    // Estableciendo conexión
  STATE_FOLLOWING,     // Siguiendo al líder
  STATE_LOST_LINK,     // Enlace perdido
  STATE_EMERGENCY,     // Modo de emergencia
  STATE_LANDING        // Procedimiento de aterrizaje
};

#endif

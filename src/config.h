#ifndef Config_H
#define Config_H

#include <Arduino.h>
#include <ArduinoLog.h>
#include <HardwareSerial.h>
#include "../lib/mavlink/ardupilotmega/mavlink.h"

// DEBUG MODE
#define DEBUG_MODE // Comentar para desactivar debug

#define VERSION "FlyWithMe V1.0"

// MAVLink config ----------------------------------------------------------------------------------------
/* The default UART header for your MCU */
// #define SYSID 1                                   ///< ID 20 for this airplane. 1 PX, 255 ground station
// #define COMPID 158                                ///< The component sending the message
// #define SELF_TYPE MAV_TYPE_FIXED_WING             ///< This system is an airplane / fixed wing

// Define the system type, in this case an airplane -> on-board controller
// #define SYS_TYPE MAV_TYPE_GENERIC
// #define AP_TYPE MAV_AUTOPILOT_INVALID

// #define SYS_MODE MAV_MODE_PREFLIGHT               ///< Booting up
// #define CUSTOM_MODE 0                             ///< Custom mode, can be defined by user/adopter
// #define SYSTEM_STATE MAV_STATE_STANDBY            ///< System ready for flight

// Target system and component

// Data Streams
#define MAV_DATA_STREAM_POSITION_RATE 0x02       ///< 2 Hz
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 0x02 ///< 2 Hz

// Params setup
#define AUTO_SET_FOLL_PARAMS 1 ///< 1 to automatically set params, 0 otherwise

// Follow Target Interval
// #define FOLLOW_TARGET_INTERVAL 1000              ///< ms 1 vez por segundo

// GCS config --------------------------------------------------------------------------------------------
// #define GCS_SYSID 255                             ///< ID 255 for this ground station

// Intervals
#define HEARTBEAT_INTERVAL 1000 ///< ms 1 vez por segundo
// #define LORA_SEND_INTERVAL 500                    ///< ms 2 veces por segundo
#define BEACON_CHECK_INTERVAL 1000 ///< ms 1 vez por segundo
#define SEND_PACKET_INTERVAL 1000

// OTHER config ------------------------------------------------------------------------------------------

// Serial Bauds
#define SERIAL_BAUD 57600
#define SERIAL_BAUD_TELEM 57600

// Time to lost link
#define LOST_TIME 5        // s
#define LOST_TIME_BEACON 5 // s

// Flight modes
#define MODE_GUIDED 15

// #define QUEUE_SIZE 10

// Web Server
#define WEB_PORT 80

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

// Data types --------------------------------------------------------------------------------------------
// #define LORA_HEARTBEAT_PACKET 0
// #define LORA_POSITION_PACKET 1
// #define LORA_LOCAL_POSITION_PACKET 2

// Stages ------------------------------------------------------------------------------------------------
#define STAGE_IDLE 0
#define STAGE_APPROACH 1
#define STAGE_HOLDPOS 2

// Follow modes --------------------------------------------------------------------------------------------
#define FOLL_MODE_OFF 0
#define FOLL_MODE_FOLLOWER 1
#define FOLL_MODE_LEADER 2

// DEFAULT PARAMS
#define FOLL_ENABLE 1
#define FOLL_OFS_TYPE 1
#define FOLL_ALT_TYPE 0
#define LINK_TIMEOUT 10
#define DEFAULT_SSID "FWM AP 1"
#define DEFAULT_PASS "12345678"
#define FOLL_MODE FOLL_MODE_OFF
#define FOLL_MODE_CH 7

// Set by target
#ifdef MASTER_BUILD_FLAG
#define TARGET_SYSID 1     ///< Pixhawk (or any other autopilot)
#define TARGET_COMPID 1    ///< Component
#define SYSID TARGET_SYSID ///< ID 20 for this airplane. 1 PX, 255 ground station
#define COMPID 158         ///< The component sending the message
#define DEFAULT_SSID "FWM AP 1"
#define DEFAULT_PASS "12345678"
#define FOLL_MODE FOLL_MODE_LEADER
#define FOLL_MODE_CH 0
#endif

#ifdef SLAVE_BUILD_FLAG
#define TARGET_SYSID 2     ///< Pixhawk (or any other autopilot)
#define TARGET_COMPID 1    ///< Component
#define SYSID TARGET_SYSID ///< ID 20 for this airplane. 1 PX, 255 ground station
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

typedef struct
{
  unsigned long tx_packet_counter = 0;
  unsigned long rx_packet_counter = 0;
  unsigned long lost_packet_counter = 0;
  uint16_t lastValidPacketSize = 0;
  LoraPacket_t lastValidPacket;
  bool have_beacon = false;
  int rssi = 0;
} CommData_t;

// Struct que contiene el modo de vuelo y su nombre asociado
struct FlightModeInfo
{
  int32_t mode;
  std::string name;
};

#endif

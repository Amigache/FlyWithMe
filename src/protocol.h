#pragma once
// -----------------------------------------------------------------------------------------------------------
// Protocolo LoRa FlyWithMe (v2) -- header PURO (sin dependencias de Arduino) para poder testear en host.
//
// Cambios v2:
//   * version + type (BEACON/REPLY/JOIN) -> un mismo struct sirve para ambas direcciones.
//   * netid ("frase"/red): el receptor DESCARTA paquetes de otras redes/sistemas (anti-cruce).
//   * mode: modo de vuelo del emisor (ArduPlane custom_mode) -> el seguidor puede exigir modo estable.
//
// El checksum suma el struct entero (padding incluido), excluyendo el propio byte de checksum.
// -----------------------------------------------------------------------------------------------------------
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_VERSION 2

enum LoraMsgType : uint8_t
{
  LORA_MSG_BEACON = 1, ///< lider -> aire (posicion/modo del lider)
  LORA_MSG_REPLY = 2,  ///< seguidor -> lider (posicion del seguidor; para el OSD del lider)
  LORA_MSG_JOIN = 3    ///< seguidor -> lider (peticion de sesion; el lider empieza a emitir)
};

enum LoraPacketFlags : uint8_t
{
  LORA_FLAG_NONE = 0,
  LORA_FLAG_REPLY_SLOT = 1 << 0, ///< líder reserva la ventana de retorno para un JOIN/REPLY
  LORA_FLAG_POSITION_VALID = 1 << 1 ///< el emisor tiene posición FC/GPS válida
};

typedef struct __attribute__((packed))
{
  uint8_t version;       ///< PROTOCOL_VERSION
  uint8_t type;          ///< LoraMsgType
  uint16_t netid;        ///< red compartida ("frase"); filtra trafico de otros sistemas
  uint8_t sysid;         ///< ID de sistema del emisor
  uint8_t mode;          ///< ArduPlane custom_mode del emisor (modo de vuelo)
  uint16_t seq;          ///< numero de secuencia (deteccion de perdidas/duplicados/reorden)
  int32_t lat;           ///< Latitud * 1E7
  int32_t lon;           ///< Longitud * 1E7
  int32_t alt;           ///< Altitud MSL (mm)
  int32_t relative_alt;  ///< Altitud sobre el terreno (mm)
  uint16_t ground_speed; ///< [cm/s]
  uint16_t hdg;          ///< rumbo * 100 (0..35999; 65535 = desconocido)
  uint32_t timestamp;    ///< [ms] millis() del emisor al enviar
  int16_t vx;            ///< [cm/s] velocidad NED: norte
  int16_t vy;            ///< [cm/s] velocidad NED: este
  int16_t vz;            ///< [cm/s] velocidad NED: abajo
  uint8_t flags;         ///< LoraPacketFlags
  uint8_t checksum;      ///< Checksum
} LoraPacket_t;

// This is a wire format, not an in-memory ABI: prohibit implicit padding from entering the checksum
// or the transmitted frame. Keep this assertion so layout regressions fail the firmware build.
static_assert(offsetof(LoraPacket_t, checksum) == sizeof(LoraPacket_t) - 1,
              "LoraPacket_t checksum must be the final wire byte");
static_assert(sizeof(LoraPacket_t) == 40, "Unexpected FlyWithMe protocol v2 frame size");

// Checksum por suma de bytes (excluye el propio checksum).
inline uint8_t loraPacketChecksum(const LoraPacket_t &p)
{
  const uint8_t *b = reinterpret_cast<const uint8_t *>(&p);
  const size_t n = sizeof(LoraPacket_t);
  const size_t skip = offsetof(LoraPacket_t, checksum);
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i)
  {
    if (i != skip)
    {
      c += b[i];
    }
  }
  return c;
}

inline bool loraChecksumOk(const LoraPacket_t &p) { return loraPacketChecksum(p) == p.checksum; }
inline bool loraVersionOk(const LoraPacket_t &p) { return p.version == PROTOCOL_VERSION; }
inline bool loraNetidOk(const LoraPacket_t &p, uint16_t netid) { return p.netid == netid; }
inline bool loraPositionOk(const LoraPacket_t &p) { return (p.flags & LORA_FLAG_POSITION_VALID) != 0; }

// Comparación modular de seq: soporta wrap 65535 -> 0 y rechaza repetidos/paquetes atrasados.
inline bool loraSeqIsNewer(uint16_t candidate, uint16_t previous)
{
  return static_cast<int16_t>(candidate - previous) > 0;
}

// Sync word de radio (1 byte) derivado del netid; debe coincidir en ambos extremos.
inline uint8_t loraSyncWordFor(uint16_t netid) { return (uint8_t)(0x10 | (netid & 0x0F)); }

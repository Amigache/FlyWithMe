#pragma once
// -----------------------------------------------------------------------------------------------------------
// Auto-test del protocolo FlyWithMe. Solo depende de protocol.h (puro) -> corre en la placa al arrancar y
// tambien puede compilarse en host. Devuelve el numero de fallos y deja un resumen en `out`/`n`.
// -----------------------------------------------------------------------------------------------------------
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "status_text.h"
#include "wifi_identity.h"

inline int protocolSelfTest(char *out, size_t n)
{
  int checks = 0, fails = 0;

#define FWM_CHECK(cond)          \
  do                             \
  {                              \
    ++checks;                    \
    if (!(cond))                 \
      ++fails;                   \
  } while (0)

  // 1. Version esperada
  FWM_CHECK(PROTOCOL_VERSION == 2);

  // 2. El checksum es el ultimo campo del struct
  FWM_CHECK(offsetof(LoraPacket_t, checksum) == sizeof(LoraPacket_t) - 1);

  // 3. Checksum: se calcula y valida; al corromper un byte deja de validar
  LoraPacket_t p = {};
  p.version = PROTOCOL_VERSION;
  p.type = LORA_MSG_BEACON;
  p.netid = 0x1234;
  p.sysid = 1;
  p.mode = 10;
  p.lat = 370000000;
  p.lon = -65600000;
  p.relative_alt = 200000;
  p.ground_speed = 2200;
  p.hdg = 18000;
  p.timestamp = 12345678;
  p.vx = 2000;
  p.vy = -100;
  p.vz = 50;
  p.flags = LORA_FLAG_POSITION_VALID;
  p.checksum = loraPacketChecksum(p);
  FWM_CHECK(loraChecksumOk(p));
  FWM_CHECK(loraPositionOk(p));

  // 4. Round-trip exacto de la trama y rechazo de corrupción en cualquier byte cubierto.
  uint8_t wire[sizeof(LoraPacket_t)];
  memcpy(wire, &p, sizeof(p));
  LoraPacket_t decoded = {};
  memcpy(&decoded, wire, sizeof(decoded));
  FWM_CHECK(loraChecksumOk(decoded));
  for (size_t i = 0; i < sizeof(wire) - 1; ++i)
  {
    wire[i] ^= 0x01;
    memcpy(&decoded, wire, sizeof(decoded));
    FWM_CHECK(!loraChecksumOk(decoded));
    wire[i] ^= 0x01;
  }
  decoded = p;
  decoded.checksum ^= 0x01;
  FWM_CHECK(!loraChecksumOk(decoded));

  // 5. Filtro de red (netid)
  FWM_CHECK(loraNetidOk(p, 0x1234));
  FWM_CHECK(!loraNetidOk(p, 0x1235));

  // 6. Filtro de version
  FWM_CHECK(loraVersionOk(p));
  LoraPacket_t pv = p;
  pv.version = PROTOCOL_VERSION + 1;
  FWM_CHECK(!loraVersionOk(pv));

  // 7. Secuencia modular: avance normal y wrap, duplicado/paquete atrasado rechazados.
  FWM_CHECK(loraSeqIsNewer(11, 10));
  FWM_CHECK(loraSeqIsNewer(0, 65535));
  FWM_CHECK(!loraSeqIsNewer(10, 10));
  FWM_CHECK(!loraSeqIsNewer(9, 10));

  // 8. Sync word derivado del netid: determinista y distinto por red
  FWM_CHECK(loraSyncWordFor(0x1234) == loraSyncWordFor(0x1234));
  FWM_CHECK(loraSyncWordFor(0x1234) != loraSyncWordFor(0x1235));

  // 9. STATUSTEXT: iguales no se repiten nunca; un cambio de texto/severidad sí se envía.
  StatusTextDeduper deduper;
  FWM_CHECK(deduper.shouldSend("FWM: Follower 48m", 4));
  FWM_CHECK(!deduper.shouldSend("FWM: Follower 48m", 4));
  FWM_CHECK(deduper.shouldSend("FWM: Follower 49m", 4));
  FWM_CHECK(deduper.shouldSend("FWM: Follower 49m", 6));

  // 10. Identidad WiFi única: MAC -> SSID con los últimos 3 bytes, sin separadores.
  const uint8_t testMac[6] = {0x30, 0xAE, 0xA4, 0x07, 0x0D, 0x64};
  char macText[18] = {};
  char ssid[11] = {};
  FWM_CHECK(fwmWifiIdentityFromMac(testMac, macText, sizeof(macText), ssid, sizeof(ssid)));
  FWM_CHECK(strcmp(macText, "30:AE:A4:07:0D:64") == 0);
  FWM_CHECK(strcmp(ssid, "FWM 070D64") == 0);

#undef FWM_CHECK

  if (out && n)
  {
    snprintf(out, n, "protocol: %d checks, %d fails, size=%u", checks, fails,
             (unsigned)sizeof(LoraPacket_t));
  }
  return fails;
}

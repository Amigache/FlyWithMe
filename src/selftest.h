#pragma once
// -----------------------------------------------------------------------------------------------------------
// Auto-test del protocolo FlyWithMe. Solo depende de protocol.h (puro) -> corre en la placa al arrancar y
// tambien puede compilarse en host. Devuelve el numero de fallos y deja un resumen en `out`/`n`.
// -----------------------------------------------------------------------------------------------------------
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "protocol.h"

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
  LoraPacket_t p;
  memset(&p, 0, sizeof(p));
  p.version = PROTOCOL_VERSION;
  p.type = LORA_MSG_BEACON;
  p.netid = 0x1234;
  p.sysid = 1;
  p.mode = 10;
  p.lat = 370000000;
  p.lon = -65600000;
  p.relative_alt = 200000;
  p.checksum = loraPacketChecksum(p);
  FWM_CHECK(loraChecksumOk(p));
  p.relative_alt ^= 0x01; // corromper
  FWM_CHECK(!loraChecksumOk(p));
  p.relative_alt ^= 0x01; // restaurar

  // 4. Filtro de red (netid)
  FWM_CHECK(loraNetidOk(p, 0x1234));
  FWM_CHECK(!loraNetidOk(p, 0x1235));

  // 5. Filtro de version
  FWM_CHECK(loraVersionOk(p));
  LoraPacket_t pv = p;
  pv.version = PROTOCOL_VERSION + 1;
  FWM_CHECK(!loraVersionOk(pv));

  // 6. Sync word derivado del netid: determinista y distinto por red
  FWM_CHECK(loraSyncWordFor(0x1234) == loraSyncWordFor(0x1234));
  FWM_CHECK(loraSyncWordFor(0x1234) != loraSyncWordFor(0x1235));

#undef FWM_CHECK

  if (out && n)
  {
    snprintf(out, n, "protocol: %d checks, %d fails, size=%u", checks, fails,
             (unsigned)sizeof(LoraPacket_t));
  }
  return fails;
}

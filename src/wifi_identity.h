#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Passphrase WPA2-PSK: 8..63 caracteres ASCII imprimibles (0x20..0x7E).
#define FWM_AP_PASS_MIN_LEN 8
#define FWM_AP_PASS_MAX_LEN 63

inline bool fwmApPassphraseValid(const char *pass)
{
  if (pass == nullptr)
    return false;
  const size_t length = strlen(pass);
  if (length < FWM_AP_PASS_MIN_LEN || length > FWM_AP_PASS_MAX_LEN)
    return false;
  for (size_t i = 0; i < length; ++i)
  {
    const unsigned char c = (unsigned char)pass[i];
    if (c < 0x20 || c > 0x7E)
      return false;
  }
  return true;
}

// La SSID se deriva de la MAC de la interfaz SoftAP (los últimos 3 bytes).
// Ejemplo: 30:AE:A4:07:0D:64 -> "FWM 070D64".
inline bool fwmWifiIdentityFromMac(const uint8_t mac[6], char *macText, size_t macTextSize,
                                   char *ssid, size_t ssidSize)
{
  if (mac == nullptr || macText == nullptr || ssid == nullptr || macTextSize < 18 || ssidSize < 11)
    return false;

  const int macLength = snprintf(macText, macTextSize, "%02X:%02X:%02X:%02X:%02X:%02X",
                                 (unsigned)mac[0], (unsigned)mac[1], (unsigned)mac[2],
                                 (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
  const int ssidLength = snprintf(ssid, ssidSize, "FWM %02X%02X%02X",
                                  (unsigned)mac[3], (unsigned)mac[4], (unsigned)mac[5]);
  return macLength == 17 && ssidLength == 10;
}

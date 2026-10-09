#pragma once

#include <stdint.h>

// Bandas LoRa soportadas por el SX1276 de la TTGO LoRa32.
//
// Limite de HARDWARE, no de software: segun el datasheet de Semtech el SX1276/77/78/79
// cubren 137-1020 MHz. 2.4 GHz queda FUERA de rango y requiere otro radio/otra placa,
// asi que no es una opcion configurable. El SX1278 (433 MHz) si llega a 525 MHz, pero el
// SX1276 de esta placa cubre las tres bandas de abajo.
#define FWM_BAND_433 0
#define FWM_BAND_868 1
#define FWM_BAND_900 2
#define FWM_BAND_COUNT 3

// Frecuencias en Hz (literales enteras: comparables en el preprocesador con #if).
#define FWM_LORA_FREQ_433 433000000UL
#define FWM_LORA_FREQ_868 866000000UL
#define FWM_LORA_FREQ_900 915000000UL

inline bool loraBandValid(int32_t band)
{
  return band >= 0 && band < FWM_BAND_COUNT;
}

// Frecuencia que se pasa a LoRa.begin(). Para un indice invalido devuelve la de 868 MHz:
// preferimos un valor seguro a abortar el arranque del radio.
inline unsigned long loraBandFrequency(int32_t band)
{
  switch (band)
  {
  case FWM_BAND_433:
    return FWM_LORA_FREQ_433;
  case FWM_BAND_900:
    return FWM_LORA_FREQ_900;
  case FWM_BAND_868:
  default:
    return FWM_LORA_FREQ_868;
  }
}

// Etiqueta corta para OSD, WebUI y STATUSTEXT.
inline const char *loraBandLabel(int32_t band)
{
  switch (band)
  {
  case FWM_BAND_433:
    return "433";
  case FWM_BAND_900:
    return "915";
  case FWM_BAND_868:
  default:
    return "868";
  }
}

// Inversa de loraBandFrequency(), para derivar el indice a partir de una frecuencia.
inline int32_t loraBandFromFrequency(unsigned long frequency)
{
  if (frequency == FWM_LORA_FREQ_433)
    return FWM_BAND_433;
  if (frequency == FWM_LORA_FREQ_900)
    return FWM_BAND_900;
  return FWM_BAND_868;
}
#include "Comm.h"

Comm *Comm::self = nullptr;

Comm::Comm(FWM *fwm)
{
  self = this;
  this->fwm = fwm;
}

void Comm::begin()
{
  // SPI- --------------------------------------------------------------------------------------------------
  Log.notice("Init SPI for LoRa" CR);
  SPI.begin(SCK, MISO, MOSI, SS);
  delay(2000);
  Log.notice("SPI for LoRa Ready" CR);

  // LORA --------------------------------------------------------------------------------------------------
  Log.notice("Init LoRa" CR);
  LoRa.setPins(SS, RST, DIO0);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH); // 125kHz
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR); // SF12
  LoRa.setCodingRate4(LORA_CODING_RATE);          // 4/5
  LoRa.setTxPower(LORA_TX_POWER);                 // 20dBm
  LoRa.setSyncWord(LORA_SYNC_WORD);

  if (!LoRa.begin(LORA_BAND))
  {
    Log.error("Starting LoRa failed!" CR);
    for (;;)
      ; // Don't proceed, loop forever
  }

  delay(3000);
  Log.notice("LoRa Ready" CR);

  // TICKER -----------------------------------------------------------------------------------------------
  beacon_ticker.attach_ms(BEACON_CHECK_INTERVAL, Comm::beacon_ticker_callback);
}

// Static member function definition
void Comm::beacon_ticker_callback()
{
  if (self)
  {
    // Only work if we are on follower mode
    if (self->fwm->follow_mode == FOLL_MODE_FOLLOWER)
    {
      // Calculamos tiempo pasado desde el último beacon
      unsigned long now = millis() / 1000;
      unsigned long last_bc = self->last_beacon / 1000;
      int dif = now - last_bc;

      if (dif >= LOST_TIME_BEACON)
      { // PERDEMOS LINK
        Log.warning("WAITING BEACON" CR);
        self->commData.have_beacon = false;
      }
      else if (dif <= 1 && !self->commData.have_beacon)
      { // RECUPERAMOS LINK
        Log.notice("BEACON LOCK" CR);
        self->commData.have_beacon = true;
      }
    }
  }
}

void Comm::run()
{
  // Only work if we are on follower mode
  if (fwm->follow_mode == FOLL_MODE_FOLLOWER)
  {
    // Recibir paquete
    if (LoRa.parsePacket())
    {
      LoraPacket_t incomingPacket;
      LoRa.readBytes((uint8_t *)&incomingPacket, sizeof(incomingPacket));

      commData.rssi = LoRa.packetRssi();

      // Comprobamos el checksum de msg
      if (validateChecksum(incomingPacket))
      {
        // Actualizamos commData
        commData.lastValidPacket = incomingPacket;
        commData.lastValidPacketSize = sizeof(commData.lastValidPacket);
        commData.rx_packet_counter++;

        // Actualizamos target si estamos en modo follow
        if (fwm->stage_follow == STAGE_APPROACH && fwm->mav->link)
        {
          fwm->mav->nav_waypoint(commData.lastValidPacket.lat, commData.lastValidPacket.lon, commData.lastValidPacket.relative_alt);
          fwm->mav->do_change_speed(commData.lastValidPacket.ground_speed); 
        }

        // Time to get
        last_beacon = millis();
      }
      else
      {
        // Paquete con checksum inválido, Incrementamos
        commData.lost_packet_counter++;
      }
    }
  }
}

void Comm::sendPacket(LoraPacket_t packet)
{
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&packet, sizeof(packet));
  LoRa.endPacket();

  // Incrementamos
  commData.tx_packet_counter++;
}

bool Comm::validateChecksum(LoraPacket_t packet)
{
  uint8_t calculated = calChecksum(packet);
  return calculated == packet.checksum;
}

uint8_t Comm::calChecksum(LoraPacket_t packet)
{
  uint8_t checksum = 0;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  size_t size = sizeof(packet);

  // Excluir el campo 'checksum' del cálculo
  size_t checksumIndex = offsetof(LoraPacket_t, checksum);
  for (size_t i = 0; i < size; ++i)
  {
    if (i != checksumIndex)
    {
      checksum += bytes[i];
    }
  }

  return checksum;
}

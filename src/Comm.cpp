#include "Comm.h"

Comm *Comm::self = nullptr;

Comm::Comm(FWM *fwm)
{
  self = this;
  // A1: estado de comunicación conocido desde el arranque
  memset(&commData, 0, sizeof(commData));
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

  if (!LoRa.begin(LORA_BAND))
  {
    Log.error("Starting LoRa failed!" CR);
    for (;;)
      ; // Don't proceed, loop forever
  }

  // IMPORTANTE: los setters deben ir DESPUÉS de begin(). Antes, los registros del SX1276 están
  // sin inicializar y setLdoFlag() calcula getSignalBandwidth()/2^SF == 0 -> divide by zero.
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH); // 125kHz
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR); // SF12
  LoRa.setCodingRate4(LORA_CODING_RATE);          // 4/5
  LoRa.setTxPower(LORA_TX_POWER);                 // 20dBm
  // Radio: sync word y CRC (config conocida-buena por defecto; ver flags en config.h).
  LoRa.setSyncWord(LORA_SYNC_WORD);
#if NETID_USE_SYNCWORD
  if (fwm->params.netid)
  {
    LoRa.setSyncWord(loraSyncWordFor(fwm->params.netid));
  }
#endif
#if LORA_CRC
  LoRa.enableCrc();
#endif

  delay(3000);
  Log.notice("LoRa Ready" CR);
  
  // FASE 3: Auto-calibración de LoRa (si está habilitada)
  #if AUTO_CALIBRATE_LORA
  if (!MAV_BRIDGE && fwm->follow_mode == FOLL_MODE_FOLLOWER)
  {
    Log.notice("Auto-calibration enabled - waiting for leader signal..." CR);
    delay(5000); // Esperar a que el líder comience a transmitir
    autoCalibrate();
  }
  #endif

}

void Comm::markBeaconReceived()
{
  last_beacon = millis();
  if (!commData.have_beacon)
  {
    Log.notice("BEACON LOCK" CR);
    commData.have_beacon = true;
  }
}

void Comm::run()
{
#if NETID_USE_SYNCWORD
  // Las escrituras de parámetros pueden venir de Web/core0; el SX1276 solo lo toca este loop.
  if (syncWordPending)
  {
    syncWordPending = false;
    LoRa.setSyncWord(loraSyncWordFor(fwm->params.netid));
  }
#endif

  // FASE 4: Modo simulación - generar packets sintéticos
  #if SIMULATION_MODE
  if (fwm->follow_mode == FOLL_MODE_FOLLOWER)
  {
    uint32_t now = millis();
    if (now - lastSimulatedPacket > SIMULATION_UPDATE_RATE)
    {
      lastSimulatedPacket = now;
      
      // Actualizar datos simulados
      fwm->mav->updateSimulation();
      
      // Obtener packet simulado
      LoraPacket_t simulatedPacket = fwm->mav->getSimulatedPacket();
      
      // Simular RSSI y SNR
      commData.rssi = -60 + random(-20, 20);
      commData.snr = 8 + random(-2, 2);
      
      // Procesar como packet real
      commData.lastValidPacket = simulatedPacket;
      commData.lastValidPacketSize = sizeof(LoraPacket_t);
      commData.rx_packet_counter++;
      
      // Calcular posición de formación y enviar waypoint
      int32_t targetLat, targetLon, targetAlt;
      fwm->mav->calculateFormationPosition(simulatedPacket, fwm->mav->currentFormation,
                                           targetLat, targetLon, targetAlt);
      
      fwm->mav->nav_waypoint(targetLat, targetLon, targetAlt);
      
      Log.trace("Simulación: packet #%lu, RSSI=%d" CR, 
                commData.rx_packet_counter, commData.rssi);
    }
    return;  // No procesar LoRa real en modo simulación
  }
#endif

  // Beacon timeout en el mismo loop/propietario del enlace: sin Ticker concurrente sobre commData.
  if (!MAV_BRIDGE && fwm->follow_mode == FOLL_MODE_FOLLOWER && commData.have_beacon &&
      last_beacon != 0 && (uint32_t)(millis() - (uint32_t)last_beacon) >= LOST_TIME_BEACON * 1000u)
  {
    Log.warning("WAITING BEACON" CR);
    commData.have_beacon = false;
    last_beacon = 0;
    lastRxSeqInitialized = false; // permitir reinicio del líder (seq vuelve a cero) tras timeout.
    replyPending = false;
  }

#if FOLLOWER_REPLY
  // Ejecutar el REPLY en el mismo loop que recibe LoRa. Nunca transmitir desde otro ticker/task.
  if (fwm->follow_mode == FOLL_MODE_FOLLOWER && replyPending &&
      (int32_t)(millis() - replyDueMs) >= 0)
  {
    replyPending = false;
    sendReplyPacket();
  }
#endif

  // RX/TX del líder se ejecutan secuencialmente en este loop (el callback del Ticker solo marca due).
#if FOLLOWER_REPLY
  if (fwm->follow_mode == FOLL_MODE_LEADER)
  {
    if (LoRa.parsePacket())
    {
      commData.rssi = LoRa.packetRssi();
      commData.snr = LoRa.packetSnr();
      if (LoRa.available() == (int)sizeof(LoraPacket_t))
      {
        LoraPacket_t p;
        LoRa.readBytes((uint8_t *)&p, sizeof(p));
        if (validatePacket(p) && (FWM_PEER_SYSID == 0 || p.sysid == FWM_PEER_SYSID) &&
            (p.type == LORA_MSG_REPLY || p.type == LORA_MSG_JOIN) && acceptSequence(p.seq))
        {
          commData.rx_packet_counter++;
          fwm->onFollowerReply(p);
        }
      }
      else
      {
        while (LoRa.available())
        {
          LoRa.read();
        }
      }
    }
  }
#endif

  // Only work if we are on follower mode
  if (fwm->follow_mode == FOLL_MODE_FOLLOWER)
  {
    // Recibir paquete
    if (LoRa.parsePacket())
    {
      commData.rssi = LoRa.packetRssi();
      commData.snr = LoRa.packetSnr();
      
      // FASE 2: Detectar tipo de paquete por tamaño
      int packetSize = LoRa.available();
      static uint32_t lastSzLog = 0;
      if (packetSize != (int)sizeof(LoraPacket_t) && millis() - lastSzLog > 1000)
      {
        Log.warning("RX size=%d esperado=%d" CR, packetSize, (int)sizeof(LoraPacket_t));
        lastSzLog = millis();
      }
      
      #if USE_COMPRESSED_PACKETS
      if (packetSize == sizeof(CompressedLoraPacket_t))
      {
        // Paquete comprimido
        CompressedLoraPacket_t compressedPacket;
        LoRa.readBytes((uint8_t *)&compressedPacket, sizeof(compressedPacket));
        
        // Validar checksum comprimido
        if (validateChecksumCompressed(compressedPacket))
        {
          // Descomprimir a formato normal
          LoraPacket_t incomingPacket = decompressPacket(compressedPacket);
          
          // FASE 1: Validación completa del paquete
          if (validatePacket(incomingPacket))
          {
            // Actualizamos commData
            commData.lastValidPacket = incomingPacket;
            commData.lastValidPacketSize = sizeof(compressedPacket);
            commData.rx_packet_counter++;

            // Update target if we are in approach stage
            if (fwm->stage_follow == STAGE_APPROACH)
            {
              // FASE 1: Verificar seguridad antes de seguir
              if (fwm->mav->isSafeToFollow(incomingPacket))
              {
                // FASE 3: Calcular posición objetivo con predicción y formación
                int32_t targetLat, targetLon, targetAlt;
                
                if (fwm->mav->predictionEnabled)
                {
                  // Predecir posición futura del líder
                  PredictedPosition predicted = fwm->mav->predictLeaderPosition(incomingPacket);

                  // Usar posición predicha si la confianza es suficiente
                  if (predicted.confidence > 0.5)
                  {
                    // Calcular posición de formación basada en predicción
                    LoraPacket_t predictedPacket = incomingPacket;
                    predictedPacket.lat = predicted.lat;
                    predictedPacket.lon = predicted.lon;
                    predictedPacket.relative_alt = predicted.alt;

                    fwm->mav->calculateFormationPosition(predictedPacket, fwm->mav->currentFormation,
                                                         targetLat, targetLon, targetAlt);
                    fwm->mav->lastPrediction = predicted;
                  }
                  else
                  {
                    // Usar posición actual si la predicción no es confiable
                    fwm->mav->calculateFormationPosition(incomingPacket, fwm->mav->currentFormation,
                                                         targetLat, targetLon, targetAlt);
                  }
                }
                else
                {
                  // Sin predicción, calcular posición de formación directamente
                  fwm->mav->calculateFormationPosition(incomingPacket, fwm->mav->currentFormation,
                                                       targetLat, targetLon, targetAlt);
                }

                if (fwm->mav->filterEnabled)
                {
                  // Aplicar filtro de posición para suavizar movimientos
                  fwm->mav->positionFilter.update(targetLat, targetLon, targetAlt);
                  targetLat = fwm->mav->positionFilter.getLat();
                  targetLon = fwm->mav->positionFilter.getLon();
                  targetAlt = fwm->mav->positionFilter.getAlt();
                }
                
                // Actualizar waypoint con posición calculada
                fwm->mav->nav_waypoint(targetLat, targetLon, targetAlt);
                
                // Get dynamic speed
                // Distancia real al LIDER (no wp_dist, que en GUIDED+DO_REPOSITION suele ser 0) para
                // que el seguidor acelere si va por detras y converja a la distancia de formacion.
                float distToLeader = fwm->mav->calculateDistance(
                    fwm->mav->APdata.lat, fwm->mav->APdata.lon,
                    incomingPacket.lat, incomingPacket.lon);
                uint16_t dynamic_speed = fwm->mav->calculate_dynamic_speed(incomingPacket.ground_speed, distToLeader);

                // Change speed
                fwm->mav->do_change_speed(dynamic_speed);
              }
            }

            // Time to get
            markBeaconReceived();
          }
          else
          {
            commData.lost_packet_counter++;
          }
        }
        else
        {
          Log.warning("Invalid compressed checksum" CR);
          commData.lost_packet_counter++;
        }
      }
      else
      #endif
      if (packetSize == sizeof(LoraPacket_t))
      {
        // Trama normal del protocolo v2.
        LoraPacket_t incomingPacket;
        LoRa.readBytes((uint8_t *)&incomingPacket, sizeof(incomingPacket));

        // FASE 1: Validación completa del paquete (checksum + datos GPS)
        if (validatePacket(incomingPacket) && incomingPacket.type == LORA_MSG_BEACON &&
            (FWM_PEER_SYSID == 0 || incomingPacket.sysid == FWM_PEER_SYSID))
        {
          if (!acceptSequence(incomingPacket.seq))
          {
            // No refrescar el timeout con un paquete duplicado/antiguo. Así, si el líder reinicia
            // y su secuencia vuelve a cero, el enlace expira y el próximo beacon reinicializa seq.
            return;
          }
          // Actualizamos commData
          commData.lastValidPacket = incomingPacket;
          commData.lastValidPacketSize = sizeof(commData.lastValidPacket);
          commData.rx_packet_counter++;

#if FOLLOWER_REPLY
          if (incomingPacket.flags & LORA_FLAG_REPLY_SLOT)
          {
            replyPending = true;
            replyDueMs = millis() + REPLY_SLOT_DELAY_MS;
          }
#endif

          // v2 (Idea 2): en distancia de aproximacion, exigir que el lider este en modo ESTABLE.
          float dLead = fwm->mav->calculateDistance(fwm->mav->APdata.lat, fwm->mav->APdata.lon,
                                                    incomingPacket.lat, incomingPacket.lon);
          static bool modeGateActive = false;
          if (dLead < fwm->params.approach_dist && !isLeaderModeStable(incomingPacket.mode))
          {
            static uint32_t lastGateMsg = 0;
            if (millis() - lastGateMsg > 2000)
            {
              Log.warning("Leader mode unstable (%d) at %dm: follow update suspended" CR,
                          incomingPacket.mode, (int)dLead);
              fwm->mav->status_text("Leader mode unstable - follow suspended");
              lastGateMsg = millis();
            }
            modeGateActive = true;
            markBeaconReceived(); // el enlace esta vivo; no es perdida de beacon
            return;
          }
          if (modeGateActive)
          {
            Log.notice("Leader mode gate released; follow updates resumed" CR);
            fwm->mav->status_text("Leader stable - follow resumed");
            modeGateActive = false;
          }

          // Update target if we are in approach stage
          if (fwm->stage_follow == STAGE_APPROACH)
          {
            // FASE 1: Verificar límites de seguridad antes de seguir
            if (fwm->mav->isSafeToFollow(commData.lastValidPacket))
            {
              // FASE 3: Calcular posición objetivo con predicción y formación
              int32_t targetLat, targetLon, targetAlt;
              
              if (fwm->mav->predictionEnabled)
              {
                // Predecir posición futura del líder
                PredictedPosition predicted = fwm->mav->predictLeaderPosition(commData.lastValidPacket);

                // Usar posición predicha si la confianza es suficiente
                if (predicted.confidence > 0.5)
                {
                  LoraPacket_t predictedPacket = commData.lastValidPacket;
                  predictedPacket.lat = predicted.lat;
                  predictedPacket.lon = predicted.lon;
                  predictedPacket.relative_alt = predicted.alt;

                  fwm->mav->calculateFormationPosition(predictedPacket, fwm->mav->currentFormation,
                                                       targetLat, targetLon, targetAlt);
                  fwm->mav->lastPrediction = predicted;
                }
                else
                {
                  fwm->mav->calculateFormationPosition(commData.lastValidPacket, fwm->mav->currentFormation,
                                                       targetLat, targetLon, targetAlt);
                }
              }
              else
              {
                fwm->mav->calculateFormationPosition(commData.lastValidPacket, fwm->mav->currentFormation,
                                                     targetLat, targetLon, targetAlt);
              }

              if (fwm->mav->filterEnabled)
              {
                // Aplicar filtro de posición
                fwm->mav->positionFilter.update(targetLat, targetLon, targetAlt);
                targetLat = fwm->mav->positionFilter.getLat();
                targetLon = fwm->mav->positionFilter.getLon();
                targetAlt = fwm->mav->positionFilter.getAlt();
              }
              
              #if USE_HEADING_GUIDANCE
              // Guiado por RUMBO + velocidad + altitud: no crea objetivo de posicion -> no loitea
              fwm->mav->guided_follow(commData.lastValidPacket, targetLat, targetLon, targetAlt);
              #else
              // Actualizar waypoint
              fwm->mav->nav_waypoint(targetLat, targetLon, targetAlt);

              // Get dynamic speed
              // Distancia real al LIDER (no wp_dist) para que el seguidor converja a la formacion.
              float distToLeader = fwm->mav->calculateDistance(
                  fwm->mav->APdata.lat, fwm->mav->APdata.lon,
                  commData.lastValidPacket.lat, commData.lastValidPacket.lon);
              uint16_t dynamic_speed = fwm->mav->calculate_dynamic_speed(commData.lastValidPacket.ground_speed, distToLeader);

              // Change speed
              fwm->mav->do_change_speed(dynamic_speed);
              #endif
            }
            else
            {
              // No es seguro seguir - el sistema ya manejó la emergencia en isSafeToFollow
              Log.warning("Safety check failed - not following" CR);
            }
          }

          // Time to get
          markBeaconReceived();
        }
        else
        {
          // Paquete inválido (checksum o datos GPS), Incrementamos
          commData.lost_packet_counter++;
        }
      }
      else
      {
        Log.warning("Unknown packet size: %d bytes" CR, packetSize);
        commData.lost_packet_counter++;
      }
    }
  }
}

void Comm::bridgeRun()
{
  if (LoRa.parsePacket())
  {
    receive_mavlink_lora();
  }
}

void Comm::receive_mavlink_lora()
{
  static mavlink_message_t message;
  static mavlink_status_t status;

  commData.rssi = LoRa.packetRssi();
  commData.snr = LoRa.packetSnr();

  while (LoRa.available() > 0)
  {
    uint8_t serial_byte = LoRa.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status))
    {
      switch (message.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT:
        //fwm->mav->send_to_fc(message);
        break;
      case MAVLINK_MSG_ID_HIGH_LATENCY2:
        fwm->mav->send_to_fc(message);
        break;
      default:
        //fwm->mav->send_to_fc(message);
        break;
      }
    }
  }
}

void Comm::send_mavlink_lora(mavlink_message_t message)
{
  static uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  static uint16_t mavlink_message_length = 0;
  mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, &message);

  LoRa.beginPacket();
  LoRa.write(mavlink_message_buffer, mavlink_message_length);
  LoRa.endPacket();
}

void Comm::sendPacket(const LoraPacket_t &packet)
{
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&packet, sizeof(packet));
  LoRa.endPacket();

  // Incrementamos
  commData.tx_packet_counter++;
}

bool Comm::validateChecksum(const LoraPacket_t &packet)
{
  return loraChecksumOk(packet);
}

uint8_t Comm::calChecksum(const LoraPacket_t &packet)
{
  return loraPacketChecksum(packet);
}

bool Comm::acceptSequence(uint16_t seq)
{
  if (!lastRxSeqInitialized)
  {
    lastRxSeq = seq;
    lastRxSeqInitialized = true;
    return true;
  }
  if (!loraSeqIsNewer(seq, lastRxSeq))
  {
    Log.trace("Duplicate/old LoRa seq=%u last=%u" CR, (unsigned)seq, (unsigned)lastRxSeq);
    return false;
  }
  const uint16_t delta = (uint16_t)(seq - lastRxSeq);
  if (delta > 1)
    commData.lost_packet_counter += (uint16_t)(delta - 1);
  lastRxSeq = seq;
  return true;
}

void Comm::applyNetid()
{
#if NETID_USE_SYNCWORD
  syncWordPending = true;
#endif
}

// v2: REPLY (ya hay enlace) o JOIN (aun no) con la posicion del seguidor -> sesion + OSD del lider.
void Comm::sendReplyPacket()
{
  if (!fwm || !fwm->mav)
  {
    return;
  }
  LoraPacket_t p;
  memset(&p, 0, sizeof(p));
  p.version = PROTOCOL_VERSION;
  p.type = commData.have_beacon ? LORA_MSG_REPLY : LORA_MSG_JOIN;
  p.netid = fwm->params.netid;
  p.sysid = SYSID;
  p.mode = (uint8_t)fwm->mav->APdata.custom_mode;
  p.seq = (uint16_t)(txSeq + 1);
  p.lat = fwm->mav->APdata.lat;
  p.lon = fwm->mav->APdata.lon;
  p.alt = fwm->mav->APdata.alt;
  p.relative_alt = fwm->mav->APdata.relative_alt;
  p.ground_speed = fwm->mav->APdata.ground_speed;
  p.hdg = fwm->mav->APdata.hdg;
  p.timestamp = millis();
  p.vx = fwm->mav->APdata.vx;
  p.vy = fwm->mav->APdata.vy;
  p.vz = fwm->mav->APdata.vz;
  p.checksum = calChecksum(p);
  if (sendPacketWithRetry(p))
    txSeq = p.seq;
}

// ============================================================================================================
// FASE 1: VALIDACIÓN COMPLETA DE PAQUETES
// ============================================================================================================

/**
 * @brief Valida completamente un paquete LoRa (checksum + datos GPS)
 * 
 * @param packet LoraPacket_t - Paquete a validar
 * @return bool - true si el paquete es válido
 */
bool Comm::validatePacket(const LoraPacket_t &packet)
{
  // 0. Version + red (netid): descarta paquetes de otras versiones/sistemas (anti-cruce).
  if (!loraVersionOk(packet))
  {
    Log.trace("Proto version mismatch (%d)" CR, packet.version);
    return false;
  }
  if (fwm->params.netid != 0 && !loraNetidOk(packet, fwm->params.netid))
  {
    Log.trace("Netid mismatch (%d != %d)" CR, packet.netid, fwm->params.netid);
    return false;
  }
  if (packet.type < LORA_MSG_BEACON || packet.type > LORA_MSG_JOIN)
  {
    Log.trace("Invalid LoRa message type (%d)" CR, packet.type);
    return false;
  }

  // 1. Validar checksum
  if (!validateChecksum(packet))
  {
    Log.warning("Invalid checksum" CR);
    return false;
  }
  
  // 2. Validar coordenadas GPS (rango válido)
  if (abs(packet.lat) > MAX_VALID_LATITUDE)
  {
    Log.warning("Invalid latitude: %d" CR, packet.lat);
    return false;
  }
  
  if (abs(packet.lon) > MAX_VALID_LONGITUDE)
  {
    Log.warning("Invalid longitude: %d" CR, packet.lon);
    return false;
  }
  
  // 3. Validar altitud razonable
  if (packet.alt < MIN_VALID_ALTITUDE || packet.alt > MAX_VALID_ALTITUDE)
  {
    Log.warning("Invalid altitude: %d mm" CR, packet.alt);
    return false;
  }
  
  // 4. Validar velocidad razonable
  if (packet.ground_speed > MAX_VALID_GROUND_SPEED)
  {
    Log.warning("Invalid ground speed: %d cm/s" CR, packet.ground_speed);
    return false;
  }
  
  // 5. Validar heading (0-36000, donde 36000 = 360.00 grados)
  if (packet.hdg > 36000 && packet.hdg != 65535) // 65535 = desconocido
  {
    Log.warning("Invalid heading: %d" CR, packet.hdg);
    return false;
  }
  
  // Paquete válido
  return true;
}

// ============================================================================================================
// FASE 2: COMPRESIÓN DE PAQUETES
// ============================================================================================================

/**
 * @brief Comprime un paquete LoRa normal a formato comprimido
 * 
 * @param packet LoraPacket_t - Paquete normal
 * @return CompressedLoraPacket_t - Paquete comprimido (15 bytes vs 27 bytes)
 */
CompressedLoraPacket_t Comm::compressPacket(LoraPacket_t packet)
{
  CompressedLoraPacket_t compressed;
  
  // System ID
  compressed.sysid = packet.sysid;
  
  // Latitud: separar parte entera (floor) y decimal.
  // IMPORTANTE: usar floor() (no truncado hacia cero) para que la fracción sea siempre [0,1).
  // Si no, las coordenadas negativas se reconstruyen mal (p. ej. -3.703 -> -3 + 0.703 = -2.297).
  // lat está en formato * 1E7, ej: 404567890 = 40.4567890°
  double lat_degrees = packet.lat / 1E7;
  compressed.lat_deg = (int8_t)floor(lat_degrees);        // Parte entera, hacia abajo
  double lat_fraction = lat_degrees - compressed.lat_deg; // [0,1)
  compressed.lat_frac = (uint8_t)(lat_fraction * 255.0);  // Mapear 0.0-1.0 a 0-255
  
  // Longitud: separar parte entera (floor) y decimal
  double lon_degrees = packet.lon / 1E7;
  compressed.lon_deg = (int16_t)floor(lon_degrees);
  double lon_fraction = lon_degrees - compressed.lon_deg; // [0,1)
  compressed.lon_frac = (uint8_t)(lon_fraction * 255.0);
  
  // Altitud relativa: convertir de mm a decímetros
  // relative_alt está en mm, convertir a dm (1 dm = 100 mm)
  compressed.relative_alt_dm = (uint16_t)(packet.relative_alt / 100);
  
  // Ground speed y heading sin cambios
  compressed.ground_speed = packet.ground_speed;
  compressed.hdg = packet.hdg;
  
  // Flags
  compressed.flags = 0x01; // Bit 0 = altitude válida
  
  // Calcular checksum
  compressed.checksum = calChecksumCompressed(compressed);
  
  return compressed;
}

/**
 * @brief Descomprime un paquete LoRa comprimido a formato normal
 * 
 * @param compressed CompressedLoraPacket_t - Paquete comprimido
 * @return LoraPacket_t - Paquete normal
 */
LoraPacket_t Comm::decompressPacket(CompressedLoraPacket_t compressed)
{
  LoraPacket_t packet;
  
  // System ID
  packet.sysid = compressed.sysid;
  
  // Latitud: reconstruir desde parte entera y decimal
  double lat_degrees = compressed.lat_deg + (compressed.lat_frac / 255.0);
  packet.lat = (int32_t)(lat_degrees * 1E7);
  
  // Longitud: reconstruir desde parte entera y decimal
  double lon_degrees = compressed.lon_deg + (compressed.lon_frac / 255.0);
  packet.lon = (int32_t)(lon_degrees * 1E7);
  
  // Altitud relativa: convertir de decímetros a mm
  packet.relative_alt = compressed.relative_alt_dm * 100;
  
  // Altitud absoluta: aproximar desde relative_alt (no disponible en compressed)
  packet.alt = packet.relative_alt; // Aproximación
  
  // Ground speed y heading sin cambios
  packet.ground_speed = compressed.ground_speed;
  packet.hdg = compressed.hdg;
  // P1: el paquete comprimido no lleva timestamp ni velocidad NED
  packet.timestamp = 0;
  packet.vx = 0;
  packet.vy = 0;
  packet.vz = 0;

  // Recalcular checksum del paquete descomprimido
  packet.checksum = calChecksum(packet);
  
  return packet;
}

/**
 * @brief Calcula checksum para paquete comprimido
 */
uint8_t Comm::calChecksumCompressed(CompressedLoraPacket_t packet)
{
  uint8_t checksum = 0;
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&packet);
  size_t size = sizeof(packet);
  
  size_t checksumIndex = offsetof(CompressedLoraPacket_t, checksum);
  for (size_t i = 0; i < size; ++i)
  {
    if (i != checksumIndex)
    {
      checksum += bytes[i];
    }
  }
  
  return checksum;
}

/**
 * @brief Valida checksum de paquete comprimido
 */
bool Comm::validateChecksumCompressed(CompressedLoraPacket_t packet)
{
  uint8_t calculated = calChecksumCompressed(packet);
  return calculated == packet.checksum;
}

// ============================================================================================================
// FASE 2: MANEJO DE COLISIONES
// ============================================================================================================

/**
 * @brief Envía un paquete con reintentos y backoff aleatorio para evitar colisiones
 * 
 * @param packet LoraPacket_t - Paquete a enviar
 * @param maxRetries uint8_t - Número máximo de intentos
 * @return bool - true si se envió exitosamente
 */
bool Comm::sendPacketWithRetry(const LoraPacket_t &packet, uint8_t maxRetries)
{
  for (uint8_t attempt = 0; attempt < maxRetries; attempt++)
  {
    // Esperar tiempo aleatorio para evitar colisiones (backoff exponencial)
    if (attempt > 0)
    {
      uint32_t backoff = random(LORA_RETRY_DELAY_MIN, LORA_RETRY_DELAY_MAX * (1 << attempt));
      delay(backoff);
      Log.notice("LoRa retry %d/%d after %d ms" CR, attempt + 1, maxRetries, backoff);
    }
    
    // Intentar enviar el paquete
    #if USE_COMPRESSED_PACKETS
      // Usar paquete comprimido
      CompressedLoraPacket_t compressed = compressPacket(packet);
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&compressed, sizeof(compressed));
      if (LoRa.endPacket())
      {
        Log.verbose("Sent compressed packet: %d bytes" CR, sizeof(compressed));
        commData.tx_packet_counter++;
        return true;
      }
    #else
      // Usar paquete normal
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&packet, sizeof(packet));
      if (LoRa.endPacket())
      {
        Log.verbose("Sent normal packet: %d bytes" CR, sizeof(packet));
        commData.tx_packet_counter++;
        return true;
      }
    #endif
    
    Log.warning("Failed to send, attempt %d/%d" CR, attempt + 1, maxRetries);
  }
  
  Log.error("Failed to send packet after %d attempts" CR, maxRetries);
  return false;
}

// ============================================================================================================
// FASE 3: CALIBRACIÓN AUTOMÁTICA
// ============================================================================================================

/**
 * @brief Mide el RSSI promedio de paquetes recibidos
 * 
 * @param samples int - Número de muestras a promediar
 * @return int - RSSI promedio
 */
int Comm::measureAverageRSSI(int samples)
{
    int totalRSSI = 0;
    int validSamples = 0;
    uint32_t startTime = millis();
    
    Log.notice("Measuring average RSSI with %d samples..." CR, samples);
    
    while (validSamples < samples && (millis() - startTime < 30000)) // Timeout 30s
    {
        if (LoRa.parsePacket())
        {
            int rssi = LoRa.packetRssi();
            totalRSSI += rssi;
            validSamples++;
            Log.verbose("Sample %d/%d: RSSI = %d dBm" CR, validSamples, samples, rssi);
            delay(100);
        }
        delay(10);
    }
    
    if (validSamples == 0)
    {
        Log.warning("No packets received for RSSI measurement" CR);
        return -999;
    }
    
    int avgRSSI = totalRSSI / validSamples;
    Log.notice("Average RSSI: %d dBm (%d samples)" CR, avgRSSI, validSamples);
    
    return avgRSSI;
}

/**
 * @brief Calibra automáticamente la potencia de transmisión LoRa para optimizar alcance/consumo
 * 
 * Prueba diferentes niveles de potencia y selecciona el óptimo basándose en RSSI
 * NOTA: Requiere que haya otro dispositivo transmitiendo para medir RSSI
 */
void Comm::autoCalibrate()
{
    Log.notice("Starting LoRa auto-calibration..." CR);
    Log.notice("NOTE: Requires another device transmitting for RSSI measurement" CR);
    
    int bestPower = LORA_TX_POWER;
    int bestRSSI = -999;
    
    // Probar diferentes niveles de potencia de recepción
    // (La potencia TX no afecta RX, pero podemos medir calidad de señal)
    
    // Medir RSSI actual
    int currentRSSI = measureAverageRSSI(5);
    
    if (currentRSSI == -999)
    {
        Log.error("Auto-calibration failed: No signal detected" CR);
        return;
    }
    
    // Probar diferentes configuraciones de spreading factor
    int originalSF = LORA_SPREADING_FACTOR;
    int bestSF = originalSF;
    int bestSFRSSI = currentRSSI;
    
    Log.notice("Testing Spreading Factors..." CR);
    
    for (int sf = 7; sf <= 12; sf++)
    {
        LoRa.setSpreadingFactor(sf);
        delay(500);
        
        int rssi = measureAverageRSSI(3);
        
        if (rssi > bestSFRSSI)
        {
            bestSFRSSI = rssi;
            bestSF = sf;
        }
        
        Log.notice("SF%d: RSSI = %d dBm" CR, sf, rssi);
    }
    
    // Restaurar spreading factor óptimo
    LoRa.setSpreadingFactor(bestSF);
    
    // Probar diferentes anchos de banda
    long originalBW = LORA_SIGNAL_BANDWIDTH;
    long testBandwidths[] = {125000, 250000, 500000}; // 125kHz, 250kHz, 500kHz
    long bestBW = originalBW;
    int bestBWRSSI = currentRSSI;
    
    Log.notice("Testing Bandwidths..." CR);
    
    for (int i = 0; i < 3; i++)
    {
        LoRa.setSignalBandwidth(testBandwidths[i]);
        delay(500);
        
        int rssi = measureAverageRSSI(3);
        
        if (rssi > bestBWRSSI)
        {
            bestBWRSSI = rssi;
            bestBW = testBandwidths[i];
        }
        
        Log.notice("BW %ld Hz: RSSI = %d dBm" CR, testBandwidths[i], rssi);
    }
    
    // Aplicar configuración óptima
    LoRa.setSpreadingFactor(bestSF);
    LoRa.setSignalBandwidth(bestBW);
    
    // Medir RSSI final
    delay(500);
    int finalRSSI = measureAverageRSSI(5);
    
    Log.notice("=== Calibration Complete ===" CR);
    Log.notice("Original: SF%d, BW %ld Hz, RSSI %d dBm" CR, originalSF, originalBW, currentRSSI);
    Log.notice("Optimized: SF%d, BW %ld Hz, RSSI %d dBm" CR, bestSF, bestBW, finalRSSI);
    Log.notice("Improvement: %d dBm" CR, finalRSSI - currentRSSI);
    
    // Log de calibración
    if (fwm->logger)
    {
        char data[128];
        snprintf(data, sizeof(data), "SF=%d,BW=%ld,RSSI=%d,improvement=%d", 
                bestSF, bestBW, finalRSSI, finalRSSI - currentRSSI);
        fwm->logger->info("LORA_CALIBRATION", data);
    }
}

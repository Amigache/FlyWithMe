#include "FWM.h"

#include <esp_idf_version.h>
#include <esp_mac.h>
#include <cstring>
#include <cstdlib>
#include "wifi_identity.h"

FWM *FWM::self = nullptr;

FWM::FWM()
{
    self = this;
}

void FWM::begin()
{
    // FASE 1: Configurar watchdog (30 segundos timeout)
    Log.notice("Configuring Watchdog Timer (30s timeout)" CR);
#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    // ESP-IDF >= 5 (Arduino core 3.x): API basada en configuración.
    // El core puede haber inicializado ya el WDT, por lo que se intenta reconfigurar.
    esp_task_wdt_config_t wdt_config = {};
    wdt_config.timeout_ms = 30000;
    wdt_config.idle_core_mask = 0;
    wdt_config.trigger_panic = true;
    if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK)
    {
        esp_task_wdt_init(&wdt_config);
    }
#else
    // ESP-IDF < 5 (Arduino core 2.x)
    esp_task_wdt_init(30, true);
#endif
    esp_task_wdt_add(NULL);
    
    // resetParams();

    if (!existParams())
    {
        Log.notice("Params data not exist" CR);
        Log.notice("Saving default" CR);

        // Default params
        params.role = FWM_DEFAULT_ROLE;
        params.ap_mode = FWM_DEFAULT_AP_MODE;
        params.foll_enable = FOLL_ENABLE;
        params.foll_ofs_type = FOLL_OFS_TYPE;
        params.foll_alt_type = FOLL_ALT_TYPE;
        params.link_timeout = LINK_TIMEOUT;
        strncpy(params.ssid, DEFAULT_SSID, sizeof(params.ssid));
        strncpy(params.pass, DEFAULT_PASS, sizeof(params.pass));
        params.dist_offset = DIST_OFFSET;
        params.lateral_offset = FORMATION_LATERAL_OFFSET;
        params.vertical_offset = FORMATION_VERTICAL_OFFSET;
        params.cross_gain = CROSS_TRACK_GAIN_DEG_PER_M;
        params.heading_corr_max = MAX_HEADING_CORR_DEG;
        params.along_gain = ALONG_GAIN_CMS_PER_M;
        params.netid = NETID_DEFAULT;
        params.approach_dist = APPROACH_DIST_DEFAULT;

        saveParams();
    }
    else
    {
        // load saved params
        loadParams();
    }

    // Versiones anteriores no almacenaban el rol. En ese caso se adopta OFF de forma segura.
    if (params.role < FWM_ROLE_OFF || params.role > FWM_ROLE_LEADER)
    {
        params.role = FWM_ROLE_OFF;
    }
    logDeviceIdentity();

    // Init instances

    screen = new Screen(this);
    screen->begin();

    comm = new Comm(this);
    comm->begin();

    mav = new Telem(this);
    mav->begin();

    // El parámetro role selecciona el comportamiento persistente de esta placa.
    if (!MAV_BRIDGE)
        changeFollowMode((uint8_t)params.role);

    // Restaurar config guardada (formación, predicción, filtro)
    preferences.begin("storage", true);
    int savedFormation = preferences.getInt("formation", DEFAULT_FORMATION);
    bool savedPrediction = preferences.getBool("prediction", USE_PREDICTION);
    bool savedFilter = preferences.getBool("filter", USE_POSITION_FILTER);
    preferences.end();
    if (savedFormation >= 0 && savedFormation <= 4)
    {
        mav->currentFormation = (FormationType)savedFormation;
    }
    mav->predictionEnabled = savedPrediction;
    mav->filterEnabled = savedFilter;

    web = new Web(this);
    
    // FASE 2: Inicializar logger
    logger = new Logger();
    logger->init();
    logger->info("FWM_INIT", "System initialized");
    
    // FASE 4: Inicializar WiFi AP y servidor web (si está habilitado)
    #if WEB_START_AP_IMMEDIATELY
    // AP solo si la preferencia lo permite y la comprobación segura confirma tierra.
    if (params.ap_mode != FWM_AP_MODE_OFF && canChangeApMode())
    {
        web->startAP();
        delay(500);
    }
    #endif
    
    #if USE_WEB_SERVER
    // Luego inicializar servidor web (requiere WiFi activo)
    web->begin();
    #endif

    // FASE 1: Inicializar máquina de estados
    transitionState(STATE_SEARCHING);

    Log.info("FWM Ready" CR);
}

/**
 * @brief Main loop
 */
void FWM::run()
{
    runRt();
    runIo();
}

/**
 * @brief Núcleo 0 (UI/logging): AP gate, web, pantalla y logger. No critico para el vuelo.
 */
void FWM::runIo()
{
    processSerialProvisioning();

    if (roleRestartPending && (int32_t)(millis() - roleRestartAtMs) >= 0)
    {
        Serial.flush();
        delay(100);
        ESP.restart();
    }

    // AP/WiFi solo en tierra: comprobar periodicamente y levantar/apagar segun corresponda
    static uint32_t lastApGate = 0;
    if (millis() - lastApGate > WEB_AP_GATE_INTERVAL_MS)
    {
        updateApGate();
        lastApGate = millis();
    }

    // FASE 2: Log periódico de telemetría (cada 30 segundos)
    static uint32_t lastTelemetryLog = 0;
    if (logger && (millis() - lastTelemetryLog > 30000))
    {
        logger->logTelemetry(mav->APdata);
        lastTelemetryLog = millis();
    }

    // A4: métricas de enlace periódicas (RSSI/SNR, pérdidas, distancia)
    static uint32_t lastLinkLog = 0;
    if (millis() - lastLinkLog > LINK_METRICS_LOG_INTERVAL_MS)
    {
        int rx = (int)comm->commData.rx_packet_counter;
        int lost = (int)comm->commData.lost_packet_counter;
        int lossPct = (rx + lost) > 0 ? (lost * 100) / (rx + lost) : 0;
        int dist = (int)getLinkDistance();
        Log.notice("Link: %s rssi=%d snr=%d rx=%d tx=%d lost=%d (%dpct) dist=%dm" CR,
                   getStateName(currentState),
                   comm->commData.rssi, comm->commData.snr,
                   rx, (int)comm->commData.tx_packet_counter,
                   lost, lossPct, dist);
        lastLinkLog = millis();
    }

    screen->run();
    web->run();
}

/**
 * @brief Núcleo 1 (tiempo real): LoRa + MAVLink con el FC + maquina de estados + mensajeria.
 *        Todo el acceso a Telem (puerto del FC) queda en este mismo core.
 */
void FWM::runRt()
{
    // FASE 1: Reset watchdog en cada ciclo (el loopTask de Arduino corre en el core 1)
    esp_task_wdt_reset();

#if FWM_ALLOW_RUNTIME_SITL
    // En SIM la consola serie queda silenciada para no mezclar texto con MAVLink. Publicar
    // contadores estructurados por MAVLink para que el bench mida RX/TX/sesión por el TAP.
    static uint32_t lastRuntimeDiagnostics = 0;
    const uint32_t diagnosticsNow = millis();
    if (runtimeSitlMode && mav && comm && diagnosticsNow - lastRuntimeDiagnostics >= 1000)
    {
        lastRuntimeDiagnostics = diagnosticsNow;
        const bool linkActive = params.role == FWM_ROLE_FOLLOWER
                                    ? comm->commData.have_beacon
                                    : (params.role == FWM_ROLE_LEADER && lastFollowerMs != 0 &&
                                       diagnosticsNow - lastFollowerMs < SESSION_TIMEOUT_MS);
        auto sendNamedInt = [this, diagnosticsNow](const char *name, int32_t value)
        {
            mavlink_message_t message;
            mavlink_msg_named_value_int_pack(fwmSystemId(), COMPID, &message,
                                             diagnosticsNow, name, value);
            mav->send_to_fc(message);
        };
        sendNamedInt("FWM_RX", (int32_t)comm->commData.rx_packet_counter);
        sendNamedInt("FWM_TX", (int32_t)comm->commData.tx_packet_counter);
        sendNamedInt("FWM_LINK", linkActive ? 1 : 0);
    }
#endif

    // Mensajeria (seguidor): notificar distancia de seguimiento al FC/GCS periodicamente
    static uint32_t lastFollowStatus = 0;
    static int lastFollowAnnouncedDist = -1;
    if (follow_mode == FOLL_MODE_FOLLOWER && comm->commData.have_beacon &&
        (millis() - lastFollowStatus > STATUS_DISTANCE_INTERVAL_MS))
    {
        int dist = (int)getLinkDistance();
        const char *trend = " holding";
        if (lastFollowAnnouncedDist >= 0 && dist < lastFollowAnnouncedDist - 5) trend = " approaching";
        else if (lastFollowAnnouncedDist >= 0 && dist > lastFollowAnnouncedDist + 5) trend = " falling behind";
        if (dist >= 0 && (lastFollowAnnouncedDist < 0 || abs(dist - lastFollowAnnouncedDist) >= 5))
        {
            char s[48];
            snprintf(s, sizeof(s), "Follow %dm%s", dist, trend);
            mav->status_text(s); // INFO: distancia de seguimiento, no es una alerta de fallo
            lastFollowAnnouncedDist = dist;
        }
        lastFollowStatus = millis();
    }
    else if (follow_mode != FOLL_MODE_FOLLOWER || !comm->commData.have_beacon)
    {
        lastFollowAnnouncedDist = -1; // al recuperar enlace, anunciar de nuevo
    }

    // v2 (Idea 1): OSD del LIDER -> dice a su FC quien le sigue y a que distancia
    static uint32_t lastLeadStatus = 0;
    static int lastLeadAnnouncedDist = -1;
    static bool followerWasPresent = false;
    bool followerPresent = follow_mode == FOLL_MODE_LEADER && lastFollowerMs != 0 &&
                          (millis() - lastFollowerMs) < SESSION_TIMEOUT_MS;
    if (followerPresent && (millis() - lastLeadStatus > STATUS_DISTANCE_INTERVAL_MS))
    {
        int dist = lastFollowerDistM;
        if (dist >= 0 && (lastLeadAnnouncedDist < 0 || abs(dist - lastLeadAnnouncedDist) >= 5))
        {
            char s[40];
            // Telem::status_text ya añade el prefijo "FWM: ".
            snprintf(s, sizeof(s), "Follower %dm", dist);
            mav->status_text(s); // INFO: distancia del seguidor
            lastLeadAnnouncedDist = dist;
        }
        followerWasPresent = true;
        lastLeadStatus = millis();
    }
    else if (!followerPresent)
    {
        if (followerWasPresent && follow_mode == FOLL_MODE_LEADER)
            mav->status_text("Follower link lost", MAV_SEVERITY_WARNING);
        followerWasPresent = false;
        lastLeadAnnouncedDist = -1;
    }
    
    // A2: recuperación tras emergencia (histéresis: se reintenta pasado el cooldown)
    if (currentState == STATE_EMERGENCY && (millis() - stateEntryTime) > EMERGENCY_RECOVERY_MS)
    {
        transitionState(STATE_SEARCHING);
    }

    // State machine
    if (follow_mode == FOLL_MODE_FOLLOWER) // Only work if we are on follower mode
    {
        if (mav->APdata.custom_mode == MODE_GUIDED) // Only work if we are on guided mode
        {
            if (comm->commData.have_beacon && stage_follow == STAGE_IDLE)
            {
                stage_follow = STAGE_APPROACH;
                // FASE 1: Transición a estado FOLLOWING
                transitionState(STATE_FOLLOWING);
            }
            else if (!comm->commData.have_beacon && stage_follow == STAGE_APPROACH)
            {
                stage_follow = STAGE_IDLE;
                // FASE 1: Transición a estado LOST_LINK
                transitionState(STATE_LOST_LINK);
            }
        }
        else if (stage_follow == STAGE_APPROACH) // Mode changed, stop follow
        {
            stage_follow = STAGE_IDLE;
            transitionState(STATE_SEARCHING);
        }
    }
    else
    {
        stage_follow = STAGE_IDLE; // Not in follower mode
    }

    // Run instances: toda operacion SX1276 se serializa en loopTask/core1.
    comm->run();
    processSendPacket();
    mav->run();
}

/**
 * @brief Bridge run
 */
void FWM::bridgeRun()
{
    comm->bridgeRun();
    mav->bridgeRun();
    screen->bridgeRun();
}

/**
 * @brief Send packet ticker callback
 * 
 * Send packet to follower
 * 
 * @param void
 * @return void
 * 
 * @note This function is called by a Ticker
 * 
 */
// v2: el lider recibe un REPLY/JOIN del seguidor -> activa la sesion y registra la distancia (OSD).
void FWM::onFollowerReply(const LoraPacket_t &p)
{
    lastFollowerMs = millis();
    replyWindowUntilMs = lastFollowerMs; // respuesta recibida: cerrar la ventana y poder transmitir
    if (mav && mav->positionValid && loraPositionOk(p))
    {
        lastFollowerDistM = (int)mav->calculateDistance(mav->APdata.lat, mav->APdata.lon, p.lat, p.lon);
    }
    else
    {
        lastFollowerDistM = -1; // sesión válida, pero aún no hay dos posiciones válidas para medir
    }
}

void FWM::send_packet_ticker_callback()
{
    if (self)
    {
        // El callback del Ticker NO toca SPI/LoRa. Solo deja una marca; el loop propietario del
        // SX1276 procesa RX/TX secuencialmente en processSendPacket().
        self->beaconDue = true;
    }
}

void FWM::processSendPacket()
{
    if (!beaconDue || follow_mode != FOLL_MODE_LEADER)
        return;

    uint32_t now = millis();
#if FOLLOWER_REPLY
    // Mientras esperamos un REPLY, no transmitir sobre la ventana RX del líder.
    if ((int32_t)(now - replyWindowUntilMs) < 0)
        return;

    bool sessionActive = lastFollowerMs != 0 && (now - lastFollowerMs) < SESSION_TIMEOUT_MS;
    static uint32_t lastDiscoveryMs = 0;
    if (!sessionActive && lastFollowerMs != 0 && comm->lastRxSeqInitialized)
    {
        // El seguidor puede reiniciar su contador seq; después del timeout aceptar un nuevo JOIN.
        comm->lastRxSeqInitialized = false;
    }
    if (!sessionActive && (now - lastDiscoveryMs) < DISCOVERY_INTERVAL_MS)
        return; // conservar beaconDue para enviar el siguiente discovery al vencer el timeout
#endif

    beaconDue = false;
    if (!mav || !comm || mav->linkTimeout || !mav->positionValid)
    {
        Log.trace("Packet send skipped: No FC connection (AP config mode)" CR);
        return;
    }

    LoraPacket_t packet = {};
    packet.version = PROTOCOL_VERSION;
    packet.type = LORA_MSG_BEACON;
    packet.netid = params.netid;
    packet.mode = (uint8_t)mav->APdata.custom_mode;
    packet.sysid = fwmSystemId();
    packet.seq = (uint16_t)(comm->txSeq + 1);
    packet.flags = mav->positionValid ? LORA_FLAG_POSITION_VALID : LORA_FLAG_NONE;
#if FOLLOWER_REPLY
    bool requestReply = !sessionActive || (now - lastReplyRequestMs >= FOLLOWER_REPLY_MS);
    if (requestReply)
        packet.flags |= LORA_FLAG_REPLY_SLOT;
#endif
    packet.lat = mav->APdata.lat;
    packet.lon = mav->APdata.lon;
    packet.alt = mav->APdata.alt;
    packet.relative_alt = mav->APdata.relative_alt;
    packet.ground_speed = mav->APdata.ground_speed;
    packet.hdg = mav->APdata.hdg;
    packet.timestamp = millis();
    packet.vx = mav->APdata.vx;
    packet.vy = mav->APdata.vy;
    packet.vz = mav->APdata.vz;
    packet.checksum = comm->calChecksum(packet);

    if (comm->sendPacketWithRetry(packet))
    {
        comm->txSeq = packet.seq;
#if FOLLOWER_REPLY
        if (requestReply)
        {
            lastReplyRequestMs = millis();
            replyWindowUntilMs = lastReplyRequestMs + REPLY_WINDOW_MS;
        }
        if (!sessionActive)
            lastDiscoveryMs = millis();
#endif
        Log.notice("Send Packet: sys=%d seq=%d lat=%d lon=%d alt=%d" CR,
                   packet.sysid, packet.seq, packet.lat, packet.lon, packet.relative_alt);
        if (logger)
            logger->logPacket(packet, 0, 0);
    }
    else
    {
        Log.error("Failed to send packet after retries" CR);
    }

    if (adaptiveRateEnabled)
        updateTransmissionRate();
}

/**
 * @brief Change follow mode
 * 
 * @param mode uint8_t
 * @return void
 * 
 */
void FWM::changeFollowMode(uint8_t mode)
{
    follow_mode = mode;

    // Mode off or follower no need to send packets
    if (follow_mode == FOLL_MODE_OFF || follow_mode == FOLL_MODE_FOLLOWER)
    {
        // Stop ticker
        send_packet_ticker.detach();
    }
    else if (follow_mode == FOLL_MODE_LEADER)
    {
        // Start ticker
        send_packet_ticker.attach_ms(sendPacketIntervalMs, FWM::send_packet_ticker_callback);
    }
}

uint8_t FWM::fwmSystemId() const
{
    if (params.role == FWM_ROLE_LEADER)
        return FWM_LEADER_SYSID;
    if (params.role == FWM_ROLE_FOLLOWER)
        return FWM_FOLLOWER_SYSID;
    return FWM_SETUP_SYSID;
}

uint8_t FWM::targetSystemId() const
{
    if (params.role == FWM_ROLE_FOLLOWER)
        return FWM_FOLLOWER_SYSID;
    if (params.role == FWM_ROLE_OFF && mav != nullptr && mav->autopilotSystemId != 0)
        return mav->autopilotSystemId;
    // En modo OFF, antes de detectar un heartbeat, usar el SYSID histórico del líder.
    return FWM_LEADER_SYSID;
}

uint8_t FWM::peerSystemId() const
{
    if (params.role == FWM_ROLE_LEADER)
        return FWM_FOLLOWER_SYSID;
    if (params.role == FWM_ROLE_FOLLOWER)
        return FWM_LEADER_SYSID;
    return 0;
}

bool FWM::enableRuntimeSitlMode(String &error)
{
#if FWM_ALLOW_RUNTIME_SITL && !FC_LINK_USB && !FC_EMULATION
    if (runtimeSitlMode)
        return true;
    if (mav == nullptr || mav->link || mav->lock_ap)
    {
        error = "SIM requiere ausencia de FC desde el arranque; un enlace previo bloquea la conmutación";
        return false;
    }
    if (!mav->linkTimeout)
    {
        error = "WAIT: esperando timeout inicial de FC antes de habilitar SIM";
        return false;
    }
    if (!canChangeApMode())
    {
        error = "SIM bloqueado por el interlock de tierra";
        return false;
    }
    if (mav == nullptr || comm == nullptr)
    {
        error = "telemetría o LoRa todavía no inicializados";
        return false;
    }

    // Responder antes de convertir UART0 a MAVLink y silenciar la consola. Así el host
    // recibe el ACK completo antes de que el SITL/bridge empiece a mandar bytes binarios.
    Serial.println("SIMCFG OK mode=sim; reset returns to flight");
    Serial.flush();
    Log.begin(LOG_LEVEL_SILENT, &Serial);

    adaptiveRateEnabled = false;
    sendPacketIntervalMs = FWM_SITL_SEND_PACKET_INTERVAL;
    currentTransmissionInterval = FWM_SITL_SEND_PACKET_INTERVAL;
    if (follow_mode == FOLL_MODE_LEADER)
    {
        send_packet_ticker.detach();
        send_packet_ticker.attach_ms(sendPacketIntervalMs, FWM::send_packet_ticker_callback);
    }
    comm->requestRuntimeSitlProfile();
    mav->enableRuntimeSitlUsb();
    runtimeSitlMode = true; // volátil: un reset siempre vuelve a UART1/modo vuelo
    error = "";
    return true;
#else
    error = "modo runtime SITL bloqueado en este firmware";
    return false;
#endif
}

void FWM::logDeviceIdentity(bool includeRadio)
{
    uint8_t apMac[6] = {};
    char macText[18] = {};
    char generatedSsid[sizeof(params.ssid)] = {};
    if (esp_read_mac(apMac, ESP_MAC_WIFI_SOFTAP) != ESP_OK ||
        !fwmWifiIdentityFromMac(apMac, macText, sizeof(macText), generatedSsid, sizeof(generatedSsid)))
    {
        Serial.println("FWM_ID_ERR unable to read SoftAP MAC");
        return;
    }

    strncpy(params.ssid, generatedSsid, sizeof(params.ssid) - 1);
    params.ssid[sizeof(params.ssid) - 1] = '\0';
    const char *roleName = params.role == FWM_ROLE_LEADER ? "leader" :
                           params.role == FWM_ROLE_FOLLOWER ? "follower" : "off";
    // Línea estable, independiente del nivel DEBUG_MODE, para que GUI/herramientas puedan
    // identificar la placa incluso en la imagen normal de vuelo. `band` se añade al final: si
    // dos placas no coinciden no hay enlace ni error visible, y comparar esta línea en ambas es
    // la forma rápida de detectarlo.
    // El estado del radio SOLO se incluye bajo demanda ("FWM ID"): el banner de arranque se llama
    // antes de Comm::begin(), donde radioHealthy todavía vale false, y publicarlo ahí daría un
    // "radio=FAIL" falso.
    if (includeRadio)
    {
        Serial.printf("FWM_ID ap_mac=%s ap_ssid=\"%s\" role=%s sysid=%u band=%s radio=%s\r\n",
                      macText, params.ssid, roleName, (unsigned)fwmSystemId(),
                      loraBandLabel(params.band),
                      (comm != nullptr && comm->radioOk()) ? "ok" : "FAIL");
    }
    else
    {
        Serial.printf("FWM_ID ap_mac=%s ap_ssid=\"%s\" role=%s sysid=%u band=%s\r\n",
                      macText, params.ssid, roleName, (unsigned)fwmSystemId(),
                      loraBandLabel(params.band));
    }
}

void FWM::requestRestart()
{
    roleRestartAtMs = millis() + 1200;
    roleRestartPending = true;
}

void FWM::processSerialProvisioning()
{
#if !FC_LINK_USB
    if (runtimeSitlMode)
        return; // UART0 queda dedicado al MAVLink del SITL mientras dure esta sesión.

    static String line;
    while (Serial.available() > 0)
    {
        const char ch = (char)Serial.read();
        if (ch == '\r')
            continue;
        if (ch != '\n')
        {
            if (line.length() < 48)
                line += ch;
            else
                line = "";
            continue;
        }

        line.trim();
        if (line == "FWM ID")
        {
            // Aqui el radio ya esta inicializado, asi que el estado si es fiable.
            logDeviceIdentity(true);
        }
        else if (line == "FWM SIM ON")
        {
            String error;
            if (!enableRuntimeSitlMode(error))
            {
                Serial.print(error.startsWith("WAIT:") ? "SIMCFG WAIT " : "SIMCFG ERR ");
                Serial.println(error);
            }
        }
        else if (line.startsWith("FWM ROLE "))
        {
            String requested = line.substring(9);
            requested.trim();
            int role = -1;
            if (requested.equalsIgnoreCase("off"))
                role = FWM_ROLE_OFF;
            else if (requested.equalsIgnoreCase("follower"))
                role = FWM_ROLE_FOLLOWER;
            else if (requested.equalsIgnoreCase("leader"))
                role = FWM_ROLE_LEADER;

            if (role < 0)
            {
                Serial.println("ROLECFG ERR invalid role (off|leader|follower)");
            }
            else if (params.role != role && !canChangeRole())
            {
                Serial.println("ROLECFG WAIT FC must be connected, disarmed and on ground");
            }
            else if (role == FWM_ROLE_LEADER && !canBecomeLeader())
            {
                Serial.println("ROLECFG ERR LoRa radio did not respond; cannot become LEADER");
            }
            else
            {
                const bool roleChanged = params.role != role;
                String value = String(role);
                String error;
                if (!setParamByKey("role", value.c_str(), error))
                {
                    Serial.print("ROLECFG ERR ");
                    Serial.println(error);
                }
                else
                {
                    Serial.print("ROLECFG OK role=");
                    Serial.print(requested);
                    Serial.println(roleChanged ? " rebooting" : " already set");
                }
            }
        }
        else if (line.startsWith("FWM AP "))
        {
            String requested = line.substring(7);
            requested.trim();
            int mode = -1;
            if (requested.equalsIgnoreCase("auto"))
                mode = FWM_AP_MODE_AUTO;
            else if (requested.equalsIgnoreCase("on"))
                mode = FWM_AP_MODE_ON;
            else if (requested.equalsIgnoreCase("off"))
                mode = FWM_AP_MODE_OFF;

            if (mode < 0)
            {
                Serial.println("APCFG ERR invalid mode (auto|on|off)");
            }
            else if (params.ap_mode != mode && !canChangeApMode())
            {
                Serial.println("APCFG WAIT FC must be connected, disarmed and on ground");
            }
            else
            {
                const bool changed = params.ap_mode != mode;
                String value = String(mode);
                String error;
                if (!setParamByKey("ap_mode", value.c_str(), error))
                {
                    Serial.print("APCFG ERR ");
                    Serial.println(error);
                }
                else
                {
                    Serial.print("APCFG OK mode=");
                    Serial.print(requested);
                    Serial.println(changed ? " saved" : " already set");
                }
            }
        }
        else if (line.startsWith("FWM BAND "))
        {
            String requested = line.substring(9);
            requested.trim();
            int band = -1;
            // Acepta tanto el indice (0|1|2) como la frecuencia habitual en MHz, que es lo
            // que se escribe de memoria en el banco.
            if (requested.equalsIgnoreCase("433"))
                band = FWM_BAND_433;
            else if (requested.equalsIgnoreCase("868"))
                band = FWM_BAND_868;
            else if (requested.equalsIgnoreCase("915") || requested.equalsIgnoreCase("900"))
                band = FWM_BAND_900;
            else
            {
                int asIndex = requested.toInt();
                if (requested.length() > 0 && loraBandValid(asIndex))
                    band = asIndex;
            }

            if (band < 0)
            {
                Serial.println("BANDCFG ERR invalid band (433|868|915 or 0|1|2)");
            }
            else if (params.band != band && !canWriteConfig())
            {
                Serial.println("BANDCFG WAIT FC must be connected, disarmed and on ground");
            }
            else
            {
                const bool changed = params.band != band;
                String value = String(band);
                String error;
                if (!setParamByKey("band", value.c_str(), error))
                {
                    Serial.print("BANDCFG ERR ");
                    Serial.println(error);
                }
                else
                {
                    Serial.print("BANDCFG OK band=");
                    Serial.print(loraBandLabel(band));
                    Serial.print("MHz (");
                    Serial.print(band);
                    Serial.println(changed ? ") saved" : ") already set");
                }
            }
        }
        line = "";
        if (runtimeSitlMode)
            return;
    }
#endif
}

/**
 * @brief Reset params
 * 
 * @param void
 * @return void
 * 
 */
void FWM::resetParams()
{
    preferences.begin("storage", false);
    preferences.clear();
    preferences.end();
}

/**
 * @brief Check if exist params
 * 
 * @param void
 * @return bool
 * 
 */
bool FWM::existParams()
{
    // Try to read some preferences
    preferences.begin("storage", true); // Modo lectura
    int foll_enable = preferences.getInt("foll_enable", -1);
    int foll_ofs_type = preferences.getInt("foll_ofs_type", -1);
    String ssid = preferences.getString("ssid", "");
    preferences.end();

    // If have default set values, the preferences don't exist
    if (foll_enable == -1 || foll_ofs_type == -1 || ssid == "")
    {
        return false;
    }

    return true;
}

/**
 * @brief Save params
 * 
 * @param void
 * @return void
 * 
 */
void FWM::saveParams()
{
    preferences.begin("storage", false);
    preferences.putInt("role", params.role);
    preferences.putInt("ap_mode", params.ap_mode);
    preferences.putInt("foll_enable", params.foll_enable);
    preferences.putInt("foll_ofs_type", params.foll_ofs_type);
    preferences.putInt("foll_alt_type", params.foll_alt_type);
    preferences.putInt("link_timeout", params.link_timeout);
    preferences.putString("ssid", params.ssid);
    preferences.putString("pass", params.pass);
    // Fase 2: offsets y ganancias
    preferences.putFloat("dist_offset", params.dist_offset);
    preferences.putFloat("lateral_offset", params.lateral_offset);
    preferences.putFloat("vertical_offset", params.vertical_offset);
    preferences.putFloat("cross_gain", params.cross_gain);
    // Clave corta por el limite de 15 caracteres de ESP32 NVS (ver nota en loadParams).
    preferences.putFloat("hdg_corr_max", params.heading_corr_max);
    preferences.putFloat("along_gain", params.along_gain);
    preferences.putInt("netid", (int)params.netid);
    preferences.putFloat("approach_dist", params.approach_dist);
    preferences.putInt("band", params.band);
    if (mav != nullptr)
    {
        preferences.putInt("formation", (int)mav->currentFormation);
        preferences.putBool("prediction", mav->predictionEnabled);
        preferences.putBool("filter", mav->filterEnabled);
    }
    preferences.end();

    Log.notice("Params saved" CR);
}

/**
 * @brief Load params
 * 
 * @param void
 * @return void
 * 
 */
void FWM::loadParams()
{
    preferences.begin("storage", true);
    params.role = preferences.getInt("role", FWM_DEFAULT_ROLE);
    params.ap_mode = preferences.getInt("ap_mode", FWM_DEFAULT_AP_MODE);
    params.foll_enable = preferences.getInt("foll_enable", 0);
    params.foll_ofs_type = preferences.getInt("foll_ofs_type", 0);
    params.foll_alt_type = preferences.getInt("foll_alt_type", 0);
    // `0` provenía del default de versiones antiguas; check_link() lo interpretaría como
    // timeout inmediato y detendría el heartbeat antes de que SITL llegara por el bridge.
    const int savedLinkTimeout = preferences.getInt("link_timeout", LINK_TIMEOUT);
    params.link_timeout = (savedLinkTimeout >= 2 && savedLinkTimeout <= 120)
                              ? savedLinkTimeout : LINK_TIMEOUT;
    // Fase 2: offsets y ganancias (defaults = #define de config.h)
    params.dist_offset = preferences.getFloat("dist_offset", DIST_OFFSET);
    params.lateral_offset = preferences.getFloat("lateral_offset", FORMATION_LATERAL_OFFSET);
    params.vertical_offset = preferences.getFloat("vertical_offset", FORMATION_VERTICAL_OFFSET);
    params.cross_gain = preferences.getFloat("cross_gain", CROSS_TRACK_GAIN_DEG_PER_M);
    // OJO: clave NVS de 16 caracteres. El limite de ESP32 NVS es 15, asi que esta clave NUNCA
    // se guardo ni se leyo: nvs_set_blob/nvs_get_blob fallan con KEY_TOO_LONG/NOT_FOUND y el
    // parametro se perdia en cada reinicio aunque la WebUI dijera "guardado". Se usa el mismo
    // nombre corto que el parametro MAVLink (hdg_corr_max).
    params.heading_corr_max = preferences.getFloat("hdg_corr_max", MAX_HEADING_CORR_DEG);
    params.along_gain = preferences.getFloat("along_gain", ALONG_GAIN_CMS_PER_M);
    params.netid = (uint16_t)preferences.getInt("netid", NETID_DEFAULT);
    params.approach_dist = preferences.getFloat("approach_dist", APPROACH_DIST_DEFAULT);
    // Banda: default de compilacion (LORA_BAND) si la placa nunca la guardo. Un valor fuera de
    // rango (NVS corrupto o de una version futura) se cae al default en vez de arrancar mal.
    const int savedBand = preferences.getInt("band", FWM_DEFAULT_BAND);
    params.band = loraBandValid(savedBand) ? (int32_t)savedBand : FWM_DEFAULT_BAND;

    String ssid = preferences.getString("ssid", "");
    String pass = preferences.getString("pass", "");

    strncpy(params.ssid, ssid.c_str(), sizeof(params.ssid) - 1);
    params.ssid[sizeof(params.ssid) - 1] = '\0';
    strncpy(params.pass, pass.c_str(), sizeof(params.pass) - 1);
    params.pass[sizeof(params.pass) - 1] = '\0';
    if (!fwmApPassphraseValid(params.pass))
    {
        // Clave vacía o corrupta: volver a la de fábrica para que el AP pueda levantarse y cambiarse.
        strncpy(params.pass, DEFAULT_PASS, sizeof(params.pass) - 1);
        params.pass[sizeof(params.pass) - 1] = '\0';
    }

    preferences.end();

    Log.notice("Params loaded" CR);
}

/**
 * @brief Cambia la formación del seguidor en caliente y la persiste en NVS.
 */
void FWM::setFormation(uint8_t idx)
{
    if (idx > 4)
    {
        idx = 0;
    }
    setParamByIndex(0, (float)idx, true);
    Log.notice("Formation set to %d" CR, (int)idx);
}

/**
 * @brief Activa/desactiva la predicción en caliente y la persiste.
 */
void FWM::setPrediction(bool on)
{
    setParamByIndex(7, on ? 1.0f : 0.0f, true);
    Log.notice("Prediction %s" CR, on ? "ON" : "OFF");
}

/**
 * @brief Activa/desactiva el filtro de posición en caliente y lo persiste.
 */
void FWM::setFilter(bool on)
{
    setParamByIndex(8, on ? 1.0f : 0.0f, true);
    Log.notice("Filter %s" CR, on ? "ON" : "OFF");
}

/**
 * @brief ¿Esta el vehiculo en tierra? Habilita el AP/WiFi de configuracion solo en tierra.
 * Sin FC (o sin telemetria) se asume tierra, para poder configurar en banco/montaje.
 */
bool FWM::isOnGround()
{
#if WEB_AP_FORCE
    return true;
#elif FC_EMULATION || SIMULATION_MODE
    // El FC sintético no representa un estado de vuelo físico: permitir provisión/configuración.
    return true;
#else
    if (mav == nullptr)
    {
        return true;
    }
    // Sin enlace con el FC (linkTimeout o link caido): asumir tierra (configuracion en banco)
    if (mav->linkTimeout || !mav->link)
    {
        return true;
    }
    if (mav->APdata.armed)
    {
        return false;
    }
    if (mav->APdata.ground_speed > WEB_AP_GS_MAX_CMS)
    {
        return false;
    }
    if (mav->APdata.relative_alt > WEB_AP_ALT_MAX_MM)
    {
        return false;
    }
    return true;
#endif
}

/**
 * @brief ¿Puede la placa asumir el papel de LIDER?
 *
 * Un lider sin radio util no cumple su funcion y el fallo es silencioso: el seguidor simplemente
 * nunca encuentra beacon y nadie sabe si es la banda, el enlace o la placa. Se exige que el SX1276
 * haya respondido al begin(). Es el unico punto donde un mapa de pines equivocado (imagen de otra
 * variante de placa) se detecta en lugar de fallar en silencio.
 */
bool FWM::canBecomeLeader() const
{
    return comm != nullptr && comm->radioOk();
}

bool FWM::canChangeRole() const
{
#if FC_EMULATION || SIMULATION_MODE
    return true;
#else
    // Una placa nueva/OFF no emite órdenes de vuelo y puede provisionarse antes de conocer su FC.
    // Con un rol activo, la pérdida del enlace nunca cuenta como prueba de estar en tierra.
    if (params.role == FWM_ROLE_OFF &&
        (mav == nullptr || mav->autopilotSystemId == 0) &&
        (mav == nullptr || !mav->link || mav->linkTimeout))
        return true;
    return mav != nullptr && mav->link && !mav->linkTimeout && mav->positionValid && !mav->APdata.armed &&
           mav->APdata.ground_speed <= WEB_AP_GS_MAX_CMS &&
           mav->APdata.relative_alt <= WEB_AP_ALT_MAX_MM;
#endif
}

bool FWM::canChangeApMode() const
{
    // Si nunca se conectó un FC y expiró la búsqueda inicial, se trata como banco/setup.
    // Si el FC llegó a conectar, lock_ap queda enclavado y una pérdida posterior del enlace
    // nunca se interpreta como tierra (evita reactivar WiFi por un fallo en vuelo).
    if (mav != nullptr && !mav->lock_ap && !mav->link && mav->linkTimeout)
        return true;
    return canChangeRole();
}

/**
 * @brief Permiso para escribir configuración (parámetros groundOnly, /api/config y PARAM_SET).
 *
 * Fail-closed: a diferencia de isOnGround(), sin enlace con el FC no se considera tierra salvo que
 * nunca se haya conectado (banco). Así, si el enlace cae en vuelo, la configuración queda bloqueada.
 */
bool FWM::canWriteConfig() const
{
#if WEB_AP_FORCE || FC_EMULATION || SIMULATION_MODE
    return true;
#else
    return canChangeApMode();
#endif
}

bool FWM::apPassIsDefault() const
{
    return strcmp(params.pass, DEFAULT_PASS) == 0;
}

bool FWM::apPassChangeRequired() const
{
#if FWM_FORCE_AP_PASS_CHANGE
    return apPassIsDefault();
#else
    return false;
#endif
}

/**
 * @brief Cambia la clave WiFi (WPA2) en tierra y reinicia la placa para aplicarla.
 */
bool FWM::setApPassphrase(const char *pass, String &err)
{
    if (!canChangeApMode())
    {
        err = "clave WiFi editable solo con FC conectado, desarmado y en tierra";
        return false;
    }
    if (!fwmApPassphraseValid(pass))
    {
        err = "la clave debe tener entre 8 y 63 caracteres ASCII imprimibles";
        return false;
    }
    if (strcmp(pass, DEFAULT_PASS) == 0)
    {
        err = "elige una clave distinta de la de fabrica";
        return false;
    }
    strncpy(params.pass, pass, sizeof(params.pass) - 1);
    params.pass[sizeof(params.pass) - 1] = '\0';
    saveParams();
    requestRestart();
    return true;
}

/**
 * @brief Levanta/apaga el AP segun "en tierra" (WEB_AP_GROUND_ONLY). Llamar periodicamente.
 */
void FWM::updateApGate()
{
#if USE_WEB_SERVER
    static bool lastGround = true;
    bool ground = canChangeApMode();
    if (ground != lastGround)
    {
        Log.notice("AP gate: %s" CR, ground ? "tierra -> AP ON" : "vuelo -> AP OFF");
        lastGround = ground;
    }
    const bool apRequested = params.ap_mode != FWM_AP_MODE_OFF;
    const bool apAllowed = WEB_AP_FORCE || !WEB_AP_GROUND_ONLY || ground;
    if (apRequested && apAllowed && !web->server_up)
    {
        web->startAP();
    }
    else if ((!apRequested || !apAllowed) && web->server_up)
    {
        web->stopAP();
    }
#endif
}

// ============================================================================================================
// Fase 2: tabla de parametros FWM (fuente de verdad). get/set + persistencia + JSON.
// ============================================================================================================
static const ParamDef_t s_paramTable[] = {
    {"formation",        "Formation",            PARAM_ENUM,  0.0f,  4.0f,   "",       1, PARAM_SCOPE_FOLLOWER},
    {"dist_offset",      "Trail distance",       PARAM_FLOAT, 20.0f, 500.0f, "m",      1, PARAM_SCOPE_FOLLOWER},
    {"lateral_offset",   "Lateral offset",       PARAM_FLOAT, 5.0f,  300.0f, "m",      1, PARAM_SCOPE_FOLLOWER},
    {"vertical_offset",  "Vertical offset",      PARAM_FLOAT, 0.0f,  200.0f, "m",      1, PARAM_SCOPE_FOLLOWER},
    {"cross_gain",       "Lateral gain",         PARAM_FLOAT, 0.05f, 2.0f,   "deg/m",  1, PARAM_SCOPE_FOLLOWER},
    {"hdg_corr_max",     "Max heading corr",     PARAM_FLOAT, 5.0f,  60.0f,  "deg",    1, PARAM_SCOPE_FOLLOWER},
    {"along_gain",       "Longitudinal gain",    PARAM_FLOAT, 0.0f,  60.0f,  "cm/s/m", 1, PARAM_SCOPE_FOLLOWER},
    {"prediction",       "Prediction",           PARAM_BOOL,  0.0f,  1.0f,   "",       1, PARAM_SCOPE_FOLLOWER},
    {"filter",           "Position filter",      PARAM_BOOL,  0.0f,  1.0f,   "",       1, PARAM_SCOPE_FOLLOWER},
    {"foll_enable",      "Follow enable",        PARAM_BOOL,  0.0f,  1.0f,   "",       1, PARAM_SCOPE_FOLLOWER},
    {"link_timeout",     "Link timeout",         PARAM_INT,   2.0f,  120.0f, "s",      1, PARAM_SCOPE_COMMON},
    {"netid",            "Network ID",           PARAM_INT,   0.0f,  65535.0f,"",       1, PARAM_SCOPE_COMMON},
    {"approach_dist",    "Approach distance",    PARAM_FLOAT, 50.0f, 5000.0f, "m",      0, PARAM_SCOPE_FOLLOWER},
    {"role",             "Device role",          PARAM_ENUM,  0.0f,  2.0f,   "",       1, PARAM_SCOPE_COMMON},
    {"ap_mode",          "AP mode",              PARAM_ENUM,  0.0f,  2.0f,   "",       1, PARAM_SCOPE_COMMON},
    // Al final a proposito: los indices previos son el ABI del parametro MAVLink y no deben shifting.
    {"band",             "LoRa band",            PARAM_ENUM,  0.0f,  2.0f,   "",       1, PARAM_SCOPE_COMMON},
};

int FWM::paramCount()
{
    return (int)(sizeof(s_paramTable) / sizeof(s_paramTable[0]));
}

const ParamDef_t *FWM::paramDefAt(int idx)
{
    if (idx < 0 || idx >= paramCount())
    {
        return nullptr;
    }
    return &s_paramTable[idx];
}

int FWM::findParam(const char *key)
{
    for (int i = 0; i < paramCount(); i++)
    {
        if (strcmp(s_paramTable[i].key, key) == 0)
        {
            return i;
        }
    }
    return -1;
}

float FWM::getParamByIndex(int idx)
{
    switch (idx)
    {
    case 0: return mav ? (float)(int)mav->currentFormation : 0.0f;
    case 1: return params.dist_offset;
    case 2: return params.lateral_offset;
    case 3: return params.vertical_offset;
    case 4: return params.cross_gain;
    case 5: return params.heading_corr_max;
    case 6: return params.along_gain;
    case 7: return (mav && mav->predictionEnabled) ? 1.0f : 0.0f;
    case 8: return (mav && mav->filterEnabled) ? 1.0f : 0.0f;
    case 9: return (float)params.foll_enable;
    case 10: return (float)params.link_timeout;
    case 11: return (float)params.netid;
    case 12: return params.approach_dist;
    case 13: return (float)params.role;
    case 14: return (float)params.ap_mode;
    case 15: return loraBandValid(params.band) ? (float)params.band : (float)FWM_DEFAULT_BAND;
    default: return 0.0f;
    }
}

bool FWM::setParamByIndex(int idx, float value, bool persist)
{
    if (idx < 0 || idx >= paramCount())
    {
        return false;
    }
    const ParamDef_t &d = s_paramTable[idx];
    if (value < d.min) value = d.min;
    if (value > d.max) value = d.max;
    if (idx == 13)
    {
        const int32_t requestedRole = (int32_t)(value + 0.5f);
        if (requestedRole == params.role)
            return true;
        if (!canChangeRole())
            return false;
        // Un lider necesita un radio que funcione; sin el, el fallo es silencioso.
        if (requestedRole == FWM_ROLE_LEADER && !canBecomeLeader())
            return false;
    }
    if (idx == 14)
    {
        const int32_t requestedMode = (int32_t)(value + 0.5f);
        if (requestedMode == params.ap_mode)
            return true;
        if (!canChangeApMode())
            return false;
    }
    bool restartForRole = false;
    switch (idx)
    {
    case 0: if (mav) mav->currentFormation = (FormationType)(int)value; break;
    case 1: params.dist_offset = value; break;
    case 2: params.lateral_offset = value; break;
    case 3: params.vertical_offset = value; break;
    case 4: params.cross_gain = value; break;
    case 5: params.heading_corr_max = value; break;
    case 6: params.along_gain = value; break;
    case 7: if (mav) mav->predictionEnabled = value >= 0.5f; break;
    case 8: if (mav) mav->filterEnabled = value >= 0.5f; break;
    case 9: params.foll_enable = (int32_t)value; break;
    case 10: params.link_timeout = (int32_t)value; break;
    case 11: params.netid = (uint16_t)value; if (comm) comm->applyNetid(); break;
    case 12: params.approach_dist = value; break;
    case 13:
    {
        const int32_t newRole = (int32_t)(value + 0.5f);
        if (newRole != params.role)
        {
            params.role = newRole;
            restartForRole = persist;
        }
        break;
    }
    case 14: params.ap_mode = (int32_t)(value + 0.5f); break;
    case 15:
    {
        const int32_t newBand = (int32_t)(value + 0.5f);
        if (!loraBandValid(newBand))
            return false;
        if (newBand == params.band)
            break;
        // Cambiar de banda reinicia el SX1276: prohibido en vuelo. canWriteConfig() es
        // fail-closed (F-05), asi que sin enlace FC tampoco se acepta.
        if (!canWriteConfig())
            return false;
        params.band = newBand;
        if (comm)
            comm->applyBand(newBand); // el core 1 lo aplica; el core 0 no toca el SPI
        break;
    }
    default: return false;
    }
    if (persist)
    {
        saveParams();
    }
    if (restartForRole)
    {
        requestRestart();
    }
    return true;
}

bool FWM::setParamByKey(const char *key, const char *valueStr, String &err)
{
    int idx = findParam(key);
    if (idx < 0)
    {
        err = String("param desconocido: ") + key;
        return false;
    }
    const float v = String(valueStr).toFloat();
    if (idx == 13)
    {
        const ParamDef_t &d = s_paramTable[idx];
        const float bounded = constrain(v, d.min, d.max);
        if ((int32_t)(bounded + 0.5f) == params.role)
            return true;
        if (!canChangeRole())
        {
            err = "rol editable solo con FC conectado, desarmado y en tierra";
            return false;
        }
    }
    if (idx == 14)
    {
        const ParamDef_t &d = s_paramTable[idx];
        const float bounded = constrain(v, d.min, d.max);
        if ((int32_t)(bounded + 0.5f) == params.ap_mode)
            return true;
        if (!canChangeApMode())
        {
            err = "AP solo editable con FC conectado, desarmado y en tierra";
            return false;
        }
    }
    if (s_paramTable[idx].groundOnly && !canWriteConfig())
    {
        err = "solo editable en tierra";
        return false;
    }
    if (!setParamByIndex(idx, v, true))
    {
        err = idx == 13 ? "rol editable solo con FC conectado, desarmado y en tierra" :
              idx == 14 ? "AP solo editable con FC conectado, desarmado y en tierra" :
                          "no se pudo aplicar el parametro";
        return false;
    }
    return true;
}

String FWM::paramsJson()
{
    String s = "[";
    for (int i = 0; i < paramCount(); i++)
    {
        const ParamDef_t &d = s_paramTable[i];
        if (i)
        {
            s += ",";
        }
        s += "{\"key\":\"" + String(d.key) + "\",";
        s += "\"label\":\"" + String(d.label) + "\",";
        s += "\"type\":" + String((int)d.type) + ",";
        s += "\"min\":" + String(d.min, 3) + ",";
        s += "\"max\":" + String(d.max, 3) + ",";
        s += "\"unit\":\"" + String(d.unit) + "\",";
        s += "\"groundOnly\":" + String(d.groundOnly ? "true" : "false") + ",";
        s += "\"scope\":" + String(d.scope) + ",";
        s += "\"value\":" + String(getParamByIndex(i), 3) + "}";
    }
    s += "]";
    return s;
}

// ============================================================================================================
// FASE 1: IMPLEMENTACIÓN DE MÁQUINA DE ESTADOS
// ============================================================================================================

/**
 * @brief Transición a un nuevo estado
 * 
 * @param newState SystemState - Nuevo estado
 * @return void
 */
void FWM::transitionState(SystemState newState)
{
    if (isValidStateTransition(currentState, newState))
    {
        Log.notice("State transition: %s -> %s" CR, getStateName(currentState), getStateName(newState));
        
        // FASE 2: Log de transición de estado
        if (logger)
        {
            char data[64];
            snprintf(data, sizeof(data), "from=%s,to=%s", getStateName(currentState), getStateName(newState));
            logger->info("STATE_TRANSITION", data);
        }
        
        previousState = currentState;
        currentState = newState;
        stateEntryTime = millis();
        onStateEntry(newState);
    }
    else
    {
        Log.warning("Invalid state transition: %s -> %s" CR, getStateName(currentState), getStateName(newState));
    }
}

/**
 * @brief Verifica si una transición de estado es válida
 * 
 * @param from SystemState - Estado origen
 * @param to SystemState - Estado destino
 * @return bool - true si la transición es válida
 */
bool FWM::isValidStateTransition(SystemState from, SystemState to)
{
    // Permitir siempre transición a EMERGENCY
    if (to == STATE_EMERGENCY)
        return true;
    
    // Transiciones válidas según el estado actual
    switch (from)
    {
    case STATE_INIT:
        return (to == STATE_SEARCHING || to == STATE_LANDING);
        
    case STATE_SEARCHING:
        return (to == STATE_CONNECTING || to == STATE_FOLLOWING || to == STATE_LANDING);
        
    case STATE_CONNECTING:
        return (to == STATE_FOLLOWING || to == STATE_SEARCHING || to == STATE_LANDING);
        
    case STATE_FOLLOWING:
        return (to == STATE_LOST_LINK || to == STATE_LANDING || to == STATE_SEARCHING);
        
    case STATE_LOST_LINK:
        return (to == STATE_FOLLOWING || to == STATE_SEARCHING || to == STATE_CONNECTING || to == STATE_LANDING);
        
    case STATE_EMERGENCY:
        return (to == STATE_SEARCHING || to == STATE_LANDING);
        
    case STATE_LANDING:
        return (to == STATE_SEARCHING); // Puede volver a buscar después de aterrizar
        
    default:
        return false;
    }
}

/**
 * @brief Ejecuta acciones al entrar en un estado
 * 
 * @param state SystemState - Estado al que se entra
 * @return void
 */
void FWM::onStateEntry(SystemState state)
{
    switch (state)
    {
    case STATE_INIT:
        Log.notice("Initializing system..." CR);
        break;
        
    case STATE_SEARCHING:
        Log.notice("Searching for leader beacon..." CR);
        // Al buscar es cuando un desajuste de banda se manifiesta (el otro extremo es
        // inaudible y no genera error). Publicamos la banda propia para poder compararla.
        if (mav != nullptr)
        {
            char bandLine[32];
            snprintf(bandLine, sizeof(bandLine), "Searching on %s MHz", loraBandLabel(params.band));
            mav->status_text(bandLine, MAV_SEVERITY_INFO);
        }
        stage_follow = STAGE_IDLE;
        break;
        
    case STATE_CONNECTING:
        Log.notice("Establishing connection..." CR);
        break;
        
    case STATE_FOLLOWING:
        Log.notice("Following leader" CR);
        mav->status_text("Following leader");
        stage_follow = STAGE_APPROACH;
        break;
        
    case STATE_LOST_LINK:
        Log.warning("Link lost! Searching for leader..." CR);
        mav->status_text("Link lost - searching");
        stage_follow = STAGE_IDLE;
        break;
        
    case STATE_EMERGENCY:
        Log.error("EMERGENCY MODE ACTIVATED!" CR);
        mav->status_text("EMERGENCY - RTL");
        // Aquí se podría activar modo RTL o LOITER
        stage_follow = STAGE_IDLE;
        break;
        
    case STATE_LANDING:
        Log.notice("Landing procedure initiated" CR);
        mav->status_text("Landing");
        stage_follow = STAGE_IDLE;
        break;
    }
}

/**
 * @brief Obtiene el nombre de un estado
 * 
 * @param state SystemState - Estado
 * @return const char* - Nombre del estado
 */
const char* FWM::getStateName(SystemState state)
{
    switch (state)
    {
    case STATE_INIT:       return "INIT";
    case STATE_SEARCHING:  return "SEARCHING";
    case STATE_CONNECTING: return "CONNECTING";
    case STATE_FOLLOWING:  return "FOLLOWING";
    case STATE_LOST_LINK:  return "LOST_LINK";
    case STATE_EMERGENCY:  return "EMERGENCY";
    case STATE_LANDING:    return "LANDING";
    default:               return "UNKNOWN";
    }
}

// ============================================================================================================
// FASE 2: CONTROL DE FLUJO ADAPTATIVO
// ============================================================================================================

/**
 * @brief Obtiene la distancia al seguidor más cercano
 * 
 * @return float - Distancia en metros (0 si no hay seguidor)
 */
float FWM::getDistanceToFollower()
{
    // Si somos follower, calcular distancia al líder
    if (follow_mode == FOLL_MODE_FOLLOWER && comm->commData.have_beacon)
    {
        return mav->calculateDistance(mav->APdata.lat, mav->APdata.lon,
                                     comm->commData.lastValidPacket.lat,
                                     comm->commData.lastValidPacket.lon);
    }
    
    // Si somos líder, no tenemos forma de saber la distancia al seguidor
    // Retornar una distancia media por defecto
    return DISTANCE_THRESHOLD_MEDIUM;
}

/**
 * @brief A4: Distancia al peer (líder o seguidor) en metros
 *
 * @return float - distancia en metros, o -1 si no hay beacon válido
 */
float FWM::getLinkDistance()
{
    if (!comm->commData.have_beacon)
    {
        return -1.0f;
    }

    return mav->calculateDistance(mav->APdata.lat, mav->APdata.lon,
                                  comm->commData.lastValidPacket.lat,
                                  comm->commData.lastValidPacket.lon);
}

/**
 * @brief Actualiza la tasa de transmisión basándose en la distancia
 * 
 * Ajusta dinámicamente la frecuencia de envío de paquetes:
 * - Cerca (< 100m): 0.5 Hz (2000ms)
 * - Medio (100-500m): 1 Hz (1000ms)
 * - Lejos (> 500m): 2 Hz (500ms)
 */
void FWM::updateTransmissionRate()
{
    float distance = getDistanceToFollower();
    uint32_t newInterval;
    
    if (distance < DISTANCE_THRESHOLD_CLOSE)
    {
        newInterval = PACKET_RATE_CLOSE; // 2000ms - 0.5 Hz cuando está cerca
        Log.verbose("Transmission rate: CLOSE (%dm) - %dms" CR, (int)distance, newInterval);
    }
    else if (distance < DISTANCE_THRESHOLD_MEDIUM)
    {
        newInterval = PACKET_RATE_MEDIUM; // 1000ms - 1 Hz distancia media
        Log.verbose("Transmission rate: MEDIUM (%dm) - %dms" CR, (int)distance, newInterval);
    }
    else
    {
        newInterval = PACKET_RATE_FAR; // 500ms - 2 Hz cuando está lejos
        Log.verbose("Transmission rate: FAR (%dm) - %dms" CR, (int)distance, newInterval);
    }
    
    // Solo actualizar si el intervalo cambia significativamente (> 100ms diferencia)
    if (abs((int32_t)(newInterval - currentTransmissionInterval)) > 100)
    {
        currentTransmissionInterval = newInterval;
        sendPacketIntervalMs = newInterval;
        send_packet_ticker.detach();
        send_packet_ticker.attach_ms(newInterval, send_packet_ticker_callback);
        
        Log.notice("Transmission rate updated to %d ms" CR, newInterval);
        
        if (logger)
        {
            char data[64];
            snprintf(data, sizeof(data), "interval=%dms,distance=%.1fm", newInterval, distance);
            logger->info("RATE_CHANGE", data);
        }
    }
}

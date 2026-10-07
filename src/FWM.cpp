#include "FWM.h"

#include <esp_idf_version.h>
#include <cstring>
#include <cstdlib>

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

    // Init instances

    screen = new Screen(this);
    screen->begin();

    comm = new Comm(this);
    comm->begin();

    mav = new Telem(this);
    mav->begin();

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
    // Primero iniciar AP (WiFi debe estar listo ANTES del servidor web)
    web->startAP();
    delay(500);  // Dar tiempo al AP para estabilizarse
    #endif
    
    #if USE_WEB_SERVER
    // Luego inicializar servidor web (requiere WiFi activo)
    web->begin();
    #endif

    // Default Follow mode
    if (!MAV_BRIDGE)
    {
        changeFollowMode(FOLL_MODE);
    }

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
            mav->status_text(s, MAV_SEVERITY_WARNING); // WARNING/4 aparece en HUD de Mission Planner
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
            mav->status_text(s, MAV_SEVERITY_WARNING); // WARNING/4 se muestra en el HUD de MP
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
    if (mav)
    {
        lastFollowerDistM = (int)mav->calculateDistance(mav->APdata.lat, mav->APdata.lon, p.lat, p.lon);
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
    if (!mav || !comm || mav->linkTimeout)
    {
        Log.trace("Packet send skipped: No FC connection (AP config mode)" CR);
        return;
    }

    LoraPacket_t packet = {};
    packet.version = PROTOCOL_VERSION;
    packet.type = LORA_MSG_BEACON;
    packet.netid = params.netid;
    packet.mode = (uint8_t)mav->APdata.custom_mode;
    packet.sysid = SYSID;
    packet.seq = (uint16_t)(comm->txSeq + 1);
#if FOLLOWER_REPLY
    bool requestReply = !sessionActive || (now - lastReplyRequestMs >= FOLLOWER_REPLY_MS);
    packet.flags = requestReply ? LORA_FLAG_REPLY_SLOT : LORA_FLAG_NONE;
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

#if ADAPTIVE_RATE
    updateTransmissionRate();
#endif
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
        send_packet_ticker.attach_ms(SEND_PACKET_INTERVAL, FWM::send_packet_ticker_callback);
    }
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
    preferences.putFloat("heading_corr_max", params.heading_corr_max);
    preferences.putFloat("along_gain", params.along_gain);
    preferences.putInt("netid", (int)params.netid);
    preferences.putFloat("approach_dist", params.approach_dist);
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
    params.foll_enable = preferences.getInt("foll_enable", 0);
    params.foll_ofs_type = preferences.getInt("foll_ofs_type", 0);
    params.foll_alt_type = preferences.getInt("foll_alt_type", 0);
    params.link_timeout = preferences.getInt("link_timeout", 0);
    // Fase 2: offsets y ganancias (defaults = #define de config.h)
    params.dist_offset = preferences.getFloat("dist_offset", DIST_OFFSET);
    params.lateral_offset = preferences.getFloat("lateral_offset", FORMATION_LATERAL_OFFSET);
    params.vertical_offset = preferences.getFloat("vertical_offset", FORMATION_VERTICAL_OFFSET);
    params.cross_gain = preferences.getFloat("cross_gain", CROSS_TRACK_GAIN_DEG_PER_M);
    params.heading_corr_max = preferences.getFloat("heading_corr_max", MAX_HEADING_CORR_DEG);
    params.along_gain = preferences.getFloat("along_gain", ALONG_GAIN_CMS_PER_M);
    params.netid = (uint16_t)preferences.getInt("netid", NETID_DEFAULT);
    params.approach_dist = preferences.getFloat("approach_dist", APPROACH_DIST_DEFAULT);

    String ssid = preferences.getString("ssid", "");
    String pass = preferences.getString("pass", "");

    strncpy(params.ssid, ssid.c_str(), sizeof(params.ssid) - 1);
    params.ssid[sizeof(params.ssid) - 1] = '\0';
    strncpy(params.pass, pass.c_str(), sizeof(params.pass) - 1);
    params.pass[sizeof(params.pass) - 1] = '\0';

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
 * @brief Levanta/apaga el AP segun "en tierra" (WEB_AP_GROUND_ONLY). Llamar periodicamente.
 */
void FWM::updateApGate()
{
#if USE_WEB_SERVER && WEB_AP_GROUND_ONLY
    static bool lastGround = true;
    bool ground = isOnGround();
    if (ground != lastGround)
    {
        Log.notice("AP gate: %s" CR, ground ? "tierra -> AP ON" : "vuelo -> AP OFF");
        lastGround = ground;
    }
    if (ground && !web->server_up)
    {
        web->startAP();
    }
    else if (!ground && web->server_up)
    {
        web->stopAP();
    }
#endif
}

// ============================================================================================================
// Fase 2: tabla de parametros FWM (fuente de verdad). get/set + persistencia + JSON.
// ============================================================================================================
static const ParamDef_t s_paramTable[] = {
    {"formation",        "Formation",            PARAM_ENUM,  0.0f,  4.0f,   "",       1},
    {"dist_offset",      "Trail distance",       PARAM_FLOAT, 20.0f, 500.0f, "m",      1},
    {"lateral_offset",   "Lateral offset",       PARAM_FLOAT, 5.0f,  300.0f, "m",      1},
    {"vertical_offset",  "Vertical offset",      PARAM_FLOAT, 0.0f,  200.0f, "m",      1},
    {"cross_gain",       "Lateral gain",         PARAM_FLOAT, 0.05f, 2.0f,   "deg/m",  1},
    {"hdg_corr_max",     "Max heading corr",     PARAM_FLOAT, 5.0f,  60.0f,  "deg",    1},
    {"along_gain",       "Longitudinal gain",    PARAM_FLOAT, 0.0f,  60.0f,  "cm/s/m", 1},
    {"prediction",       "Prediction",           PARAM_BOOL,  0.0f,  1.0f,   "",       1},
    {"filter",           "Position filter",      PARAM_BOOL,  0.0f,  1.0f,   "",       1},
    {"foll_enable",      "Follow enable",        PARAM_BOOL,  0.0f,  1.0f,   "",       1},
    {"link_timeout",     "Link timeout",         PARAM_INT,   2.0f,  120.0f, "s",      1},
    {"netid",            "Network ID",           PARAM_INT,   0.0f,  65535.0f,"",       1},
    {"approach_dist",    "Approach distance",    PARAM_FLOAT, 50.0f, 5000.0f, "m",      0},
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
    default: return false;
    }
    if (persist)
    {
        saveParams();
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
    if (s_paramTable[idx].groundOnly && !isOnGround())
    {
        err = "solo editable en tierra";
        return false;
    }
    float v = String(valueStr).toFloat();
    setParamByIndex(idx, v, true);
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
    static uint32_t currentInterval = SEND_PACKET_INTERVAL;
    if (abs((int32_t)(newInterval - currentInterval)) > 100)
    {
        currentInterval = newInterval;
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

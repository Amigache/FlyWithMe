#include "FWM.h"

#include <esp_idf_version.h>

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
    // FASE 1: Reset watchdog en cada ciclo
    esp_task_wdt_reset();
    
    // FASE 2: Log periódico de telemetría (cada 30 segundos)
    static uint32_t lastTelemetryLog = 0;
    if (logger && (millis() - lastTelemetryLog > 30000))
    {
        logger->logTelemetry(mav->APdata);
        lastTelemetryLog = millis();
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

    // Run instances
    comm->run();
    mav->run();
    screen->run();
    web->run();
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
void FWM::send_packet_ticker_callback()
{
    if (self)
    {
        // No enviar packets si no hay conexión con FC (modo AP configuración)
        if (self->mav->linkTimeout)
        {
            Log.trace("Packet send skipped: No FC connection (AP config mode)" CR);
            return;
        }
        
        LoraPacket_t packet;
        packet.sysid = SYSID;
        packet.lat = self->mav->APdata.lat;
        packet.lon = self->mav->APdata.lon;
        packet.alt = self->mav->APdata.alt;
        packet.relative_alt = self->mav->APdata.relative_alt;
        packet.ground_speed = self->mav->APdata.ground_speed;
        packet.hdg = self->mav->APdata.hdg;
        packet.checksum = 0; // Se calcula en el momento de enviar el paquete

        packet.checksum = self->comm->calChecksum(packet);
        
        // FASE 2: Usar sendPacketWithRetry con reintentos
        if (self->comm->sendPacketWithRetry(packet))
        {
            Log.notice("Send Packet: %d, %d, %d, %d, %d, %d, %d, %d" CR, 
                      packet.sysid, packet.lat, packet.lon, packet.alt, 
                      packet.relative_alt, packet.ground_speed, packet.hdg, packet.checksum);
            
            // FASE 2: Log del paquete enviado
            if (self->logger)
            {
                self->logger->logPacket(packet, 0, 0);
            }
        }
        else
        {
            Log.error("Failed to send packet after retries" CR);
        }
        
        // FASE 2: Actualizar tasa de transmisión adaptativa
        #if ADAPTIVE_RATE
        self->updateTransmissionRate();
        #endif
    }
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

    String ssid = preferences.getString("ssid", "");
    String pass = preferences.getString("pass", "");

    strncpy(params.ssid, ssid.c_str(), sizeof(params.ssid) - 1);
    params.ssid[sizeof(params.ssid) - 1] = '\0';
    strncpy(params.pass, pass.c_str(), sizeof(params.pass) - 1);
    params.pass[sizeof(params.pass) - 1] = '\0';

    preferences.end();

    Log.notice("Params loaded" CR);
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
        return (to == STATE_CONNECTING || to == STATE_LANDING);
        
    case STATE_CONNECTING:
        return (to == STATE_FOLLOWING || to == STATE_SEARCHING || to == STATE_LANDING);
        
    case STATE_FOLLOWING:
        return (to == STATE_LOST_LINK || to == STATE_LANDING || to == STATE_SEARCHING);
        
    case STATE_LOST_LINK:
        return (to == STATE_SEARCHING || to == STATE_CONNECTING || to == STATE_LANDING);
        
    case STATE_EMERGENCY:
        return (to == STATE_LANDING);
        
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
        Log.verbose("Transmission rate: CLOSE (%.1fm) - %dms" CR, distance, newInterval);
    }
    else if (distance < DISTANCE_THRESHOLD_MEDIUM)
    {
        newInterval = PACKET_RATE_MEDIUM; // 1000ms - 1 Hz distancia media
        Log.verbose("Transmission rate: MEDIUM (%.1fm) - %dms" CR, distance, newInterval);
    }
    else
    {
        newInterval = PACKET_RATE_FAR; // 500ms - 2 Hz cuando está lejos
        Log.verbose("Transmission rate: FAR (%.1fm) - %dms" CR, distance, newInterval);
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
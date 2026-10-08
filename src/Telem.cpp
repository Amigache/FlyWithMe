#include "Telem.h"
#include "status_text.h"

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

Telem *Telem::self = nullptr;
static portMUX_TYPE s_statusTextMux = portMUX_INITIALIZER_UNLOCKED;
static StatusTextDeduper s_statusTextDeduper;

Telem::Telem(FWM *fwm) : SerialPort(1)
{
    // A1: APdata debe partir de un estado conocido. Sin esto, si no llega MAVLink
    // (o antes del primer mensaje) los campos contienen basura y se transmiten
    // coordenadas inválidas que el seguidor descarta.
    memset(&APdata, 0, sizeof(APdata));
    APdata.armed = false;

    this->fwm = fwm;
}

/**
 * @brief Telemtry Begin
 * @return Nothing
 */
void Telem::begin()
{
    self = this;

#if FC_EMULATION
    // Inicializar los datos sintéticos ANTES de usarlos (si no, parten con basura y desbordan)
    simulatedData.init();
    Log.notice("FC_EMULATION: telemetria sintetica activada (lat=%d, lon=%d)" CR,
               (int)(simulatedData.lat * 1e7), (int)(simulatedData.lon * 1e7));
#endif

#if FC_LINK_USB
    // Pruebas con SITL: MAVLink por el USB (UART0); no se usa el UART1.
    fcPort = &Serial;
    Log.notice("FC link por USB (UART0) - modo SITL" CR);
#else
    Log.notice("Init MAVLink Serial" CR);
    fcPort = &SerialPort;
    fcPort->begin(SERIAL_BAUD_TELEM, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    delay(3000);
    Log.notice("MAVLink Serial Ready" CR);
#endif

    // TICKER -----------------------------------------------------------------------------------------------
    if (!MAV_BRIDGE)
    {
        heartbeat_ticker.attach_ms(HEARTBEAT_INTERVAL, Telem::heartbeat_ticker_callback);
    }
}

/**
 * @brief Telemtry Heartbeat Ticker Callback
 * @return Nothing
 */
void Telem::heartbeat_ticker_callback()
{
    if (self)
    {
        // Led action
        digitalWrite(LED_BUILTIN, (!self->led_status) ? LOW : HIGH);

        // Enviamos heartbeat
        self->heartbeat(self->fwm->fwmSystemId(), COMPID, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);

        // Comprobamos link
        self->check_link();

        // Latido led
        self->led_status = (!self->led_status) ? true : false;
    }
}

/**
 * @brief Telemtry Run
 * @return Nothing
 */
void Telem::run()
{
#if FC_EMULATION
    // Emulación de FC: sintetizar telemetría válida (no usar el UART1)
    updateFcEmulation();
    return;
#endif

    // Try during to stablish link
    if (!linkTimeout)
    {
        // ESCUCHAMOS
        while (fcPort->available() > 0)
        {
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, fcPort->read(), &msg, &status))
            {
                // Antes de asignar rol, aprender el SYSID del ArduPilot conectado para
                // permitir provisión en cualquier avión (incluido el futuro seguidor SYSID 2).
                if (fwm->params.role == FWM_ROLE_OFF && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
                    msg.compid == TARGET_COMPID)
                {
                    mavlink_heartbeat_t candidate;
                    mavlink_msg_heartbeat_decode(&msg, &candidate);
                    if (candidate.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA)
                        autopilotSystemId = msg.sysid;
                }

                // MSGS que vienen de la FC
                if (msg.sysid == fwm->targetSystemId() && msg.compid == TARGET_COMPID)
                {
                    switch (msg.msgid)
                    {

                    case MAVLINK_MSG_ID_HEARTBEAT:
                    { // #0: Heartbeat

                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                        // Capturamos datos para APdata
                        APdata.custom_mode = heartbeat.custom_mode;
                        APdata.type = heartbeat.type;
                        APdata.autopilot = heartbeat.autopilot;
                        APdata.base_mode = heartbeat.base_mode;
                        APdata.system_status = heartbeat.system_status;
                        APdata.mavlink_version = heartbeat.mavlink_version;

                        // Detectamos armado/desarmado desde la pix (no gusta, poco preciso)
                        if (APdata.base_mode > 200)
                            APdata.armed = 1;
                        else
                            APdata.armed = 0;

                        // Time to get
                        last_heartbeat = millis();

                        break;
                    }

                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    { // #33: Global Position Int

                        mavlink_global_position_int_t global_position_int;
                        mavlink_msg_global_position_int_decode(&msg, &global_position_int);

                        // Capturamos datos para APdata
                        APdata.lat = global_position_int.lat;
                        APdata.lon = global_position_int.lon;
                        APdata.alt = global_position_int.alt;
                        APdata.relative_alt = global_position_int.relative_alt;
                        APdata.vx = global_position_int.vx;
                        APdata.vy = global_position_int.vy;
                        APdata.vz = global_position_int.vz;
                        APdata.hdg = global_position_int.hdg;
                        positionValid = true;

                        break;
                    }

                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                    { // #24: GPS Raw Int

                        mavlink_gps_raw_int_t gps_raw_int;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);

                        // Capturamos datos para APdata
                        // GPS_RAW_INT.vel viene en cm/s: mantener cm/s (convencion de APdata/LoraPacket).
                        // Antes se dividia entre 100 (=> m/s) pero el resto del codigo lo trata como
                        // cm/s, lo que dejaba la prediccion de movimiento casi a cero.
                        APdata.ground_speed = gps_raw_int.vel;

                        break;
                    }

                    case MAVLINK_MSG_ID_PARAM_VALUE:
                    { // #22: Param Value

                        mavlink_param_value_t param_value;
                        mavlink_msg_param_value_decode(&msg, &param_value);

                        // mostramos parámetro recibido por serial
                        // Log.notice("Param: %s = %F" CR, param_value.param_id, param_value.param_value);

                        break;
                    }

                    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
                    { // #62: Nav Controller Output

                        mavlink_nav_controller_output_t nav_controller_output;
                        mavlink_msg_nav_controller_output_decode(&msg, &nav_controller_output);

                        APdata.wp_dist = nav_controller_output.wp_dist;

                        break;
                    }

                    case MAVLINK_MSG_ID_RC_CHANNELS:
                    {
                        mavlink_rc_channels_t rc_channels;
                        mavlink_msg_rc_channels_decode(&msg, &rc_channels);

                        int16_t channel_values[18];

                        channel_values[0] = rc_channels.chan1_raw;
                        channel_values[1] = rc_channels.chan2_raw;
                        channel_values[2] = rc_channels.chan3_raw;
                        channel_values[3] = rc_channels.chan4_raw;
                        channel_values[4] = rc_channels.chan5_raw;
                        channel_values[5] = rc_channels.chan6_raw;
                        channel_values[6] = rc_channels.chan7_raw;
                        channel_values[7] = rc_channels.chan8_raw;
                        channel_values[8] = rc_channels.chan9_raw;
                        channel_values[9] = rc_channels.chan10_raw;
                        channel_values[10] = rc_channels.chan11_raw;
                        channel_values[11] = rc_channels.chan12_raw;
                        channel_values[12] = rc_channels.chan13_raw;
                        channel_values[13] = rc_channels.chan14_raw;
                        channel_values[14] = rc_channels.chan15_raw;
                        channel_values[15] = rc_channels.chan16_raw;
                        channel_values[16] = rc_channels.chan17_raw;
                        channel_values[17] = rc_channels.chan18_raw;

                        if (FOLL_MODE_CH > 0 && FOLL_MODE_CH <= 18)
                        {
                            // Obtain the value of the channel SWITCH_CH
                            int16_t switch_value = channel_values[FOLL_MODE_CH - 1];
                            uint8_t mode_value = FOLL_MODE_OFF;

                            // Set follow status
                            if (switch_value < 1400)
                            {
                                mode_value = FOLL_MODE_OFF;
                            }
                            else if (switch_value > 1400 && switch_value < 1600)
                            {
                                mode_value = FOLL_MODE_FOLLOWER;
                            }
                            else if (switch_value > 1600)
                            {
                                mode_value = FOLL_MODE_LEADER;
                            }

                            if (fwm->follow_mode != mode_value)
                            {
                                fwm->changeFollowMode(mode_value);
                            }
                        }

                        break;
                    }
                    case MAVLINK_MSG_ID_MISSION_ACK:
                    {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);

                        if (ack.type != MAV_MISSION_ACCEPTED)
                        {
                            Log.error("Mission ACK Error, Type: %d" CR, ack.type);
                        }
                        break;
                    }
                    case MAVLINK_MSG_ID_COMMAND_ACK:
                    {
                        mavlink_command_ack_t ack;
                        mavlink_msg_command_ack_decode(&msg, &ack);

                        if (ack.result != MAV_RESULT_ACCEPTED)
                        {
                            Log.error("Command ACK Error %d, result: %d" CR, ack.command, ack.result);
                        }
                        break;
                    }
                    }
                }
#if MAVLINK_PARAM_SERVER
                // Mensajes dirigidos a nuestro componente (servidor de parametros FWM)
                else if (msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_LIST ||
                         msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ ||
                         msg.msgid == MAVLINK_MSG_ID_PARAM_SET)
                {
                    handle_param_message(msg);
                }
                else if (msg.msgid == MAVLINK_MSG_ID_WIFI_CONFIG_AP)
                {
                    handle_wifi_config_ap(msg);
                }
#endif
            }
        }
    }
}

// ============================================================================================================
// Fase 3: servidor de parametros por MAVLink (PARAM_VALUE / PARAM_REQUEST_* / PARAM_SET)
// ============================================================================================================
void Telem::send_param_value(uint16_t index)
{
    if (index >= (uint16_t)fwm->paramCount())
    {
        return;
    }
    const ParamDef_t *d = fwm->paramDefAt(index);
    if (d == nullptr)
    {
        return;
    }
    char id[16];
    memset(id, 0, sizeof(id));
    strncpy(id, d->key, sizeof(id) - 1);
    mavlink_message_t msg;
    mavlink_msg_param_value_pack(fwm->fwmSystemId(), COMPID, &msg, id, fwm->getParamByIndex(index),
                                 MAV_PARAM_TYPE_REAL32, (uint16_t)fwm->paramCount(), index);
    send_to_fc(msg);
}

void Telem::send_all_params()
{
    for (int i = 0; i < fwm->paramCount(); i++)
    {
        send_param_value((uint16_t)i);
    }
}

void Telem::handle_param_message(mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        mavlink_param_request_list_t r;
        mavlink_msg_param_request_list_decode(&msg, &r);
        if ((r.target_system == 0 || r.target_system == fwm->fwmSystemId()) &&
            (r.target_component == 0 || r.target_component == COMPID))
        {
            send_all_params();
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        mavlink_param_request_read_t r;
        mavlink_msg_param_request_read_decode(&msg, &r);
        if (!((r.target_system == 0 || r.target_system == fwm->fwmSystemId()) &&
              (r.target_component == 0 || r.target_component == COMPID)))
        {
            break;
        }
        int idx = -1;
        if (r.param_index >= 0)
        {
            idx = r.param_index;
        }
        else
        {
            char id[17];
            memcpy(id, r.param_id, 16);
            id[16] = 0;
            idx = fwm->findParam(id);
        }
        if (idx >= 0)
        {
            send_param_value((uint16_t)idx);
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET:
    {
        mavlink_param_set_t s;
        mavlink_msg_param_set_decode(&msg, &s);
        if (!((s.target_system == 0 || s.target_system == fwm->fwmSystemId()) &&
              (s.target_component == 0 || s.target_component == COMPID)))
        {
            break;
        }
        char id[17];
        memcpy(id, s.param_id, 16);
        id[16] = 0;
        int idx = fwm->findParam(id);
        if (idx >= 0)
        {
            if (fwm->canWriteConfig() && !fwm->apPassChangeRequired())
            {
                fwm->setParamByIndex(idx, s.param_value, true);
            }
            // Echo del valor (confirma; si no estamos en tierra devuelve el valor sin cambios)
            send_param_value((uint16_t)idx);
        }
        break;
    }
    default:
        break;
    }
}

/**
 * @brief Cambio de la clave WiFi desde el GCS (WIFI_CONFIG_AP). El SSID se genera desde la MAC
 * y no se admite cambiarlo; la confirmación llega como STATUSTEXT.
 */
void Telem::handle_wifi_config_ap(mavlink_message_t &msg)
{
    mavlink_wifi_config_ap_t config;
    mavlink_msg_wifi_config_ap_decode(&msg, &config);

    char ssid[sizeof(config.ssid) + 1];
    char pass[sizeof(config.password) + 1];
    memcpy(ssid, config.ssid, sizeof(config.ssid));
    ssid[sizeof(config.ssid)] = '\0';
    memcpy(pass, config.password, sizeof(config.password));
    pass[sizeof(config.password)] = '\0';

    if (ssid[0] != '\0')
    {
        status_text("WiFi: el SSID se genera desde la MAC y no se puede cambiar", MAV_SEVERITY_WARNING);
        return;
    }
    String error;
    if (!fwm->setApPassphrase(pass, error))
    {
        char text[96];
        snprintf(text, sizeof(text), "WiFi: %s", error.c_str());
        status_text(text, MAV_SEVERITY_WARNING);
        return;
    }
    status_text("WiFi: clave guardada; reiniciando");
}

/**
 * @brief Telemtry Bridge Run
 * @return Nada
 */
void Telem::bridgeRun()
{
    if (fcPort->available())
    {
        receive_mavlink_serial();
    }
}

/**
 * @brief Telemtry Receive MAVLink Serial
 * @return Nada
 */
void Telem::receive_mavlink_serial()
{
    static mavlink_message_t message;
    static mavlink_status_t status;

    // Read all data available from Serial and send over LoRa
    while (fcPort->available() > 0)
    {
        uint8_t serial_byte = fcPort->read();
        if (mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status))
        {
            if (message.sysid == fwm->targetSystemId() && message.compid == TARGET_COMPID)
            {
                switch (message.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    // fwm->comm->send_mavlink_lora(message);
                    break;
                case MAVLINK_MSG_ID_HIGH_LATENCY2:
                    fwm->comm->send_mavlink_lora(message);
                    break;
                default:
                    // fwm->comm->send_mavlink_lora(message);
                    break;
                }
            }
        }
    }
}

/**
 * @brief Telemtry Send to FC
 * @param mavlink_message_t msg
 * @return Nothing
 */
void Telem::send_to_fc(mavlink_message_t msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    fcPort->write(buf, len);
}

/**
 * @brief Sends a heartbeat message via the MAVLink protocol.
 *
 * This method constructs a heartbeat message using the MAVLink library and sends it through
 * the MAVLink serial communication object (_MAVSerial).
 *
 * @param system_id The ID of the system sending the heartbeat.
 * @param component_id The ID of the component sending the heartbeat.
 * @param type The type of the vehicle (quadrotor, fixed-wing, etc.).
 * @param autopilot The type of autopilot software running on the vehicle.
 * @param base_mode The base mode of the vehicle (e.g. armed, disarmed, etc.).
 * @param custom_mode A custom mode value that can be used for autopilot-specific flags.
 * @param system_status The status of the vehicle system (e.g. active, standby, etc.).
 */
void Telem::heartbeat(uint8_t system_id, uint8_t component_id, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, base_mode, custom_mode, system_status);
    send_to_fc(msg);
}

/**
 * @brief Sends a status message via the MAVLink protocol with the specified text.
 *
 * This method constructs a status text message using the MAVLink library and sends it through
 * the MAVLink serial communication object (_MAVSerial).
 *
 * @param text Pointer to the text that will be included in the status message.
 */
void Telem::status_text(const char *text, uint8_t severity)
{
    if (text == nullptr)
    {
        return;
    }

    // MAVLink STATUSTEXT trae 50 bytes. Comparar el mensaje ya prefijado y truncado exactamente
    // como se transmite: si no cambió texto ni severidad, no volver a inundar GCS/OSD nunca.
    char wireText[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = {};
    snprintf(wireText, sizeof(wireText), "FWM: %s", text);
    portENTER_CRITICAL(&s_statusTextMux);
    const bool shouldSend = s_statusTextDeduper.shouldSend(wireText, severity);
    portEXIT_CRITICAL(&s_statusTextMux);
    if (!shouldSend)
    {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(fwm->fwmSystemId(), COMPID, &msg, severity, wireText, 0, 0);
    Log.notice("STATUSTEXT sent sev=%d text=%s" CR, severity, wireText);
    send_to_fc(msg);
}

void Telem::enableRuntimeSitlUsb()
{
    // Serial/UART0 ya está inicializado a 57600 por setup(); el perfil runtime dev
    // reserva ese puerto para MAVLink y deja de usarlo como consola.
    fcPort = &Serial;
    link = false;
    linkTimeout = false;
    linkTryTime = 0;
    positionValid = false;
    is_connecting = true;
    // check_link() convierte last_heartbeat de ms a s; guardar aquí la misma unidad que
    // las actualizaciones MAVLink para no disparar falsamente el timeout al entrar en SIM.
    last_heartbeat = millis();
    autopilotSystemId = 0;
    // El timeout de búsqueda inicial detuvo el ticker. Reanudarlo para negociar heartbeats
    // con SITL cuando el bridge serie aparezca.
    heartbeat_ticker.detach();
    heartbeat_ticker.attach_ms(HEARTBEAT_INTERVAL, Telem::heartbeat_ticker_callback);
}

uint32_t Telem::minimumSafeAltitude() const
{
    return fwm != nullptr && fwm->isRuntimeSitlMode()
               ? FWM_SITL_MIN_SAFE_ALTITUDE
               : MIN_SAFE_ALTITUDE;
}

/**
 * @brief Sets the value of a parameter on the vehicle.
 *
 * This method constructs a parameter set message using the MAVLink library and sends it through
 * the MAVLink serial communication object (_MAVSerial).
 *
 * @param param_name The name of the parameter to set.
 * @param param_value The new value of the parameter.
 */
void Telem::set_param_value(std::string param_name, float param_value)
{
    mavlink_message_t msg;
    mavlink_msg_param_set_pack(fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID, param_name.c_str(), param_value, MAVLINK_TYPE_UINT8_T);
    send_to_fc(msg);
}

/**
 * @brief Requests the value of a parameter from the vehicle.
 *
 * This method constructs a parameter request read message using the MAVLink library and sends it through
 * the MAVLink serial communication object (_MAVSerial).
 *
 * @param param_name The name of the parameter to request.
 */
void Telem::request_param_value(std::string param_name)
{
    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID, param_name.c_str(), -1);
    send_to_fc(msg);
}

/**
 * @brief Requests a data stream from the vehicle.
 *
 * This method constructs a request data stream message using the MAVLink library and sends it through
 * the MAVLink serial communication object (_MAVSerial).
 *
 * @param req_stream_id The ID of the requested data stream.
 * @param req_message_rate The rate at which the data stream should be sent.
 * @param start_stop A flag indicating whether to start or stop the data stream.
 */
void Telem::request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID, req_stream_id, req_message_rate, start_stop);
    send_to_fc(msg);
}

/**
 * Sends a MAVLink command to change the vehicle's speed.
 *
 * @param speed Target speed in m/s, adjusted by SPEED_OFFSET if applicable.
 *
 * This method packs and sends a `MAV_CMD_DO_CHANGE_SPEED` message with the specified speed.
 * The message is sent to the autopilot system/component selected by the configured role.
 */
void Telem::do_change_speed(uint16_t speed)
{
    // Mensajeria: enviar DO_CHANGE_SPEED SOLO cuando la velocidad cambia (no repetir la misma).
    // ArduPilot responde un STATUSTEXT "Set groundspeed ..." por cada comando; si no cambia, no lo mandamos.
    static uint16_t lastSpeed = 0xFFFF;
    if (lastSpeed != 0xFFFF && abs((int)speed - (int)lastSpeed) < SPEED_CHANGE_THRESHOLD)
    {
        return;
    }
    lastSpeed = speed;

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        fwm->fwmSystemId(),      // Sender system ID
        COMPID,                  // Sender component ID
        &msg,                    // MAVLink message
        fwm->targetSystemId(),   // Target system ID
        TARGET_COMPID,           // Target component ID
        MAV_CMD_DO_CHANGE_SPEED, // Command ID
        0,                       // Confirmation
        1,                       // Param 1: Speed type (0=Airspeed, 1=Ground Speed).
        (float)speed / 100.0f,   // Param 2: Target speed (m/s). `speed` llega en cm/s.
        -1,                      // Param 3: Throttle as a percentage (0-100%). A value of 0 or negative indicates no change.
        0,                       // Param 4: Empty
        0,                       // Param 5: Empty
        0,                       // Param 6: Empty
        0                        // Param 7: Empty
    );

    send_to_fc(msg);
}

/**
 * Sends a MAVLink command to set a navigation waypoint.
 *
 * @param lat Latitude in degrees (multiplied by 1E7).
 * @param lon Longitude in degrees (multiplied by 1E7).
 * @param alt Altitude in meters, adjusted by ALT_OFFSET if applicable.
 *
 * This method packs and sends a `MAV_CMD_NAV_WAYPOINT` message with the specified coordinates and altitude.
 * The message is sent to the autopilot system/component selected by the configured role.
 */
void Telem::nav_waypoint(int32_t lat, int32_t lon, int32_t alt)
{
    // P2: la altitud objetivo ya incluye el offset de formacion (calculateFormationPosition);
    // no volver a sumar ALT_OFFSET aqui (antes se aplicaba dos veces => +20 m).

    // ArduPlane en GUIDED ignora MISSION_ITEM (NAV_WAYPOINT) para guiado; el comando correcto es
    // MAV_CMD_DO_REPOSITION (COMMAND_INT). Antes enviabamos mission_item_int y el avion no se movia.
    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        fwm->fwmSystemId(),                // Sender system ID
        COMPID,                            // Sender component ID
        &msg,                              // MAVLink message
        fwm->targetSystemId(),             // Target system ID
        TARGET_COMPID,                     // Target component ID
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
        MAV_CMD_DO_REPOSITION,             // Command
        0,                                 // current
        0,                                 // autocontinue
        -1,                                // param1: velocidad (-1 = por defecto)
        0,                                 // param2: flags
        0,                                 // param3: radio
        0,                                 // param4: yaw
        lat,                               // x: latitud (* 1E7)
        lon,                               // y: longitud (* 1E7)
        (alt / 1000.0f)                    // z: altitud (m, relativa)
    );

    send_to_fc(msg);
}

// ============================================================================================================
// GUIADO POR RUMBO (GUIDED_CHANGE_*) - evita el loiter de DO_REPOSITION y maneja los giros
// ============================================================================================================

void Telem::guided_change_heading(float heading_deg, float rate_dps)
{
    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_GUIDED_CHANGE_HEADING,
        0, 0,
        0.0f,       // param1: 0 = course over ground
        heading_deg,// param2: rumbo objetivo (deg 0-359.99)
        rate_dps,   // param3: velocidad de cambio (deg/s)
        0.0f,
        0, 0, 0.0f);
    send_to_fc(msg);
}

void Telem::guided_change_speed(float speed_mps, float accel_mps2)
{
    // ArduPlane SOLO acepta airspeed (param1=0); con groundspeed (1) responde DENIED.
    if (speed_mps < GUIDED_AIRSPEED_MIN)
    {
        speed_mps = GUIDED_AIRSPEED_MIN;
    }
    if (speed_mps > GUIDED_AIRSPEED_MAX)
    {
        speed_mps = GUIDED_AIRSPEED_MAX;
    }

    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_GUIDED_CHANGE_SPEED,
        0, 0,
        0.0f,        // param1: 0 = airspeed (unico soportado por ArduPlane)
        speed_mps,   // param2: velocidad (m/s)
        accel_mps2,  // param3: aceleracion (m/s^2)
        0.0f,
        0, 0, 0.0f);
    send_to_fc(msg);
}

void Telem::guided_change_altitude(float alt_m, float rate_mps)
{
    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        fwm->fwmSystemId(), COMPID, &msg, fwm->targetSystemId(), TARGET_COMPID,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_CMD_GUIDED_CHANGE_ALTITUDE,
        0, 0,
        0.0f,
        0.0f,
        rate_mps,    // param3: regimen de cambio (m/s)
        0.0f,
        0, 0, alt_m);// param7 (z): altitud objetivo (m)
    send_to_fc(msg);
}

/**
 * @brief Guiado de formacion por TRACK (cross-track), suave y sin zigzag:
 *  - Rumbo = direccion de la traza del lider + correccion proporcional al error lateral.
 *  - Velocidad = la del lider + correccion por error longitudinal (distancia al punto).
 *  - Cada GUIDED_ALT_REFRESH_MS un DO_REPOSITION al punto de formacion fija la altitud (next_WP_loc).
 *  El rumbo se reafirma cada ciclo y manda sobre la posicion.
 */
void Telem::guided_follow(LoraPacket_t leader, int32_t targetLat, int32_t targetLon, int32_t targetAlt)
{
    // 1) Direccion de la traza del lider (plano NE) desde su velocidad
    float un = 0.0f, ue = 0.0f;
    float vmag = sqrtf((float)leader.vx * (float)leader.vx + (float)leader.vy * (float)leader.vy);
    if (vmag > 100.0f) // > 1 m/s
    {
        un = (float)leader.vx / vmag;
        ue = (float)leader.vy / vmag;
    }
    else
    {
        // Respaldo: marcacion del seguidor al punto de formacion
        double lat1 = APdata.lat / 1E7, lon1 = APdata.lon / 1E7;
        double lat2 = targetLat / 1E7, lon2 = targetLon / 1E7;
        double dLon = (lon2 - lon1) * PI / 180.0;
        double yy = sin(dLon) * cos(lat2 * PI / 180.0);
        double xx = cos(lat1 * PI / 180.0) * sin(lat2 * PI / 180.0) -
                    sin(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) * cos(dLon);
        float brg = (float)atan2(yy, xx);
        un = cosf(brg);
        ue = sinf(brg);
    }

    // 2) Errores del seguidor respecto al punto de formacion, en ejes de la traza
    double fnlat = APdata.lat / 1E7, fnlon = APdata.lon / 1E7;
    double plat = targetLat / 1E7, plon = targetLon / 1E7;
    float dn = (float)((fnlat - plat) * 111320.0);
    float de = (float)((fnlon - plon) * 111320.0 * cos(plat * PI / 180.0));
    float along = dn * un + de * ue;   // positivo = por delante del punto
    float cross = -dn * ue + de * un;  // positivo = a la derecha de la traza

    // 3) Rumbo = traza + correccion cross-track
    float theta = atan2f(ue, un) * 180.0f / PI;
    float corr = -fwm->params.cross_gain * cross;
    if (corr > fwm->params.heading_corr_max)
        corr = fwm->params.heading_corr_max;
    if (corr < -fwm->params.heading_corr_max)
        corr = -fwm->params.heading_corr_max;
    float hcmd = theta + corr;
    if (hcmd < 0.0f)
        hcmd += 360.0f;
    if (hcmd >= 360.0f)
        hcmd -= 360.0f;

    // 4) Velocidad = la del lider + correccion longitudinal (por detras -> acelerar)
    float along_dead = along;
    if (fabsf(along_dead) < SPEED_DEADBAND_M)
        along_dead = 0.0f; // banda muerta: evita cazar la velocidad con ruido pequeno
    float boost = -along_dead * fwm->params.along_gain;
    if (boost > MAX_SPEED_BOOST)
        boost = MAX_SPEED_BOOST;
    if (boost < -MAX_SPEED_SLOW)
        boost = -MAX_SPEED_SLOW;
    float speed_cms = (float)leader.ground_speed + boost;
    // Cuantizar para que la velocidad comandada cambie a saltos y no cazar (el FC ignora valores repetidos)
    speed_cms = roundf(speed_cms / (float)SPEED_QUANT_CMS) * (float)SPEED_QUANT_CMS;

    // 5) Refresco periodico del WP: fija la altitud objetivo (next_WP_loc) y respaldo de posicion
    static uint32_t lastWpMs = 0;
    uint32_t now = millis();
    if (lastWpMs == 0 || (now - lastWpMs) >= GUIDED_ALT_REFRESH_MS)
    {
        nav_waypoint(targetLat, targetLon, targetAlt);
        lastWpMs = now;
    }

#if HEAD_ON_GUARD
    // Guarda frente a frente: si el lider viene de cara y esta cerca, romper perpendicular y frenar.
    {
        double fLat = APdata.lat / 1E7, fLon = APdata.lon / 1E7;
        double lLat = leader.lat / 1E7, lLon = leader.lon / 1E7;
        float dn2 = (float)((lLat - fLat) * 111320.0);
        float de2 = (float)((lLon - fLon) * 111320.0 * cosf(fLat * PI / 180.0));
        float rng = sqrtf(dn2 * dn2 + de2 * de2);
        float brg = atan2f(de2, dn2) * 180.0f / PI;           // marcacion seguidor -> lider
        float lhdg = leader.hdg / 100.0f;                     // rumbo del lider (deg)
        float diff = fmodf(fabsf(lhdg - brg), 360.0f);
        if (diff > 180.0f) diff = 360.0f - diff;
        const bool guardActive = rng < HEAD_ON_RANGE && diff > HEAD_ON_FACE_DEG;
#if FWM_ALLOW_RUNTIME_SITL
        static uint32_t lastHeadOnDiag = 0;
        if (fwm->isRuntimeSitlMode() && rng < 800.0f && millis() - lastHeadOnDiag > 1000)
        {
            auto sendNamedInt = [this](const char *name, int32_t value)
            {
                mavlink_message_t diagnostic;
                mavlink_msg_named_value_int_pack(fwm->fwmSystemId(), COMPID, &diagnostic,
                                                 millis(), name, value);
                send_to_fc(diagnostic);
            };
            sendNamedInt("FWM_RNG", (int32_t)rng);
            sendNamedInt("FWM_FACE", (int32_t)diff);
            sendNamedInt("FWM_GUARD", guardActive ? 1 : 0);
            lastHeadOnDiag = millis();
        }
#endif
        static uint32_t lastDbg = 0;
        if (rng < 800.0f && millis() - lastDbg > 1500)
        {
            Log.notice("HO dbg rng=%dm face=%d lhdg=%d brg=%d" CR,
                       (int)rng, (int)diff, (int)lhdg, (int)brg);
            lastDbg = millis();
        }
        if (guardActive)   // el lider mira hacia nosotros
        {
            static uint32_t lastGuardLog = 0;
            if (millis() - lastGuardLog > 1000)
            {
                Log.warning("HEAD_ON guard: rng=%dm face=%d" CR, (int)rng, (int)diff);
                if (fwm->isRuntimeSitlMode())
                    status_text("HEAD_ON guard active", MAV_SEVERITY_WARNING);
                lastGuardLog = millis();
            }
            float hb = brg + HEAD_ON_BREAK_DEG;          // perpendicular a la visual
            if (hb < 0.0f) hb += 360.0f;
            if (hb >= 360.0f) hb -= 360.0f;
            guided_change_heading(hb, GUIDED_TURN_RATE_DPS);
            guided_change_speed(HEAD_ON_SPEED, GUIDED_SPEED_ACCEL);
            return;
        }
    }
#endif

    guided_change_heading(hcmd, GUIDED_TURN_RATE_DPS);
    guided_change_speed(speed_cms / 100.0f, GUIDED_SPEED_ACCEL);
}

/**
 * Sends a MAVLink command to reposition the vehicle.
 *
 * @param lat Latitude in degrees (multiplied by 1E7).
 * @param lon Longitude in degrees (multiplied by 1E7).
 * @param alt Altitude in meters, adjusted by ALT_OFFSET if applicable.
 * @param hdg Heading in degrees (multiplied by 100).
 *
 * This method packs and sends a `MAV_CMD_DO_REPOSITION` message with the specified coordinates, altitude, and heading.
 * The message is sent to the autopilot system/component selected by the configured role.
 */
void Telem::do_reposition(int32_t lat, int32_t lon, float alt, uint16_t hdg)
{
    // P2: no aplicar ALT_OFFSET aqui (se aplicaba tambien en CalculateFormationPosition).
    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        fwm->fwmSystemId(),                // Sender system ID
        COMPID,                            // Sender component ID
        &msg,                              // MAVLink message
        fwm->targetSystemId(),             // Target system ID
        TARGET_COMPID,                     // Target component ID
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
        MAV_CMD_DO_REPOSITION,             // Command ID
        0,                                 // current
        0,                                 // autocontinue
        -1,                                // Speed
        0,                                 // Bitmask of options MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
        0,                                 // Radius
        0,                                 // Yaw
        lat,                               // Latitude
        lon,                               // Longitude
        (alt / 1000)                       // Altitude
    );

    send_to_fc(msg);
}

uint16_t Telem::calculate_dynamic_speed(float leader_speed, float distance)
{
    // B: control PROPORCIONAL suave (evita el ciclo limite del esquema por escalones).
    // Si vamos a la distancia de formacion, igualamos la velocidad del lider; si vamos por
    // detras, sumamos una sobre-velocidad proporcional al error (con tope).
    float error = distance - DIST_OFFSET; // m por detras del punto de formacion
    if (error < 0.0f)
    {
        error = 0.0f;
    }

    float boost = error * SPEED_GAIN_CMS_PER_M; // cm/s
    if (boost > MAX_SPEED_BOOST)
    {
        boost = MAX_SPEED_BOOST;
    }

    return static_cast<uint16_t>(leader_speed + boost);
}

// Functions
void Telem::init_setup()
{

    // // Seteamos parámetros
    // if (AUTO_SET_FOLL_PARAMS)
    // {
    //     set_param_value("FOLL_ENABLE", FOLL_ENABLE);
    //     set_param_value("FOLL_OFS_TYPE", FOLL_OFS_TYPE);
    //     set_param_value("FOLL_ALT_TYPE", FOLL_ALT_TYPE);
    // }

    // Setup done
    init_setup_done = true;

    // Mandamos init setup done
    status_text("Init Setup Done");
}

void Telem::check_link()
{
#if FC_EMULATION
    // Sin FC real: mantener el link "vivo" para que el líder siga transmitiendo
    link = true;
    linkTimeout = false;
    linkTryTime = 0;
    return;
#endif

    if (linkTryTime >= fwm->params.link_timeout && !lock_ap)
    {
        Log.error("NOT FC CONNECTION" CR);
        link = false;
        linkTimeout = true;
        positionValid = false; // no anunciar distancias con la última posición de un FC desconectado
        is_connecting = false;

        heartbeat_ticker.detach();
    }
    else
    {
        // Calculamos tiempo pasado desde el último heartbeat
        unsigned long now = millis() / 1000;
        unsigned long last_hb = last_heartbeat / 1000;
        int dif = now - last_hb;

        if (dif >= LOST_TIME)
        { // PERDEMOS LINK

            linkTryTime++;

            Log.warning("LINK TO FC LOST! (%d)" CR, linkTryTime);
            link = false;
            is_connecting = true;
        }
        else if (dif <= 1 && !link)
        { // RECUPERAMOS LINK
            Log.notice("LINK TO FC OK!" CR);
            link = true;

            linkTryTime = 0;

            // On first connect we lock AP mode
            lock_ap = true;
            is_connecting = false;

            // Mandamos status text
            status_text("Connected");

            // Data Streams a 0
            // request_data_streams(MAV_DATA_STREAM_ALL, 0, 1);
            request_data_streams(MAV_DATA_STREAM_ALL, MAV_DATA_STREAM_POSITION_RATE, 1);

            // Data stream de posición
            // request_data_streams(MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, 1);
            // request_data_streams(MAV_DATA_STREAM_RAW_CONTROLLER, MAV_DATA_STREAM_RAW_CONTROLLER_RATE, 1);

            // Setup Inicial
            if (!init_setup_done)
                init_setup();
        }
    }
}

// ============================================================================================================
// FASE 1: LÍMITES DE SEGURIDAD
// ============================================================================================================

/**
 * @brief Calcula la distancia entre dos puntos GPS usando fórmula de Haversine
 * 
 * @param lat1 int32_t - Latitud punto 1 (* 1E7)
 * @param lon1 int32_t - Longitud punto 1 (* 1E7)
 * @param lat2 int32_t - Latitud punto 2 (* 1E7)
 * @param lon2 int32_t - Longitud punto 2 (* 1E7)
 * @return float - Distancia en metros
 */
float Telem::calculateDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    // Convertir a grados
    double lat1_deg = lat1 / 1E7;
    double lon1_deg = lon1 / 1E7;
    double lat2_deg = lat2 / 1E7;
    double lon2_deg = lon2 / 1E7;
    
    // Convertir a radianes
    double lat1_rad = lat1_deg * PI / 180.0;
    double lat2_rad = lat2_deg * PI / 180.0;
    double delta_lat = (lat2_deg - lat1_deg) * PI / 180.0;
    double delta_lon = (lon2_deg - lon1_deg) * PI / 180.0;
    
    // Fórmula de Haversine
    double a = sin(delta_lat / 2.0) * sin(delta_lat / 2.0) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(delta_lon / 2.0) * sin(delta_lon / 2.0);
    
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    
    // Radio de la Tierra en metros
    const double EARTH_RADIUS = 6371000.0;
    
    double distance = EARTH_RADIUS * c;
    
    return (float)distance;
}

/**
 * @brief Verifica si es seguro seguir al líder
 * 
 * @param leaderData LoraPacket_t - Datos del líder
 * @return bool - true si es seguro seguir
 */
bool Telem::isSafeToFollow(LoraPacket_t leaderData)
{
    // 1. Calcular distancia al líder
    float distance = calculateDistance(APdata.lat, APdata.lon, 
                                      leaderData.lat, leaderData.lon);
    
    if (distance > MAX_FOLLOW_DISTANCE)
    {
        Log.warning("Leader too far: %d m (max: %d m) foll=(%d,%d) lead=(%d,%d)" CR,
                    (int)distance, MAX_FOLLOW_DISTANCE,
                    APdata.lat, APdata.lon, leaderData.lat, leaderData.lon);
        status_text("Leader too far - aborting");
        
        // FASE 1: Transición a estado de emergencia
        fwm->transitionState(STATE_EMERGENCY);
        return false;
    }
    
    // 2. Verificar altitud mínima del líder
    const uint32_t minSafeAltitude = minimumSafeAltitude();
    if (leaderData.relative_alt < minSafeAltitude)
    {
        Log.warning("Leader altitude too low: %d mm (min: %d mm)" CR, 
                   leaderData.relative_alt, (int)minSafeAltitude);
        status_text("Leader altitude too low");
        return false;
    }
    
    // 3. Verificar velocidad del líder
    if (leaderData.ground_speed > MAX_FOLLOW_SPEED)
    {
        Log.warning("Leader speed too high: %d cm/s (max: %d cm/s)" CR, 
                   leaderData.ground_speed, MAX_FOLLOW_SPEED);
        status_text("Leader speed too high");
        return false;
    }
    
    // 4. Verificar nuestra propia altitud
    if (APdata.relative_alt < minSafeAltitude)
    {
        Log.warning("Own altitude too low: %d mm (min: %d mm)" CR, 
                   APdata.relative_alt, (int)minSafeAltitude);
        status_text("Altitude too low");
        return false;
    }
    
    // Todas las verificaciones pasaron
    return true;
}

// ============================================================================================================
// FASE 3: PREDICCIÓN DE MOVIMIENTO
// ============================================================================================================

/**
 * @brief Predice la posición futura del líder basándose en velocidad y rumbo actual
 * 
 * @param current LoraPacket_t - Datos actuales del líder
 * @param futureTime uint32_t - Tiempo futuro en ms (millis())
 * @return PredictedPosition - Posición predicha
 */
PredictedPosition Telem::predictLeaderPosition(LoraPacket_t current)
{
    PredictedPosition predicted;
    predicted.timestamp = millis();

    // NO restar current.timestamp de millis(): son relojes de dos ESP32 distintos y no están
    // sincronizados. La diferencia podía ser arbitraria (predicción desactivada o exagerada).
    float dt = PREDICTION_TIME_MS / 1000.0f;
    if (dt < 0.0f || dt > 5.0f)
    {
        predicted.lat = current.lat;
        predicted.lon = current.lon;
        predicted.alt = current.relative_alt;
        predicted.confidence = 0.0f;
        return predicted;
    }

    // Velocidad NED (vx/vy/vz en cm/s). Si no viene, usar ground_speed + hdg.
    float vn, ve, vd;
    if (current.vx != 0 || current.vy != 0)
    {
        vn = current.vx / 100.0f; // norte (m/s)
        ve = current.vy / 100.0f; // este (m/s)
        vd = current.vz / 100.0f; // abajo (m/s)
    }
    else
    {
        float speed = current.ground_speed / 100.0f;
        float headingRad = (current.hdg / 100.0f) * PI / 180.0f;
        vn = speed * cos(headingRad);
        ve = speed * sin(headingRad);
        vd = 0.0f;
    }

    // Seguridad geométrica para TRAIL cercano: no dejar que la predicción avance el punto objetivo
    // hasta el líder (p. ej. 1 s * 20 m/s cancela un offset de 20 m y lleva al seguidor a 0 m).
    // El adelanto queda limitado a una fracción configurable del offset deseado.
    float horizontalSpeed = sqrtf(vn * vn + ve * ve);
    if (currentFormation == FORMATION_TRAIL && horizontalSpeed > 0.1f)
    {
        float maxLead = fwm->params.dist_offset * PREDICTION_MAX_LEAD_FRACTION;
        float predictedLead = horizontalSpeed * dt;
        if (predictedLead > maxLead)
        {
            dt = maxLead / horizontalSpeed;
        }
    }

    double lat_degrees = current.lat / 1E7;
    float deltaLat_deg = (vn * dt) / 111320.0;
    float deltaLon_deg = (ve * dt) / (111320.0 * cos(lat_degrees * PI / 180.0));

    predicted.lat = current.lat + (int32_t)(deltaLat_deg * 1E7);
    predicted.lon = current.lon + (int32_t)(deltaLon_deg * 1E7);
    // vz positivo = hacia abajo; la altitud relativa (mm) sube cuando vz es negativo
    predicted.alt = current.relative_alt - (int32_t)(vd * dt * 1000.0f);

    // Confianza: alta si hay velocidad NED y el horizonte de predicción es corto
    float speed = sqrtf(vn * vn + ve * ve);
    if (speed < 1.0f)
        predicted.confidence = 0.3f;
    else if (dt < 1.5f)
        predicted.confidence = 0.9f;
    else if (dt < 3.0f)
        predicted.confidence = 0.7f;
    else
        predicted.confidence = 0.5f;
    Log.verbose("Predicted position: lat=%d, lon=%d, dt=%dms confidence=%d" CR,
                predicted.lat, predicted.lon, (int)(dt * 1000), (int)(predicted.confidence * 100));

    return predicted;
}

// ============================================================================================================
// FASE 3: FORMACIONES DINÁMICAS
// ============================================================================================================

/**
 * @brief Calcula la posición objetivo basándose en el tipo de formación
 * 
 * @param leader LoraPacket_t - Datos del líder
 * @param formation FormationType - Tipo de formación
 * @param targetLat int32_t& - Latitud objetivo (salida)
 * @param targetLon int32_t& - Longitud objetivo (salida)
 * @param targetAlt int32_t& - Altitud objetivo (salida)
 */
void Telem::calculateFormationPosition(LoraPacket_t leader, FormationType formation,
                                       int32_t &targetLat, int32_t &targetLon, int32_t &targetAlt)
{
    // Distancias de offset configurables (runtime, Fase 2)
    float offsetDistance = fwm->params.dist_offset;     // metros
    float lateralOffset = fwm->params.lateral_offset;   // metros
    float verticalOffset = fwm->params.vertical_offset; // metros
    
    // Convertir heading a radianes
    float headingRad = (leader.hdg / 100.0) * PI / 180.0;
    
    // Conversión aproximada metros a grados
    double lat_degrees = leader.lat / 1E7;
    const float LAT_M_TO_DEG = 1.0 / 111320.0;
    const float LON_M_TO_DEG = 1.0 / (111320.0 * cos(lat_degrees * PI / 180.0));
    
    float deltaLat_m = 0, deltaLon_m = 0;
    int32_t deltaAlt = 0;
    
    switch (formation)
    {
    case FORMATION_TRAIL:
        // Posición detrás del líder
        deltaLat_m = -offsetDistance * cos(headingRad);
        deltaLon_m = -offsetDistance * sin(headingRad);
        deltaAlt = 0; // TRAIL va a la MISMA altitud que el lider
        Log.verbose("Formation: TRAIL, offset=%dm" CR, (int)offsetDistance);
        break;
        
    case FORMATION_LEFT:
        // Posición a la izquierda del líder (perpendicular al rumbo, -90°)
        deltaLat_m = lateralOffset * cos(headingRad - PI/2);
        deltaLon_m = lateralOffset * sin(headingRad - PI/2);
        deltaAlt = 0; // LEFT va a la MISMA altitud que el lider
        Log.verbose("Formation: LEFT, offset=%dm" CR, (int)lateralOffset);
        break;
        
    case FORMATION_RIGHT:
        // Posición a la derecha del líder (perpendicular al rumbo, +90°)
        deltaLat_m = lateralOffset * cos(headingRad + PI/2);
        deltaLon_m = lateralOffset * sin(headingRad + PI/2);
        deltaAlt = 0; // RIGHT va a la MISMA altitud que el lider
        Log.verbose("Formation: RIGHT, offset=%dm" CR, (int)lateralOffset);
        break;
        
    case FORMATION_ABOVE:
        // Posición arriba del líder (misma posición horizontal)
        deltaLat_m = 0;
        deltaLon_m = 0;
        deltaAlt = (verticalOffset * 1000); // Offset positivo hacia arriba
        Log.verbose("Formation: ABOVE, offset=%dm" CR, (int)verticalOffset);
        break;
        
    case FORMATION_BELOW:
        // Posición abajo del líder (misma posición horizontal)
        deltaLat_m = 0;
        deltaLon_m = 0;
        deltaAlt = -(verticalOffset * 1000); // Offset negativo hacia abajo
        Log.verbose("Formation: BELOW, offset=%dm" CR, (int)verticalOffset);
        break;
    }
    
    // Aplicar offsets
    targetLat = leader.lat + (int32_t)(deltaLat_m * LAT_M_TO_DEG * 1E7);
    targetLon = leader.lon + (int32_t)(deltaLon_m * LON_M_TO_DEG * 1E7);
    targetAlt = leader.relative_alt + deltaAlt;

    // A: carrot/look-ahead en DISTANCIA fija sobre la traza del lider (direccion de su velocidad).
    // El objetivo queda ~FORMATION_LEAD_M por delante -> el avion vuela hacia un punto que nunca
    // alcanza y NO loitea; la distancia longitudinal la ajusta el lazo de velocidad.
    {
        float vmag = sqrtf((float)leader.vx * (float)leader.vx + (float)leader.vy * (float)leader.vy); // cm/s
        if (vmag > 100.0f && FORMATION_LEAD_M > 0.0f)
        {
            float un = leader.vx / vmag; // componente norte unitaria
            float ue = leader.vy / vmag; // componente este unitaria
            targetLat += (int32_t)(FORMATION_LEAD_M * un * LAT_M_TO_DEG * 1E7);
            targetLon += (int32_t)(FORMATION_LEAD_M * ue * LON_M_TO_DEG * 1E7);
        }
    }

    // Asegurar que la altitud no sea negativa
    if (targetAlt < 0) targetAlt = leader.relative_alt;
    
    Log.verbose("Formation position: lat=%d, lon=%d, alt=%d" CR, 
               targetLat, targetLon, targetAlt);
}

/**
 * @brief Obtiene el nombre de un tipo de formación
 * 
 * @param formation FormationType - Tipo de formación
 * @return const char* - Nombre de la formación
 */
const char* Telem::getFormationName(FormationType formation)
{
    switch (formation)
    {
    case FORMATION_TRAIL: return "TRAIL";
    case FORMATION_LEFT:  return "LEFT";
    case FORMATION_RIGHT: return "RIGHT";
    case FORMATION_ABOVE: return "ABOVE";
    case FORMATION_BELOW: return "BELOW";
    default:              return "UNKNOWN";
    }
}

// ============================================================================
// FASE 4: MODO SIMULACIÓN
// ============================================================================

#if SIMULATION_MODE

void Telem::initSimulation()
{
  simulatedData.init();
  Log.notice("Modo simulación inicializado" CR);
  Log.notice("Posicion inicial: lat=%d, lon=%d, alt=%dm" CR,
             (int)(simulatedData.lat * 1e7), (int)(simulatedData.lon * 1e7), (int)simulatedData.alt);
}

void Telem::updateSimulation()
{
  simulatedData.update();
}

LoraPacket_t Telem::getSimulatedPacket()
{
  LoraPacket_t packet;
  
  // Convertir de float a formato MAVLink (int32 * 1e7 para lat/lon)
  packet.lat = (int32_t)(simulatedData.lat * 1e7);
  packet.lon = (int32_t)(simulatedData.lon * 1e7);
  packet.alt = (int32_t)(simulatedData.alt * 100.0f);  // cm
  packet.relative_alt = (int32_t)(simulatedData.alt * 100.0f);
  
  packet.heading = (uint16_t)simulatedData.heading;
  packet.ground_speed = (uint16_t)simulatedData.groundSpeed;
  packet.climb = (int16_t)(simulatedData.climb * 100.0f);
  
  packet.sysid = 1;
  packet.custom_mode = 10;  // Auto mode
  packet.base_mode = 81;
  
  return packet;
}

#endif // SIMULATION_MODE

// ============================================================================
// EMULACIÓN DE FC (sin autopiloto): sintetiza telemetría válida en APdata
// ============================================================================
#if FC_EMULATION
void Telem::updateFcEmulation()
{
    simulatedData.update();

    APdata.lat = (int32_t)(simulatedData.lat * 1e7);
    APdata.lon = (int32_t)(simulatedData.lon * 1e7);
    APdata.alt = (int32_t)(simulatedData.alt * 1000.0f);       // mm
    APdata.relative_alt = APdata.alt;
    APdata.ground_speed = (uint16_t)simulatedData.groundSpeed; // cm/s
    APdata.hdg = (uint16_t)(simulatedData.heading * 100.0f);   // grados * 100
    APdata.vx = 0;
    APdata.vy = 0;
    APdata.vz = 0;
    APdata.wp_dist = 0;
    APdata.custom_mode = MODE_GUIDED; // activa el seguimiento en el FSM del seguidor
    APdata.base_mode = MAV_MODE_AUTO_ARMED;
    APdata.system_status = MAV_STATE_ACTIVE;
    APdata.armed = true;
    positionValid = true;
}
#endif // FC_EMULATION

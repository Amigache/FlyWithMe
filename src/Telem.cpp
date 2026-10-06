#include "Telem.h"

#include <cstring>

Telem *Telem::self = nullptr;

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

    Log.notice("Init MAVLink Serial" CR);
    SerialPort.begin(SERIAL_BAUD_TELEM, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    delay(3000);
    Log.notice("MAVLink Serial Ready" CR);

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
        self->heartbeat(SYSID, COMPID, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);

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
        while (SerialPort.available() > 0)
        {
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, SerialPort.read(), &msg, &status))
            {
                // MSGS que vienen de la FC
                if (msg.sysid == TARGET_SYSID && msg.compid == TARGET_COMPID)
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

                        break;
                    }

                    case MAVLINK_MSG_ID_GPS_RAW_INT:
                    { // #24: GPS Raw Int

                        mavlink_gps_raw_int_t gps_raw_int;
                        mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);

                        // Capturamos datos para APdata
                        APdata.ground_speed = (gps_raw_int.vel / 100);

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
            }
        }
    }
}

/**
 * @brief Telemtry Bridge Run
 * @return Nada
 */
void Telem::bridgeRun()
{
    if (SerialPort.available())
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
    while (SerialPort.available() > 0)
    {
        uint8_t serial_byte = SerialPort.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, serial_byte, &message, &status))
        {
            if (message.sysid == TARGET_SYSID && message.compid == TARGET_COMPID)
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
    SerialPort.write(buf, len);
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
void Telem::status_text(const char *text)
{
    mavlink_message_t msg;
    std::string msgText = std::string("FWM: ") + text;
    mavlink_msg_statustext_pack(SYSID, COMPID, &msg, 6, msgText.c_str(), 0, 0);
    send_to_fc(msg);
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
    mavlink_msg_param_set_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, param_name.c_str(), param_value, MAVLINK_TYPE_UINT8_T);
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
    mavlink_msg_param_request_read_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, param_name.c_str(), -1);
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
    mavlink_msg_request_data_stream_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, req_stream_id, req_message_rate, start_stop);
    send_to_fc(msg);
}

/**
 * Sends a MAVLink command to change the vehicle's speed.
 *
 * @param speed Target speed in m/s, adjusted by SPEED_OFFSET if applicable.
 *
 * This method packs and sends a `MAV_CMD_DO_CHANGE_SPEED` message with the specified speed.
 * The message is sent to the target system and component defined by `TARGET_SYSID` and `TARGET_COMPID`.
 */
void Telem::do_change_speed(uint16_t speed)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        SYSID,                   // Sender system ID
        COMPID,                  // Sender component ID
        &msg,                    // MAVLink message
        TARGET_SYSID,            // Target system ID
        TARGET_COMPID,           // Target component ID
        MAV_CMD_DO_CHANGE_SPEED, // Command ID
        0,                       // Confirmation
        1,                       // Param 1: Speed type (0=Airspeed, 1=Ground Speed).
        speed,                   // Param 2: Target speed (m/s). If airspeed, a value below or above min/max airspeed limits results in no change. a value of -2 uses :ref:`TRIM_ARSPD_CM`
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
 * The message is sent to the target system and component defined by `TARGET_SYSID` and `TARGET_COMPID`.
 */
void Telem::nav_waypoint(int32_t lat, int32_t lon, int32_t alt)
{

    // Altitude offset
    if (ALT_OFFSET != 0)
    {
        alt = alt + (ALT_OFFSET * 1000);
    }

    mavlink_message_t msg;
    mavlink_msg_mission_item_int_pack(
        SYSID,                             // ID del sistema (tu drone o vehículo)
        COMPID,                            // ID del componente (componente del MAVLink)
        &msg,                              // Puntero al mensaje
        TARGET_SYSID,                      // ID del sistema al que envías el comando
        TARGET_COMPID,                     // ID del componente (por ejemplo, el autopiloto)
        1,                                 // Sequence
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, // Frame
        MAV_CMD_NAV_WAYPOINT,              // Command
        2,                                 // current
        1,                                 // autocontinue
        0,                                 // Param 1 Delay
        0,                                 // Param 2 Empty
        0,                                 // Param 3 Empty
        0,                                 // Param 4 Empty
        lat,                               // Latitude
        lon,                               // Longitude
        (alt / 1000),                      // Altitude
        MAV_MISSION_TYPE_MISSION           // Mission type
    );

    send_to_fc(msg);
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
 * The message is sent to the target system and component defined by `TARGET_SYSID` and `TARGET_COMPID`.
 */
void Telem::do_reposition(int32_t lat, int32_t lon, float alt, uint16_t hdg)
{

    // Altitude offset
    if (ALT_OFFSET != 0)
    {
        alt = alt + (ALT_OFFSET * 1000);
    }

    mavlink_message_t msg;
    mavlink_msg_command_int_pack(
        SYSID,                             // Sender system ID
        COMPID,                            // Sender component ID
        &msg,                              // MAVLink message
        TARGET_SYSID,                      // Target system ID
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
    // If we are within the offset, return the leader's speed
    if (distance <= DIST_OFFSET)
    {
        return leader_speed;
    }

    // Calculate the speed multiplier based on the offset
    float speed_multiplier = 1 + (SPEED_OFFSET / 100.0);

    // Calculate the dynamic speed
    float dynamic_speed = leader_speed * speed_multiplier;

    // If we are within the double offset, interpolate the speed
    if (distance <= DIST_OFFSET * 2)
    {
        float factor = (distance - DIST_OFFSET) / DIST_OFFSET;
        dynamic_speed = leader_speed + (dynamic_speed - leader_speed) * factor;
    }

    return static_cast<uint16_t>(round(dynamic_speed));
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
    if (leaderData.relative_alt < MIN_SAFE_ALTITUDE)
    {
        Log.warning("Leader altitude too low: %d mm (min: %d mm)" CR, 
                   leaderData.relative_alt, MIN_SAFE_ALTITUDE);
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
    if (APdata.relative_alt < MIN_SAFE_ALTITUDE)
    {
        Log.warning("Own altitude too low: %d mm (min: %d mm)" CR, 
                   APdata.relative_alt, MIN_SAFE_ALTITUDE);
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
PredictedPosition Telem::predictLeaderPosition(LoraPacket_t current, uint32_t futureTime)
{
    PredictedPosition predicted;
    predicted.timestamp = futureTime;
    
    // Calcular tiempo de predicción en segundos
    float deltaTime = (futureTime - millis()) / 1000.0;
    
    // Si el tiempo es negativo o muy grande, no predecir
    if (deltaTime < 0 || deltaTime > 5.0)
    {
        predicted.lat = current.lat;
        predicted.lon = current.lon;
        predicted.alt = current.relative_alt;
        predicted.confidence = 0.0;
        return predicted;
    }
    
    // Calcular distancia recorrida en el tiempo de predicción
    // ground_speed está en cm/s
    float distanceTraveled = (current.ground_speed / 100.0) * deltaTime; // metros
    
    // Convertir heading a radianes (hdg está en grados * 100)
    float headingRad = (current.hdg / 100.0) * PI / 180.0;
    
    // Calcular cambios en latitud y longitud
    // Aproximación: 1 grado lat ≈ 111320 metros
    // 1 grado lon ≈ 111320 * cos(lat) metros
    double lat_degrees = current.lat / 1E7;
    double lon_degrees = current.lon / 1E7;
    
    // Componentes norte y este del movimiento
    float deltaLat_m = distanceTraveled * cos(headingRad);  // metros norte
    float deltaLon_m = distanceTraveled * sin(headingRad);  // metros este
    
    // Convertir a grados
    float deltaLat_deg = deltaLat_m / 111320.0;
    float deltaLon_deg = deltaLon_m / (111320.0 * cos(lat_degrees * PI / 180.0));
    
    // Calcular nueva posición
    predicted.lat = current.lat + (int32_t)(deltaLat_deg * 1E7);
    predicted.lon = current.lon + (int32_t)(deltaLon_deg * 1E7);
    predicted.alt = current.relative_alt; // Mantener altitud (o podría predecir con vz)
    
    // Calcular confianza basada en velocidad y tiempo
    // Mayor velocidad y menor tiempo = mayor confianza
    if (current.ground_speed < 100) // < 1 m/s
        predicted.confidence = 0.3;
    else if (current.ground_speed < 500) // < 5 m/s
        predicted.confidence = 0.6;
    else if (deltaTime < 2.0)
        predicted.confidence = 0.9;
    else
        predicted.confidence = 0.7;
    
    Log.verbose("Predicted position: lat=%d, lon=%d, confidence=%d" CR, 
               predicted.lat, predicted.lon, (int)(predicted.confidence * 100));
    
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
    // Distancias de offset configurables
    float offsetDistance = DIST_OFFSET; // metros (desde config.h)
    float lateralOffset = FORMATION_LATERAL_OFFSET; // metros
    float verticalOffset = FORMATION_VERTICAL_OFFSET; // metros
    
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
        deltaAlt = (ALT_OFFSET * 1000); // ALT_OFFSET está en metros, convertir a mm
        Log.verbose("Formation: TRAIL, offset=%dm" CR, (int)offsetDistance);
        break;
        
    case FORMATION_LEFT:
        // Posición a la izquierda del líder (perpendicular al rumbo, -90°)
        deltaLat_m = lateralOffset * cos(headingRad - PI/2);
        deltaLon_m = lateralOffset * sin(headingRad - PI/2);
        deltaAlt = (ALT_OFFSET * 1000);
        Log.verbose("Formation: LEFT, offset=%dm" CR, (int)lateralOffset);
        break;
        
    case FORMATION_RIGHT:
        // Posición a la derecha del líder (perpendicular al rumbo, +90°)
        deltaLat_m = lateralOffset * cos(headingRad + PI/2);
        deltaLon_m = lateralOffset * sin(headingRad + PI/2);
        deltaAlt = (ALT_OFFSET * 1000);
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
}
#endif // FC_EMULATION

#include "Telem.h"

Telem *Telem::self = nullptr;

Telem::Telem(FWM *fwm) : SerialPort(1)
{
    APdata.armed = false;

    this->fwm = fwm;
}

/**
 * @brief Telemtry Begin
 * @return Nada
 */
void Telem::begin()
{
    self = this;

    Log.notice("Init MAVLink Serial" CR);
    SerialPort.begin(SERIAL_BAUD_TELEM, SERIAL_8N1, SERIAL1_RX, SERIAL1_TX);
    delay(3000);
    Log.notice("MAVLink Serial Ready" CR);

    // Inicializamos los ticker
    heartbeat_ticker.attach_ms(HEARTBEAT_INTERVAL, Telem::heartbeat_ticker_callback);
}

/**
 * @brief Telemtry Heartbeat Ticker Callback
 * @return Nada
 */
void Telem::heartbeat_ticker_callback()
{
    if (self)
    {
        // Led action
        digitalWrite(LED_BUILTIN, (!self->led_status) ? LOW : HIGH);

        // Enviamos heartbeat
        self->heartbeat(SYSID, COMPID, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_UNINIT);

        // Comprobamos link
        self->check_link();

        // Latido led
        self->led_status = (!self->led_status) ? true : false;
    }
}

/**
 * @brief Telemtry Run
 * @return Nada
 */
void Telem::run()
{
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

                        // Log.notice("Lat: %d, Lon: %d, Alt: %d, Rel Alt: %d, Vx: %d, Vy: %d, Vz: %d, Hdg: %d" CR, APdata.lat, APdata.lon, APdata.alt, APdata.relative_alt, APdata.vx, APdata.vy, APdata.vz, APdata.hdg);

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

void Telem::send_to_fc(mavlink_message_t msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialPort.write(buf, len);
}

// Latido
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

void Telem::set_param_value(std::string param_name, float param_value)
{
    mavlink_message_t msg;
    mavlink_msg_param_set_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, param_name.c_str(), param_value, MAVLINK_TYPE_UINT8_T);
    send_to_fc(msg);
}

void Telem::request_param_value(std::string param_name)
{
    mavlink_message_t msg;
    mavlink_msg_param_request_read_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, param_name.c_str(), -1);
    send_to_fc(msg);
}

void Telem::request_data_streams(uint8_t req_stream_id, uint16_t req_message_rate, uint8_t start_stop)
{
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(SYSID, COMPID, &msg, TARGET_SYSID, TARGET_COMPID, req_stream_id, req_message_rate, start_stop);
    send_to_fc(msg);
}

void Telem::do_change_speed(uint16_t speed)
{
    if (SPEED_OFFSET > 0)
    {
        speed = static_cast<uint16_t>(round(speed * (1 + (SPEED_OFFSET / 100.0))));
        Log.notice("Final Speed (uint16_t): %d" CR, speed); // Verificar valor final
    }

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
        -1,                       // Param 3: Throttle as a percentage (0-100%). A value of 0 or negative indicates no change.
        0,                       // Param 4: Empty
        0,                       // Param 5: Empty
        0,                       // Param 6: Empty
        0                        // Param 7: Empty
    );
    send_to_fc(msg);
}

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

// Deprecated
void Telem::do_reposition(int32_t lat, int32_t lon, float alt, uint16_t hdg)
{

    int32_t lat_offset = 0;
    int32_t lon_offset = 0;

    if (lat_offset != 0)
    {
        lat = (lat + lat_offset) * cos(hdg);
    }

    if (lon_offset != 0)
    {
        lon = (lon + lon_offset) * sin(hdg);
    }

    if (ALT_OFFSET != 0)
    {
        alt = alt - (ALT_OFFSET * 1000);
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

// Functions
void Telem::init_setup()
{
    // Seteamos parámetros
    if (AUTO_SET_FOLL_PARAMS)
    {
        set_param_value("FOLL_ENABLE", FOLL_ENABLE);
        set_param_value("FOLL_OFS_TYPE", FOLL_OFS_TYPE);
        set_param_value("FOLL_ALT_TYPE", FOLL_ALT_TYPE);
    }

    // Setup done
    init_setup_done = true;

    // Mandamos init setup done
    status_text("Init Setup Done");
}

void Telem::check_link()
{
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

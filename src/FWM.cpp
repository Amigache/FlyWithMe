#include "FWM.h"

FWM *FWM::self = nullptr;

FWM::FWM()
{
    self = this;
}

void FWM::begin()
{
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

    // Default Follow mode
    if (!MAV_BRIDGE)
    {
        changeFollowMode(FOLL_MODE);
    }

    Log.info("FWM Ready" CR);
}

/**
 * @brief Main loop
 */
void FWM::run()
{
    // State machine
    if (follow_mode == FOLL_MODE_FOLLOWER) // Only work if we are on follower mode
    {
        if (mav->APdata.custom_mode == MODE_GUIDED) // Only work if we are on guided mode
        {
            if (comm->commData.have_beacon && stage_follow == STAGE_IDLE)
            {
                stage_follow = STAGE_APPROACH;
            }
            else if (!comm->commData.have_beacon && stage_follow == STAGE_APPROACH)
            {
                stage_follow = STAGE_IDLE;
            }
        }
        else if (stage_follow == STAGE_APPROACH) // Mode changed, stop follow
        {
            stage_follow = STAGE_IDLE;
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
        self->comm->sendPacket(packet);

        // mostrar por log todos valores del paquete
        Log.notice("Send Packet: %d, %d, %d, %d, %d, %d, %d, %d" CR, packet.sysid, packet.lat, packet.lon, packet.alt, packet.relative_alt, packet.ground_speed, packet.hdg, packet.checksum);
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
#include "Web.h"

// Set web server port number (legacy, usar AsyncWebServer en Fase 4)
#if !USE_WEB_SERVER
WiFiServer server(WEB_PORT);
#endif

// Variable to store the HTTP request
String header;

Web::Web(FWM *fwm)
{
  this->fwm = fwm;
}

void Web::begin()
{
  #if USE_WEB_SERVER
  setupWebServer();
  #endif
}

void Web::startAP()
{
  // Start the server
  Log.notice("Init Access Point" CR);

  // IMPORTANTE: Configurar modo WiFi ANTES de iniciar AP
  WiFi.mode(WIFI_AP);
  delay(300);  // Dar tiempo al WiFi para (re)inicializar

  WiFi.softAP(fwm->params.ssid, fwm->params.pass);

  // Esperar a que el AP esté completamente activo
  delay(500);

  Log.notice("AP SSID: %s" CR, fwm->params.ssid);
  Log.notice("AP Password: %s" CR, fwm->params.pass);

  host_ip = WiFi.softAPIP();
  Log.notice("AP IP address: %s" CR, host_ip.toString().c_str());

  Log.notice("Access Point Ready" CR);
  
  // Marcar servidor como activo (para ambos modos)
  server_up = true;

  #if !USE_WEB_SERVER
  Log.notice("Init WebServer" CR);
  server.begin();
  Log.notice("Url: http://%s:%d" CR,host_ip.toString().c_str(), WEB_PORT);
  Log.notice("WebServer Ready" CR);
  #endif
}

void Web::stopAP()
{
  // Apaga SOLO el softAP (sin tumbar el stack WiFi), para poder re-levantarlo luego.
  WiFi.softAPdisconnect(true);
  server_up = false;
  ap_info_shown = false;
  Log.notice("AP detenido (no en tierra)" CR);
}

void Web::run()
{
  // Mostrar información del AP cuando hay linkTimeout
  if (fwm->mav->linkTimeout && !fwm->mav->lock_ap)
  {
    if (!server_up)
    {
      // AP no iniciado - iniciar con mensajes
      fwm->screen->showCenterText("Not FC connection");
      delay(1000);
      fwm->screen->showCenterText("Starting AP Mode");
      delay(1000);

      // Start the server
      startAP();
    }
    #if WEB_START_AP_IMMEDIATELY
    else if (server_up && !ap_info_shown)
    {
      // AP ya iniciado - solo mostrar mensajes una vez
      fwm->screen->showCenterText("Not FC connection");
      delay(1000);
      fwm->screen->showCenterText("AP Mode Active");
      delay(1000);
      ap_info_shown = true;
    }
    #endif
    else
    {
      // Display server data
      fwm->screen->showServerData(fwm->params.ssid, fwm->params.pass, host_ip);

      #if !USE_WEB_SERVER
      WiFiClient client = server.available(); // Listen for incoming clients

      if (client)
      {                               // If a new client connects,
        Log.notice("New Client." CR); // print a message out in the serial port
        String currentLine = "";      // make a String to hold incoming data from the client
        bool isPost = false;          // Track if it's a POST request
        String postBody = "";         // To hold the body of POST data

        // Read the HTTP request headers
        while (client.connected())
        { // loop while the client's connected
          if (client.available())
          {                         // if there's bytes to read from the client,
            char c = client.read(); // read a byte, then
            header += c;            // Add to header

            // Detect if it's a POST request
            if (header.indexOf("POST /save") >= 0)
            {
              isPost = true;
            }

            // Read until the end of the request
            if (c == '\n' && currentLine.length() == 0)
            {
              // POST requests have a body after the headers
              if (isPost)
              {
                // Read the body of the POST request
                while (client.available())
                {
                  char bodyChar = client.read();
                  postBody += bodyChar;
                }

                // Extract the parameter "link_stab_timeout" from the POST body
                String linkStabTimeout = getPostParam(postBody, "link_stab_timeout");
                if (linkStabTimeout.length() > 0)
                {
                  // Convert the parameter to integer and update params
                  fwm->params.link_timeout = linkStabTimeout.toInt();
                }

                // Extract the parameter "ssid" from the POST body
                String ssid = getPostParam(postBody, "ssid");
                if (ssid.length() > 0)
                {
                  // Update params
                  ssid.toCharArray(fwm->params.ssid, sizeof(fwm->params.ssid));
                }

                // Extract the parameter "pass" from the POST body
                String pass = getPostParam(postBody, "pass");
                if (pass.length() > 0)
                {
                  // Update params
                  pass.toCharArray(fwm->params.pass, sizeof(fwm->params.pass));
                }

                fwm->saveParams();

                // Optionally restart after saving params
                esp_restart();
              }

              // Send HTTP response headers
              client.println("HTTP/1.1 200 OK");
              client.println("Content-type:text/html");
              client.println("Connection: close");
              client.println();

              // Display the HTML web page
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;}");
              client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
              client.println(".button2 {background-color: #555555;}</style></head>");

              // Web Page Heading
              client.println("<body><h1>FWM Web Server</h1>");
              client.println("<form method=\"POST\" action=\"/save\">");

              // Display the current link_stab_timeout value
              client.println("<p>AP enter Timeout (s): <input type=\"text\" id=\"link_stab_timeout\" name=\"link_stab_timeout\" value=\"" + String(fwm->params.link_timeout) + "\"></p>");

              // Display the current SSID value
              client.println("<p>SSID: <input type=\"text\" id=\"ssid\" name=\"ssid\" value=\"" + String(fwm->params.ssid) + "\"></p>");

              // Display the current Password value
              client.println("<p>Password: <input type=\"text\" id=\"pass\" name=\"pass\" value=\"" + String(fwm->params.pass) + "\"></p>");

              client.println("<p><button type=\"submit\" class=\"button button2\">Save and Reboot</button></p>");
              client.println("</form>");

              client.println("</body></html>");
              client.println();
              break;
            }

            if (c == '\n')
            {
              currentLine = "";
            }
            else if (c != '\r')
            {
              currentLine += c;
            }
          }
        }

        // Clear the header variable
        header = "";
        // Close the connection
        client.stop();
        Log.notice("Client disconnected." CR);
      }
      #endif
    }
  }
  
  #if USE_WEB_SERVER && USE_WEBSOCKET && WEB_TELEMETRY_WS
  // Telemetría en vivo por WebSocket (retirada del flujo normal; solo diagnóstico en tierra)
  sendTelemetryWebSocket();
  #endif
}

// Función auxiliar para decodificar URL
String Web::urlDecode(String input)
{
  String decoded = "";
  char temp[] = "00"; // Para almacenar cada par de caracteres hexadecimales

  for (uint16_t i = 0; i < input.length(); i++)
  {
    if (input[i] == '+')
    {
      decoded += ' '; // Reemplaza los '+' con espacios
    }
    else if (input[i] == '%')
    {
      // Convierte el par hexadecimal a un carácter ASCII
      if (i + 2 < input.length())
      {
        temp[0] = input[i + 1];
        temp[1] = input[i + 2];
        decoded += (char)strtol(temp, NULL, 16); // Convierte el valor hexadecimal a char
        i += 2;                                  // Salta los dos caracteres hexadecimales
      }
    }
    else
    {
      decoded += input[i]; // Añade caracteres normales
    }
  }
  return decoded;
}

// Función auxiliar para extraer y decodificar un parámetro del cuerpo de la solicitud POST
String Web::getPostParam(String postBody, String paramName)
{
  // Encuentra el parámetro en el cuerpo
  int paramStart = postBody.indexOf(paramName + "=");
  if (paramStart == -1)
    return "";

  // Determina dónde empieza y termina el valor
  int valueStart = paramStart + paramName.length() + 1;
  int valueEnd = postBody.indexOf("&", valueStart);
  if (valueEnd == -1)
    valueEnd = postBody.length();

  // Extrae el valor sin decodificar
  String rawValue = postBody.substring(valueStart, valueEnd);

  // Decodifica el valor y lo retorna
  return urlDecode(rawValue);
}

// ============================================================================
// FASE 4: Servidor Web y WebSocket
// ============================================================================

#if USE_WEB_SERVER

void Web::setupWebServer()
{
  if (server != nullptr) return;
  
  server = new AsyncWebServer(WEB_SERVER_PORT);
  
  // Ruta principal - Panel de control HTML
  server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
    request->send(200, "text/html", generateHTML());
  });
  
  // API REST - Obtener configuración
  server->on("/api/config", HTTP_GET, [this](AsyncWebServerRequest *request){
    String cfg = "{";
    cfg += "\"formation\":" + String((int)fwm->mav->currentFormation) + ",";
    cfg += "\"formation_name\":\"" + String(fwm->mav->getFormationName(fwm->mav->currentFormation)) + "\",";
    cfg += "\"prediction\":" + String(fwm->mav->predictionEnabled ? "true" : "false") + ",";
    cfg += "\"filter\":" + String(fwm->mav->filterEnabled ? "true" : "false");
    cfg += "}";
    request->send(200, "application/json", cfg);
  });
  
  // API REST - Establecer configuración
  server->on("/api/config", HTTP_POST, [this](AsyncWebServerRequest *request){
    // Seguridad: no permitir cambios de configuracion en vuelo (solo en tierra / sin FC)
    if (!fwm->isOnGround()) {
      request->send(403, "application/json", generateAPIResponse(false, "Config bloqueada: vehiculo en vuelo"));
      return;
    }
    // Formación: 0=TRAIL, 1=LEFT, 2=RIGHT, 3=ABOVE, 4=BELOW
    if (request->hasParam("formation", true)) {
      fwm->setFormation((uint8_t)request->getParam("formation", true)->value().toInt());
    }
    if (request->hasParam("prediction", true)) {
      String p = request->getParam("prediction", true)->value();
      fwm->setPrediction(p == "1" || p == "true" || p == "on");
    }
    if (request->hasParam("filter", true)) {
      String f = request->getParam("filter", true)->value();
      fwm->setFilter(f == "1" || f == "true" || f == "on");
    }
    request->send(200, "application/json", generateAPIResponse(true, "Config updated"));
  });

  // API REST - Parametros FWM (Fase 2): GET lista completa; POST set por clave (x-www-form-urlencoded)
  server->on("/api/params", HTTP_GET, [this](AsyncWebServerRequest *request){
    request->send(200, "application/json", fwm->paramsJson());
  });
  server->on("/api/params", HTTP_POST, [this](AsyncWebServerRequest *request){
    bool ok = true;
    String err = "ok";
    for (int i = 0; i < fwm->paramCount(); i++) {
      const ParamDef_t *d = fwm->paramDefAt(i);
      if (d && request->hasParam(d->key, true)) {
        String v = request->getParam(d->key, true)->value();
        String e;
        if (!fwm->setParamByKey(d->key, v.c_str(), e)) {
          ok = false;
          err = e;
        }
      }
    }
    request->send(ok ? 200 : 400, "application/json", generateAPIResponse(ok, err.c_str()));
  });
  
  // API REST - Obtener estadísticas
  server->on("/api/stats", HTTP_GET, [this](AsyncWebServerRequest *request){
    uint32_t rx = fwm->comm->commData.rx_packet_counter;
    uint32_t lost = fwm->comm->commData.lost_packet_counter;
    int lossPct = (rx + lost) > 0 ? (int)((lost * 100UL) / (rx + lost)) : 0;
    String stats = "{";
    stats += "\"uptime\":" + String(millis()) + ",";
    stats += "\"rx_packets\":" + String(rx) + ",";
    stats += "\"tx_packets\":" + String(fwm->comm->commData.tx_packet_counter) + ",";
    stats += "\"lost_packets\":" + String(lost) + ",";
    stats += "\"packet_loss\":" + String(lossPct) + ",";
    stats += "\"rssi\":" + String(fwm->comm->commData.rssi) + ",";
    stats += "\"snr\":" + String(fwm->comm->commData.snr) + ",";
    stats += "\"distance\":" + String((int)fwm->getLinkDistance()) + ",";
    stats += "\"state\":\"" + String(fwm->getStateName(fwm->currentState)) + "\",";
    stats += "\"on_ground\":" + String(fwm->isOnGround() ? "true" : "false") + ",";
    stats += "\"link_timeout\":" + String(fwm->mav->linkTimeout ? "true" : "false") + ",";
    stats += "\"armed\":" + String((int)fwm->mav->APdata.armed) + ",";
    stats += "\"gs_cms\":" + String((int)fwm->mav->APdata.ground_speed) + ",";
    stats += "\"rel_alt_mm\":" + String((int)fwm->mav->APdata.relative_alt);
    stats += "}";
    request->send(200, "application/json", stats);
  });
  
  // API REST - Obtener logs
  server->on("/api/logs", HTTP_GET, [this](AsyncWebServerRequest *request){
    if (!SPIFFS.begin()) {
      request->send(500, "text/plain", "SPIFFS error");
      return;
    }
    
    File logFile = SPIFFS.open("/flight.log", "r");
    if (!logFile) {
      request->send(404, "text/plain", "Log file not found");
      return;
    }
    
    String logs = "";
    while (logFile.available()) {
      logs += (char)logFile.read();
    }
    logFile.close();
    
    request->send(200, "text/plain", logs);
  });
  
  // WebSocket para telemetría en tiempo real
  setupWebSocket();
  
  server->begin();
  Log.notice("Servidor web iniciado en puerto %d" CR, WEB_SERVER_PORT);
}

void Web::setupWebSocket()
{
  if (ws != nullptr) return;
  
  ws = new AsyncWebSocket("/ws");
  
  ws->onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, 
                 AwsEventType type, void *arg, uint8_t *data, size_t len){
    if (type == WS_EVT_CONNECT) {
      Log.notice("WebSocket client conectado: %u" CR, client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      Log.notice("WebSocket client desconectado: %u" CR, client->id());
    }
  });
  
  server->addHandler(ws);
  Log.notice("WebSocket iniciado en /ws" CR);
}

void Web::sendTelemetryWebSocket()
{
  if (ws == nullptr || ws->count() == 0) return;
  
  // Limitar frecuencia de envío
  uint32_t now = millis();
  if (now - lastWSBroadcast < 200) return;  // Máximo 5 Hz
  lastWSBroadcast = now;
  
  // Crear mensaje JSON con telemetría
  String telemetry = "{";
  telemetry += "\"timestamp\":" + String(now) + ",";
  telemetry += "\"lat\":" + String(fwm->mav->APdata.lat / 1e7, 7) + ",";
  telemetry += "\"lon\":" + String(fwm->mav->APdata.lon / 1e7, 7) + ",";
  telemetry += "\"alt\":" + String(fwm->mav->APdata.relative_alt / 1000.0, 2) + ",";
  telemetry += "\"heading\":" + String(fwm->mav->APdata.hdg / 100.0, 2) + ",";
  telemetry += "\"speed\":" + String(fwm->mav->APdata.ground_speed) + ",";
  telemetry += "\"rssi\":" + String(fwm->comm->commData.rssi) + ",";
  telemetry += "\"snr\":" + String(fwm->comm->commData.snr) + ",";
  telemetry += "\"distance\":" + String((int)fwm->getLinkDistance()) + ",";
  telemetry += "\"state\":\"" + String(fwm->getStateName(fwm->currentState)) + "\"";
  telemetry += "}";
  
  ws->textAll(telemetry);
}

String Web::generateHTML()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>FlyWithMe</title>
  <style>
    :root{--bg:#0f1115;--card:#171a21;--fg:#e7eaf0;--mut:#8b93a7;--acc:#3da9fc;--ok:#2ecc71;--err:#ff5c5c;--bd:#262b36}
    *{box-sizing:border-box}
    body{margin:0;font:14px/1.45 system-ui,Segoe UI,Roboto,Arial,sans-serif;background:var(--bg);color:var(--fg)}
    header{display:flex;align-items:center;gap:8px;padding:13px 16px;border-bottom:1px solid var(--bd);position:sticky;top:0;background:var(--bg);z-index:2}
    h1{font-size:16px;margin:0;font-weight:600}
    .sp{margin-left:auto}
    .badge{font-size:12px;padding:3px 9px;border-radius:99px;background:#20242e;color:var(--mut)}
    .badge.g{background:#12351f;color:var(--ok)}.badge.a{background:#3a1a1a;color:var(--err)}
    .lang{cursor:pointer;background:#20242e;color:var(--fg);border:0;border-radius:8px;padding:4px 9px;font:600 12px inherit}
    main{max-width:760px;margin:0 auto;padding:14px}
    section{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:14px;margin:12px 0}
    h2{font-size:12px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.07em}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px}
    .kv{display:flex;justify-content:space-between;gap:8px;padding:6px 9px;background:#12151b;border-radius:8px}
    .kv :first-child{color:var(--mut)}.kv :last-child{font-variant-numeric:tabular-nums}
    .ptable{display:flex;flex-direction:column}
    .prow{display:grid;grid-template-columns:175px 1fr;gap:14px;align-items:center;padding:10px 0;border-bottom:1px solid var(--bd)}
    .prow:last-child{border-bottom:0}
    label{display:flex;flex-direction:column;gap:4px;font-size:12px;color:var(--mut)}
    input,select{background:#0d1015;color:var(--fg);border:1px solid var(--bd);border-radius:8px;padding:8px;font:inherit;width:100%}
    input:focus,select:focus{outline:0;border-color:var(--acc)}
    button{background:var(--acc);color:#04121f;border:0;border-radius:9px;padding:9px 14px;font:600 13px inherit;cursor:pointer}
    button.sec{background:#20242e;color:var(--fg)}
    button:hover{filter:brightness(1.08)}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:12px}
    .msg{font-size:12px;color:var(--mut)}
    .chk{flex-direction:row;align-items:center;gap:8px}.chk input{width:auto}
    .desc{color:var(--mut);font-size:13px;line-height:1.45}
    .desc b{color:var(--acc)}
    @media(max-width:560px){.prow{grid-template-columns:1fr;gap:5px;padding:10px 0}}
  </style>
</head>
<body>
  <header>
    <h1>🛸 FlyWithMe</h1>
    <span class="sp"></span>
    <span id="gt" class="badge">—</span>
    <button class="lang" id="lang" onclick="toggleLang()">EN</button>
  </header>
  <main>
    <section><h2 id="h_stats">Status</h2><div class="grid" id="stats"></div></section>
    <section><h2 id="h_cfg">Configuration</h2>
      <form id="pf" class="ptable" onsubmit="return saveP(event)"></form>
      <div class="row"><button type="submit" form="pf" id="saveBtn">Save</button><span id="pm" class="msg"></span></div>
    </section>
    <section><h2 id="h_act">Actions</h2>
      <div class="row">
        <button class="sec" id="btnLogs" onclick="location.href='/api/logs'">Download logs</button>
        <button class="sec" id="btnReload" onclick="location.reload()">Reload</button>
      </div>
    </section>
  </main>
  
  <script>
    const $=id=>document.getElementById(id);
    const I18N={
    en:{stats:'Status',cfg:'Configuration',act:'Actions',save:'Save',logs:'Download logs',reload:'Reload',ground:'ON GROUND',flight:'IN FLIGHT',
     st:{state:'State',up:'Uptime',rx:'RX / TX',rssi:'RSSI',snr:'SNR',loss:'Losses',dist:'Distance'},
     help:'Configuration help',hdef:'Adjust FWM parameters here <b>on the ground</b>, then press Save. Changes are stored on the device and applied immediately.',
     saved:'Saved',serr:'Save failed',lerr:'Error loading parameters',cur:'Current value',
     form:['Trail','Left','Right','Above','Below'],
     lb:{formation:'Formation',dist_offset:'Trail distance',lateral_offset:'Lateral offset',vertical_offset:'Vertical offset',cross_gain:'Lateral gain',hdg_corr_max:'Max heading correction',along_gain:'Longitudinal gain',prediction:'Prediction',filter:'Position filter',foll_enable:'Follow enable',link_timeout:'Link timeout',netid:'Network ID',approach_dist:'Approach distance'},
     d:{formation:'Geometry relative to the leader: <b>Trail</b> behind, <b>Left/Right</b> lateral, <b>Above/Below</b> vertical.',
        dist_offset:'Longitudinal separation behind the leader for TRAIL (m). Typical ~90-110 m.',
        lateral_offset:'Sideways separation for LEFT/RIGHT (m).',
        vertical_offset:'Vertical separation for ABOVE/BELOW (m); ABOVE adds, BELOW subtracts.',
        cross_gain:'Lateral steering: heading correction per meter of cross-track error (deg/m). 0.5-0.8 typical; too high may oscillate.',
        hdg_corr_max:'Cap on the heading correction (deg). 25-45 typical.',
        along_gain:'Speed correction per meter of longitudinal error (cm/s per m). 10-20 typical.',
        prediction:'Extrapolate the leader position to compensate for radio latency (recommended ON).',
        filter:'Low-pass filter on the target position: smoother but adds a small lag.',
        foll_enable:'Enables the following function.',
        link_timeout:'Seconds without a heartbeat before the FC link is considered lost.',
        netid:'Shared network ID ("phrase"): packets from other IDs are ignored. Must match on both aircraft. 0 = accept any.',
        approach_dist:'Below this distance (m) the follower only keeps following if the leader is in a stable mode (FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED).'}},
    es:{stats:'Estado',cfg:'Configuración FWM',act:'Acciones',save:'Guardar',logs:'Descargar logs',reload:'Recargar',ground:'EN TIERRA',flight:'EN VUELO',
     st:{state:'Estado',up:'Tiempo',rx:'RX / TX',rssi:'RSSI',snr:'SNR',loss:'Pérdidas',dist:'Distancia'},
     help:'Ayuda de configuración',hdef:'Ajusta aquí los parámetros de FWM <b>en tierra</b> y pulsa Guardar. Se guardan en el dispositivo y se aplican al momento.',
     saved:'Guardado',serr:'Error al guardar',lerr:'Error al cargar parámetros',cur:'Valor actual',
     form:['Cola','Izquierda','Derecha','Arriba','Abajo'],
     lb:{formation:'Formación',dist_offset:'Distancia TRAIL',lateral_offset:'Offset lateral',vertical_offset:'Offset vertical',cross_gain:'Ganancia lateral',hdg_corr_max:'Corrección de rumbo máx',along_gain:'Ganancia longitudinal',prediction:'Predicción',filter:'Filtro de posición',foll_enable:'Activar seguimiento',link_timeout:'Timeout de enlace',netid:'ID de red',approach_dist:'Distancia de aproximación'},
     d:{formation:'Geometría relativa al líder: <b>Cola</b> detrás, <b>Izquierda/Derecha</b> lateral, <b>Arriba/Abajo</b> vertical.',
        dist_offset:'Separación longitudinal detrás del líder para TRAIL (m). Típico ~90-110 m.',
        lateral_offset:'Separación lateral para Izquierda/Derecha (m).',
        vertical_offset:'Separación vertical para Arriba/Abajo (m); Arriba suma, Abajo resta.',
        cross_gain:'Guiado lateral: corrección de rumbo por metro de error lateral (deg/m). 0.5-0.8 típico; más puede oscilar.',
        hdg_corr_max:'Tope de corrección de rumbo (deg). 25-45 típico.',
        along_gain:'Corrección de velocidad por metro de error longitudinal (cm/s por m). 10-20 típico.',
        prediction:'Extrapola la posición del líder para compensar la latencia de radio (recomendado ON).',
        filter:'Filtro paso bajo sobre la posición objetivo: más suave pero con algo de retraso.',
        foll_enable:'Activa la función de seguimiento.',
        link_timeout:'Segundos sin heartbeat antes de considerar perdido el enlace con el FC.',
        netid:'ID de red compartido ("frase"): se ignoran paquetes de otros IDs. Debe coincidir en ambos aviones. 0 = aceptar cualquiera.',
        approach_dist:'Por debajo de esta distancia (m) el seguidor solo sigue si el líder está en un modo estable (FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED).'}}
    };
    let P=[],LANG=localStorage.getItem('lang')||(((navigator.language||'en').slice(0,2)==='es')?'es':'en');
    const L=()=>I18N[LANG];
    function toggleLang(){LANG=(LANG==='en'?'es':'en');localStorage.setItem('lang',LANG);applyI18n();if(P.length)drawForm(P);}
    function applyI18n(){
      $('h_stats').textContent=L().stats;$('h_cfg').textContent=L().cfg;$('h_act').textContent=L().act;
      $('saveBtn').textContent=L().save;$('btnLogs').textContent=L().logs;$('btnReload').textContent=L().reload;
      $('lang').textContent=LANG.toUpperCase();document.documentElement.lang=LANG;
    }
    const stat=(k,v)=>'<div class="kv"><span>'+k+'</span><span>'+v+'</span></div>';
    function drawStats(d){
      const s=L().st;
      $('stats').innerHTML=stat(s.state,d.state)+stat(s.up,(d.uptime/1000|0)+' s')+stat(s.rx,d.rx_packets+' / '+d.tx_packets)+
        stat(s.rssi,d.rssi+' dBm')+stat(s.snr,d.snr)+stat(s.loss,d.lost_packets+' ('+d.packet_loss+'%)')+
        stat(s.dist,d.distance>=0?d.distance+' m':'--');
      const g=$('gt');g.textContent=d.on_ground?L().ground:L().flight;g.className='badge '+(d.on_ground?'g':'a');
    }
    function drawForm(list){
      P=list;let h='';
      list.forEach(p=>{const id='p_'+p.key,t=p.type,lb=L().lb[p.key]||p.label,u=p.unit?' ('+p.unit+')':'';
        let fld;
        if(t===3)fld='<label>'+lb+'<select id="'+id+'">'+L().form.map((n,i)=>'<option value="'+i+'">'+n+'</option>').join('')+'</select></label>';
        else if(t===2)fld='<label class="chk"><input type="checkbox" id="'+id+'">'+lb+'</label>';
        else fld='<label>'+lb+u+'<input type="number" step="any" min="'+p.min+'" max="'+p.max+'" id="'+id+'"></label>';
        h+='<div class="prow">'+fld+'<div class="desc">'+(L().d[p.key]||'')+'</div></div>';});
      $('pf').innerHTML=h;
      list.forEach(p=>{const e=$('p_'+p.key);if(!e)return;if(p.type===2)e.checked=p.value>=0.5;else e.value=p.value;});
    }
    function saveP(ev){
      ev.preventDefault();const fd=new FormData();
      P.forEach(p=>{const e=$('p_'+p.key);if(!e)return;fd.append(p.key,p.type===2?(e.checked?'1':'0'):e.value);});
      fetch('/api/params',{method:'POST',body:fd}).then(r=>r.json())
        .then(d=>$('pm').textContent=(d.success?'✓ ':'✗ ')+(d.success?L().saved:(d.message||L().serr)))
        .catch(()=>$('pm').textContent='✗ '+L().serr);
      return false;
    }
    function poll(){fetch('/api/stats').then(r=>r.json()).then(drawStats).catch(()=>{});}
    applyI18n();
    fetch('/api/params').then(r=>r.json()).then(drawForm).catch(()=>$('pf').textContent=L().lerr);
    poll();setInterval(poll,2000);
  </script>
</body>
</html>
)rawliteral";
  
  return html;
}

String Web::generateAPIResponse(bool success, const char* message)
{
  String response = "{";
  response += "\"success\":" + String(success ? "true" : "false") + ",";
  response += "\"message\":\"" + String(message) + "\"";
  response += "}";
  return response;
}

void Web::handleGetConfig()
{
  // TODO: Implementar obtener configuración actual
}

void Web::handleSetConfig()
{
  // TODO: Implementar guardar configuración
}

void Web::handleGetStats()
{
  // Implementado inline en setupWebServer()
}

void Web::handleGetLogs()
{
  // Implementado inline en setupWebServer()
}

#endif // USE_WEB_SERVER

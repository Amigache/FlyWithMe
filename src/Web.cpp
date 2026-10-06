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
  delay(100);  // Dar tiempo al WiFi para inicializar

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
  
  #if USE_WEB_SERVER && USE_WEBSOCKET
  // Enviar telemetría por WebSocket si hay clientes conectados
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
    stats += "\"state\":\"" + String(fwm->getStateName(fwm->currentState)) + "\"";
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
  <title>FlyWithMe Control Panel</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
    .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
    h1 { color: #333; text-align: center; }
    .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }
    .stat { display: flex; justify-content: space-between; margin: 10px 0; }
    .stat-label { font-weight: bold; }
    .stat-value { color: #007bff; }
    button { padding: 10px 20px; margin: 5px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
    button:hover { background: #0056b3; }
    select, input { padding: 8px; margin: 5px; border-radius: 5px; border: 1px solid #ddd; }
    #map { height: 400px; width: 100%; border: 1px solid #ddd; margin: 10px 0; }
    .status-ok { color: green; }
    .status-error { color: red; }
  </style>
</head>
<body>
  <div class="container">
    <h1>🛸 FlyWithMe Control Panel</h1>
    
    <div class="section">
      <h2>Estado del Sistema</h2>
      <div class="stat"><span class="stat-label">Uptime:</span><span class="stat-value" id="uptime">-</span></div>
      <div class="stat"><span class="stat-label">Packets RX:</span><span class="stat-value" id="rx">-</span></div>
      <div class="stat"><span class="stat-label">Packets TX:</span><span class="stat-value" id="tx">-</span></div>
      <div class="stat"><span class="stat-label">RSSI:</span><span class="stat-value" id="rssi">-</span></div>
      <div class="stat"><span class="stat-label">SNR:</span><span class="stat-value" id="snr">-</span></div>
      <div class="stat"><span class="stat-label">Perdidas:</span><span class="stat-value" id="lost">-</span></div>
      <div class="stat"><span class="stat-label">Distancia:</span><span class="stat-value" id="dist">-</span></div>
      <div class="stat"><span class="stat-label">Estado:</span><span class="stat-value" id="state">-</span></div>
    </div>
    
    <div class="section">
      <h2>Configuración</h2>
      <div>
        <label>Formación:</label>
        <select id="formation">
          <option value="0">Trail</option>
          <option value="1">Left</option>
          <option value="2">Right</option>
          <option value="3">Above</option>
          <option value="4">Below</option>
        </select>
      </div>
      <div>
        <label>Predicción: <input type="checkbox" id="prediction"></label>
        <label>Filtro: <input type="checkbox" id="filter"></label>
      </div>
      <button onclick="saveConfig()">Guardar Config</button>
    </div>
    
    <div class="section">
      <h2>Telemetría en Tiempo Real</h2>
      <div class="stat"><span class="stat-label">Latitud:</span><span class="stat-value" id="lat">-</span></div>
      <div class="stat"><span class="stat-label">Longitud:</span><span class="stat-value" id="lon">-</span></div>
      <div class="stat"><span class="stat-label">Altitud:</span><span class="stat-value" id="alt">-</span></div>
      <div class="stat"><span class="stat-label">Velocidad:</span><span class="stat-value" id="speed">-</span></div>
    </div>
    
    <div class="section">
      <h2>Acciones</h2>
      <button onclick="calibrateLora()">Calibrar LoRa</button>
      <button onclick="downloadLogs()">Descargar Logs</button>
      <button onclick="resetStats()">Reset Estadísticas</button>
    </div>
  </div>
  
  <script>
    // WebSocket para telemetría en tiempo real
    let ws = new WebSocket('ws://' + window.location.hostname + ':81/ws');
    
    ws.onmessage = function(event) {
      let data = JSON.parse(event.data);
      document.getElementById('lat').textContent = data.lat.toFixed(7);
      document.getElementById('lon').textContent = data.lon.toFixed(7);
      document.getElementById('alt').textContent = data.alt.toFixed(2) + ' m';
      document.getElementById('speed').textContent = data.speed + ' cm/s';
    };
    
    // Actualizar estadísticas cada segundo
    setInterval(updateStats, 1000);

    // Cargar la configuración actual en los controles
    fetch('/api/config').then(r => r.json()).then(c => {
      document.getElementById('formation').value = c.formation;
      document.getElementById('prediction').checked = c.prediction;
      document.getElementById('filter').checked = c.filter;
    }).catch(() => {});
    
    function updateStats() {
      fetch('/api/stats')
        .then(r => r.json())
        .then(data => {
          document.getElementById('uptime').textContent = (data.uptime / 1000).toFixed(0) + ' s';
          document.getElementById('rx').textContent = data.rx_packets;
          document.getElementById('tx').textContent = data.tx_packets;
          document.getElementById('rssi').textContent = data.rssi + ' dBm';
          document.getElementById('snr').textContent = data.snr;
          document.getElementById('lost').textContent = data.lost_packets + ' (' + data.packet_loss + '%)';
          document.getElementById('dist').textContent = (data.distance >= 0 ? data.distance + ' m' : '--');
          document.getElementById('state').textContent = data.state;
        });
    }
    
    function saveConfig() {
      let formation = document.getElementById('formation').value;
      let prediction = document.getElementById('prediction').checked;
      let filter = document.getElementById('filter').checked;
      
      let formData = new FormData();
      formData.append('formation', formation);
      formData.append('prediction', prediction);
      formData.append('filter', filter);
      
      fetch('/api/config', {method: 'POST', body: formData})
        .then(r => r.json())
        .then(data => alert(data.message));
    }
    
    function calibrateLora() {
      alert('Calibrando LoRa... Por favor espera.');
      // TODO: Implementar endpoint
    }
    
    function downloadLogs() {
      window.open('/api/logs', '_blank');
    }
    
    function resetStats() {
      if (confirm('¿Resetear todas las estadísticas?')) {
        // TODO: Implementar endpoint
      }
    }
    
    // Inicializar
    updateStats();
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

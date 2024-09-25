#include "Web.h"

// Set web server port number
WiFiServer server(WEB_PORT);

// Variable to store the HTTP request
String header;

Web::Web(FWM *fwm)
{
  this->fwm = fwm;
}

void Web::begin()
{
}

void Web::startAP()
{
  // Start the server
  Log.notice("Init Access Point" CR);

  WiFi.softAP(fwm->params.ssid, fwm->params.pass);

  Log.notice("AP SSID: %s" CR, fwm->params.ssid);
  Log.notice("AP Password: %s" CR, fwm->params.pass);

  host_ip = WiFi.softAPIP();
  Log.notice("AP IP address: %s" CR, host_ip.toString().c_str());

  Log.notice("Access Point Ready" CR);

  Log.notice("Init WebServer" CR);
  server.begin();
  server_up = true;
  Log.notice("Url: http://%s:%d" CR,host_ip.toString().c_str(), WEB_PORT);
  Log.notice("WebServer Ready" CR);
}

void Web::run()
{
  if (fwm->mav->linkTimeout && !fwm->mav->lock_ap)
  {

    if (!server_up)
    {

      fwm->screen->showCenterText("Not FC connection");
      delay(1000);
      fwm->screen->showCenterText("Starting AP Mode");
      delay(1000);

      // Start the server
      startAP();
    }
    else
    {

      // Display server data
      fwm->screen->showServerData(fwm->params.ssid, fwm->params.pass, host_ip);

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
    }
  }
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

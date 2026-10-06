# FlyWithMe

**The way to fly with your friends.**

Sistema de **vuelo en formación** para aviones con autopiloto **ArduPilot/PX4**. Un avión
**líder** transmite su posición por radio **LoRa**; uno o más aviones **seguidores** reciben esos
datos, calculan una posición de formación y envían el waypoint resultante a su autopiloto vía
**MAVLink** (modo GUIDED). Está pensado para hardware de bajo coste basado en ESP32.

---

## Características

- 📡 **Enlace LoRa** entre líder y seguidor (paquetes MAVLink simplificados).
- 🛰️ **Modos** líder, seguidor, bridge MAVLink y off.
- 🗜️ **Paquetes comprimidos** (15 B) con compatibilidad hacia atrás para paquetes normales (27 B).
- 🔁 **Reintentos con backoff** y **tasa de transmisión adaptativa** según distancia.
- 🛡️ **Seguridad**: watchdog, validación de datos GPS y límites de seguimiento.
- 🤖 **Máquina de estados** (INIT, SEARCHING, CONNECTING, FOLLOWING, LOST_LINK, EMERGENCY, LANDING).
- 🎯 **Predicción de movimiento**, **formaciones dinámicas** (trail/left/right/above/below) y
  **filtro de posición** paso bajo.
- 🖥️ **Menú OLED** por botones y **servidor web** con API REST + WebSocket de telemetría.
- 📝 **Logging persistente** en SPIFFS con niveles y rotación de archivos.
- 🧪 **Modo simulación** (sin hardware) y tests unitarios.

> Las mejoras anteriores se implementaron por fases; ver la sección **Documentación**.

---

## Hardware

| Elemento | Detalle |
|---|---|
| Placa | TTGO LoRa32 V1 (ESP32 + SX1276) |
| Display | OLED SSD1306 128×64 (I2C: SDA 4, SCL 15, RST 16) |
| Radio | LoRa 866 MHz (Europa) |
| Autopiloto | ArduPilot/PX4 vía serial (57600 baudios, RX 12 / TX 13) |
| Botones menú | UP 12, DOWN 13, SELECT 14, BACK 15 |

> ⚠️ Los pines 12/13 se usan a la vez por el puerto serie y por los botones del menú según la
> configuración; revisa `src/config.h` antes de conectar hardware.

---

## Arquitectura

```
[LÍDER]    autopiloto --MAVLink--> Telem --(LoraPacket)--> Comm --LoRa--> aire
[SEGUIDOR] aire --LoRa--> Comm --(validar/descomprimir)-->
           Telem --(predicción + formación + filtro)--> nav_waypoint --> autopiloto
```

### Estructura del código (`src/`)

| Archivo | Responsabilidad |
|---|---|
| `main.cpp` | Punto de entrada (`setup`/`loop`), instancia global `FWM fwm`. |
| `FWM.h/.cpp` | Orquestador: watchdog, máquina de estados, parámetros, ticker de envío. |
| `Comm.h/.cpp` | LoRa: envío/recepción, checksum, compresión, retry, auto-calibración. |
| `Telem.h/.cpp` | MAVLink con el autopiloto: heartbeat, waypoints, distancia, predicción/formación. |
| `Screen.h/.cpp` | Display OLED y menú interactivo por botones. |
| `Web.h/.cpp` | Servidor web async, API REST y WebSocket de telemetría. |
| `config.h` | Configuración, constantes, structs, enums y clase `Logger`. |
| `lib/mavlink/` | Cabeceras MAVLink generadas (v2). **No editar a mano.** |

---

## Puesta en marcha

Requiere [PlatformIO](https://platformio.org/). Existen dos entornos: **master** (líder) y
**slave** (seguidor).

```bash
# Compilar
pio run -e ttgo-lora32-v1-master
pio run -e ttgo-lora32-v1-slave

# Flashear
pio run -e ttgo-lora32-v1-master -t upload
pio run -e ttgo-lora32-v1-slave  -t upload

# Monitor serie (57600 baudios)
pio device monitor

# Limpieza
pio run -t clean
```

Ajusta `monitor_port` / `upload_port` en `platformio.ini` (por defecto `COM12` para master y
`COM29` para slave) al puerto real de tu equipo.

---

## Configuración

Casi todo se configura con `#define` en [`src/config.h`](src/config.h). Los más relevantes:

| Define | Por defecto | Descripción |
|---|---|---|
| `LORA_BAND` | `866E6` | Frecuencia (Europa). `433E6` Asia / `915E6` Norteamérica. |
| `LORA_SPREADING_FACTOR` | `12` | Spreading factor LoRa. |
| `LORA_TX_POWER` | `20` | Potencia de transmisión (dBm). |
| `USE_COMPRESSED_PACKETS` | `1` | Paquete comprimido (15 B) vs normal (27 B). |
| `ADAPTIVE_RATE` | `1` | Tasa de TX adaptativa según distancia. |
| `USE_PREDICTION` | `1` | Predicción de posición del líder. |
| `USE_POSITION_FILTER` | `1` | Filtro paso bajo de posición. |
| `DEFAULT_FORMATION` | `0` | `0`=TRAIL, `1`=LEFT, `2`=RIGHT, `3`=ABOVE, `4`=BELOW. |
| `MAX_FOLLOW_DISTANCE` | `5000` | Distancia máxima de seguimiento (m). |
| `MIN_SAFE_ALTITUDE` | `50000` | Altitud mínima de seguridad (mm). |
| `USE_INTERACTIVE_MENU` | `1` | Menú OLED por botones. |
| `USE_WEB_SERVER` / `USE_WEBSOCKET` | `1` / `1` | Servidor web (80) + WebSocket (81). |
| `SIMULATION_MODE` | `0` | Simulación sin hardware. |

Además, cada variante define `SYSID`, SSID y `FOLL_MODE` mediante `MASTER_BUILD_FLAG` /
`SLAVE_BUILD_FLAG`.

### Acceso web

Con `WEB_START_AP_IMMEDIATELY = 1` el equipo levanta un punto de acceso WiFi al arrancar.
Conéctate y abre `http://192.168.4.1` para el panel de control (estado, configuración,
telemetría en vivo y descarga de logs).

---

## Tests

```bash
pio test
```

Los tests están en [`test/test_main.cpp`](test/test_main.cpp) (framework Unity, 13 pruebas de
distancia, validación, predicción y compresión).

> **Limitación actual:** `platformio.ini` no define un entorno `[env:native]` y el archivo de
> tests incluye sus propias funciones auxiliares (no enlaza contra `src/`). Para ejecutarlos de
> forma nativa hay que añadir ese entorno y unificar los helpers con el código real.

---

## Documentación

| Documento | Contenido |
|---|---|
| [`docs/FLYWITHME.md`](docs/FLYWITHME.md) | Documentación de desarrollo consolidada: roadmap y Fases 1–4. |
| [`AGENTS.md`](AGENTS.md) | Guía para agentes de código / desarrolladores. |

---

## Seguridad

Este proyecto controla aeronaves. Realiza siempre **pruebas progresivas** (banco → campo sin
vuelo → vuelo) y mantén a un piloto listo para tomar el control manual. Respeta los límites de
seguridad definidos en `src/config.h`.

---

## Licencia

Distribuido bajo la **GNU General Public License v3.0**. Ver [`LICENSE`](LICENSE).

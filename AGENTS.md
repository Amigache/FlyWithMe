# AGENTS.md — Guía para agentes de código (FlyWithMe)

Este documento describe el proyecto, su arquitectura, convenciones y comandos. Está pensado
para que cualquier agente de IA (o desarrollador) pueda trabajar de forma segura y consistente.
El idioma del proyecto y de su documentación es el **español**.

---

## 1. Qué es FlyWithMe

Sistema de **vuelo en formación** para aviones con autopiloto **ArduPilot/PX4**. Un vehículo
**líder** transmite su posición por radio **LoRa**; un vehículo **seguidor** recibe esos datos,
calcula una posición de formación y envía el waypoint resultante a su propio autopiloto vía
**MAVLink** (modo GUIDED).

- **Hardware objetivo:** TTGO LoRa32 V1 (ESP32 + SX1276), OLED SSD1306 128x64, LoRa 866 MHz (Europa).
- **Framework:** Arduino sobre **PlatformIO**.
- **Dos variantes de firmware** (mismo código, distinto `build_flag`):
  - `ttgo-lora32-v1-master` → líder (`MASTER_BUILD_FLAG`)
  - `ttgo-lora32-v1-slave` → seguidor (`SLAVE_BUILD_FLAG`)

---

## 2. Comandos

Ejecutar siempre desde la raíz del repositorio. Requiere PlatformIO (`pio`).

```bash
# Compilar
pio run -e ttgo-lora32-v1-master
pio run -e ttgo-lora32-v1-slave

# Flashear
pio run -e ttgo-lora32-v1-master -t upload
pio run -e ttgo-lora32-v1-slave  -t upload

# Monitor serie (57600 baudios; ver nota más abajo)
pio device monitor

# Limpieza
pio run -t clean
```

- Los puertos serie están en `platformio.ini`: `COMx` (master) y `COMx` (slave). **Ajustar**
  `monitor_port`/`upload_port` al entorno real antes de flashear. ⚠️ Los CP210x **se reenumeran**
  (pueden intercambiar COM entre placas); identificar cada placa por su AP (`FWM AP 1`=líder,
  `FWM AP 2`=seguidor) y usar `--upload-port` explícito.
- `monitor_speed = 57600`. ⚠️ Estas placas (TTGO LoRa32 V1.0) llevan cristal de **26 MHz**, así que
  el firmware **debe compilarse con `-DF_XTAL_MHZ=26`** (ya está en `platformio.ini`). Ese define
  hace que el core de Arduino fije el cristal y reconfigue los relojes en `app_main()`; sin él el
  core asume 40 MHz, la UART sale ×0.65 (~37440) y **la WiFi/BT queda fuera de banda** (no emite AP
  ni escanea redes). Con la flag, baud correcto (57600) y WiFi operativa. El LoRa no se ve afectado
  (SX1276 con cristal propio). No confundir con el `monitor_speed`: con el cristal corregido, 57600.
- ⚠️ **En Windows, flashear requiere UTF-8**: sin `PYTHONIOENCODING=utf-8` PlatformIO crashea con
  `UnicodeEncodeError` (cp1252) y la subida queda colgada. Usar:
  `$env:PYTHONIOENCODING='utf-8'; pio run -e ... -t upload`.
- **Compatibilidad de entorno:** se compila con **Arduino core 3.x / ESP-IDF 5**. Implicaciones:
  - Las librerías async son los forks mantenidos `esp32async/ESPAsyncWebServer` y
    `esp32async/AsyncTCP`. Las originales `me-no-dev/*` **fallan al enlazar** con core 3.x
    (`undefined reference to pxCurrentTCB`).
  - El watchdog usa la API de IDF ≥ 5 (`esp_task_wdt_config_t` + `esp_task_wdt_reconfigure`) y
    conserva la ruta IDF < 5 bajo `ESP_IDF_VERSION_MAJOR`.
  - `Web.h` **no** debe incluir `<WebServer.h>`: el proyecto usa `AsyncWebServer`, y ese header de
    Arduino rompe con el `#undef F` que hace `config.h` para MAVLink.
  - `config.h` incluye `<string>` (lo usa `FlightModeInfo`).
  - **Partición:** `board_build.partitions = huge_app.csv` (app **3 MB** + SPIFFS 896 KB). El
    firmware (~1.13 MB) usa ~**36 %**. Con la partición por defecto (`default.csv`, app 1.25 MB)
    llegaría al ~86 %; **no la cambies** sin revisar el tamaño con `pio run`.
  - Verificado: `pio run` de `master` y `slave` → **SUCCESS** (RAM 15.0 %, Flash 35.9 %).
  - **Menú OLED desactivado** (`USE_INTERACTIVE_MENU 0`): sus pines (12/13/14/15) chocan con el
    UART MAVLink (12/13), LoRa RST (14) y OLED SCL (15). No reactivar sin reasignar a GPIOs libres.
  - **Compresión LoRa**: `Comm::compressPacket` debe usar `floor()` para lat/lon. Con truncado
    hacia cero, las **longitudes negativas** (hemisferio oeste, p. ej. España) se corrompían ~100 km.
  - **LoRa**: los setters (`setSignalBandwidth`, etc.) van **después** de `LoRa.begin()`; antes,
    `setLdoFlag()` divide por cero (registros sin inicializar).

### Tests

```bash
pio test
```

- **Pruebas con FC simulado (SITL):** entornos `ttgo-lora32-v1-master-sitl` / `-slave-sitl`
  (`FC_LINK_USB=1`) + `tools/sitl_bridge.py` (puente serie↔TCP). Procedimiento completo en
  `docs/ROADMAP.md` §3.
- **Arnés HIL:** `tools/hil_test.py` valida el enlace líder/seguidor por serial (`FC_EMULATION=1`).
- **Laboratorio (CLI):** `python tools/lab.py` — menú interactivo que levanta **2 SITL** (ArduPlane)
  y un **MAVProxy headless** que reúne ambos vehículos en un único **UDP** para que Mission Planner
  los vea con una sola conexión. Con `--firmware` conecta además las placas (puentes serie). Acciones:
  despegue, GUIDED y benchmarks (recta/giro) con **reporte** en `tools/reports/`; la opción **9** lanza
  el **bench completo** (`tools/bench_suite.py`: link, params, formations, takeoff, straight, turn,
  safety + CSV por escenario).
  - `tools/sitl_bridge.py` une SITL↔placa y, con `--tap-port N`, da un **enlace MAVLink directo a la
    placa** por el mismo USB: el bench lee/escribe los parámetros FWM (componente 158) y cambia la
    formación **sin WiFi**. El `lab.py` reinicia las placas (DTR/RTS) al levantar el banco.
  - `tools/mp_launch.py` arranca MAVProxy **sin wxPython** en Windows (`has_wxpython=False` + `rline`
    dummy) y aplica un **parche a pymavlink** (las salidas UDP bindean el puerto destino y chocan con
    Mission Planner en el mismo PC → se cambia a bind efímero).

> ⚠️ **Salvedad:** `platformio.ini` **no define un `[env:native]`**, y `test/test_main.cpp`
> duplica sus propias funciones auxiliares (no enlaza contra `src/`). El runner documentado en
> `FASE4_IMPLEMENTADA.md` puede no ejecutarse tal cual. Antes de confiar en `pio test`, verificar/
> crear el entorno `native` y/o unificar los helpers con el código real.

---

## 3. Estructura del repositorio

```
src/
  main.cpp     Punto de entrada (setup/loop). Instancia global `FWM fwm`.
  FWM.h/.cpp   Orquestador: watchdog, máquina de estados, parámetros, ticker de envío.
  Comm.h/.cpp  LoRa: envío/recepción, checksum, compresión, retry, auto-calibración.
  Telem.h/.cpp MAVLink con el autopiloto: heartbeat, waypoints, distancia, predicción/formación.
  Screen.h/.cpp Display OLED + menú interactivo por botones.
  Web.h/.cpp   Servidor web async + API REST + WebSocket de telemetría.
  config.h     TODA la configuración, constantes, structs, enums y la clase Logger.
lib/mavlink/   Cabeceras MAVLink generadas (v2). NO editar a mano.
test/          test_main.cpp (Unity) — ver salvedad de arriba.
include/ lib/  Carpetas estándar de PlatformIO (README).
```

Documentación: `README.md` (raíz), `AGENTS.md` (raíz) y `docs/FLYWITHME.md` (documentación de
desarrollo consolidada: roadmap + Fases 1–4). Ver sección 7.

---

## 4. Flujo de datos

```
[LÍDER]  autopiloto --MAVLink--> Telem --(LoraPacket)--> Comm --LoRa--> aire
[SEGUIDOR] aire --LoRa--> Comm --(validar/descomprimir)-->
           Telem --(predicción + formación + filtro)--> nav_waypoint --> autopiloto
```

- El **líder** emite paquetes con un `Ticker` periódico (`send_packet_ticker`).
- El **seguidor** solo sigue si está en `FOLL_MODE_FOLLOWER` y el autopiloto está en `MODE_GUIDED`.
- La máquina de estados (`FWM::transitionState`) se sincroniza con `stage_follow` y el beacon.

---

## 5. Convenciones de código

- **C++17** (`-std=c++17`), estilo Arduino.
- Clases con acceso a `FWM*` y, cuando aplica, `static ClassName* self` + callback estático
  de `Ticker` (p. ej. `FWM::send_packet_ticker_callback`).
- **Naming mixto heredado**: métodos y funciones en `snake_case` (`nav_waypoint`, `status_text`,
  `sendPacket`), miembros en `camelCase`/`snake_case` (`commData`, `follow_mode`, `stage_follow`).
  Mantener el estilo del archivo que se edita.
- **Feature flags** vía `#define` en `config.h` (p. ej. `USE_COMPRESSED_PACKETS`, `ADAPTIVE_RATE`,
  `USE_PREDICTION`, `USE_POSITION_FILTER`, `USE_WEB_SERVER`, `USE_INTERACTIVE_MENU`,
  `SIMULATION_MODE`). Usar `#if` para activar/desactivar bloques.
- **Compilación condicional por variante**: `#ifdef MASTER_BUILD_FLAG` / `#ifdef SLAVE_BUILD_FLAG`
  definen `SYSID`, SSID y `FOLL_MODE`. No romper esta separación.
- **Logging**:
  - `Log.*` (ArduinoLog) → consola serie (requiere `DEBUG_MODE`).
  - `logger->info/debug/warning/...` → log persistente CSV en SPIFFS (`/flight.log`).
  - Niveles propios `FWM_LOG_LEVEL_*` (prefijo `FWM_` para no chocar con ArduinoLog).
  - ⚠️ **ArduinoLog no soporta `%f`**: no usar `%f`/`%.2f` en `Log.*` (imprime basura tipo `2f`).
    Formatear como entero escalado, p. ej. `Log.notice("d=%dm", (int)distance)`. `snprintf` de C
    **sí** soporta floats.
- **Errores**: preferir `isSafeToFollow()` y `validatePacket()` antes de aplicar un waypoint.
  Nunca seguir con datos GPS inválidos.

---

## 6. Configuración clave (`src/config.h`)

| Define | Valor por defecto | Descripción |
|---|---|---|
| `LORA_BAND` | `866E6` | Frecuencia (Europa). 433E6 Asia / 915E6 Norteamérica. |
| `LORA_SPREADING_FACTOR` | `12` | SF LoRa. |
| `LORA_TX_POWER` | `20` | dBm. |
| `F_XTAL_MHZ` (build flag) | `26` | Cristal de la TTGO LoRa32 V1.0. **Imprescindible** (`-DF_XTAL_MHZ=26`): sin él el core asume 40 MHz → UART ×0.65 y **WiFi/BT muertas**. |
| `USE_COMPRESSED_PACKETS` | `1` | Paquete comprimido (15 B) vs normal (27 B). |
| `ADAPTIVE_RATE` | `1` | Tasa de TX según distancia (2 s / 1 s / 0.5 s). |
| `MAX_LORA_RETRIES` | `3` | Reintentos con backoff en el envío. |
| `USE_PREDICTION` / `PREDICTION_TIME_MS` | `1` / `1000` | Predicción de posición del líder. |
| `USE_POSITION_FILTER` / `POSITION_FILTER_ALPHA` | `1` / `0.7` | Filtro paso bajo de posición. |
| `DEFAULT_FORMATION` | `0` (TRAIL) | 0=TRAIL, 1=LEFT, 2=RIGHT, 3=ABOVE, 4=BELOW. |
| `USE_HEADING_GUIDANCE` | `1` | 1 = guiado por rumbo cross-track (`GUIDED_CHANGE_*`); 0 = `DO_REPOSITION` (carrot). |
| `CROSS_TRACK_GAIN_DEG_PER_M` / `MAX_HEADING_CORR_DEG` | `0.5` / `25` | Corrección de rumbo por error lateral (grados por metro / tope). |
| `ALONG_GAIN_CMS_PER_M` / `MAX_SPEED_SLOW` | `12` / `400` | Corrección de velocidad por error longitudinal (cm/s por m / frenado máximo). |
| `GUIDED_ALT_REFRESH_MS` | `2000` | Cada cuánto se envía `DO_REPOSITION` para fijar la altitud (`next_WP_loc`). |
| `GUIDED_AIRSPEED_MIN` / `GUIDED_AIRSPEED_MAX` | `10` / `30` | Topes de la airspeed comandada (m/s). |
| `MAX_FOLLOW_DISTANCE` | `5000` | m — límite de seguridad. |
| `HEAD_ON_GUARD` | `0` | 1 = **guarda de colisión frente a frente**: si el líder viene de cara y < `HEAD_ON_RANGE` (500 m), rompe perpendicular a la visual y frena. Validado con `tools/follow_sim.py`. |
| `TIGHT_FORMATION` | `0` | 1 = **tasa LoRa rápida cuando cerca** (200/500/1000 ms) para vuelo a 10–20 m. Emparejar con SF bajo (`-D LORA_SPREADING_FACTOR=7`). |
| `MIN_SAFE_ALTITUDE` | `50000` | mm (50 m) — altitud mínima. |
| `AUTO_CALIBRATE_LORA` | `0` | Auto-calibración LoRa al inicio. |
| `USE_INTERACTIVE_MENU` | `0` | Menú OLED por botones. **Desactivado** por conflicto de pines. |
| `USE_WEB_SERVER` / `USE_WEBSOCKET` | `1` / `1` | Servidor async (80) + WS (81). |
| `WEB_TELEMETRY_WS` | `0` | Telemetría en vivo por WebSocket (retirada del flujo normal; solo diagnóstico en tierra). |
| `MAVLINK_PARAM_SERVER` | `1` | El ESP32 responde a `PARAM_REQUEST_LIST/READ/SET` como componente propio (`SYSID`,`COMPID=158`) → parámetros FWM visibles/editables en Mission Planner. `PARAM_SET` solo en tierra. |
| `WEB_AP_GROUND_ONLY` | `1` | El AP/WiFi solo se levanta **en tierra** (o sin FC); se apaga al armar/moverse. |
| `WEB_AP_FORCE` | `0` | 1 = forzar el AP siempre (banco; ignora la detección de tierra). |
| `WEB_AP_GS_MAX_CMS` / `WEB_AP_ALT_MAX_MM` | `200` / `3000` | Umbrales de "en vuelo": velocidad (cm/s) y altitud (mm) por encima de los cuales no es tierra. |
| `SIMULATION_MODE` | `0` | Simulación sin hardware en el **seguidor** (genera paquetes locales, no usa LoRa). |
| `FC_EMULATION` | `1` (banco) | Emula el FC: sintetiza telemetría en `APdata` sin UART1, **mantiene el LoRa real**. Poner `0` para vuelo con FC. |
| `FC_LINK_USB` | `0` | 1 = MAVLink por USB (UART0) en lugar de UART1 → pruebas con **SITL** (entornos `*-sitl`). En producción `0`. |
| `WEB_START_AP_IMMEDIATELY` | `1` | Inicia el AP WiFi al arrancar. |

---

## 7. Estado de la documentación (`*.md`)

| Archivo | Tipo | Estado |
|---|---|---|
| `README.md` | Presentación | ✅ Completo: descripción, hardware, arquitectura, build, config, tests y licencia. |
| `AGENTS.md` | Guía para agentes | Este documento. |
| `docs/FLYWITHME.md` | Documentación de desarrollo | **Consolidado**: roadmap (`MEJORAS_RECOMENDADAS`) + informes Fases 1–4. Alineado con el código. |
| `docs/ROADMAP.md` | Estado y plan vivo | Validación en banco, fallos corregidos y fases A–E. **Actualizar al completar tareas.** |
| `tools/hil_test.py` | Arnés de pruebas HIL | Valida el enlace líder/seguidor por serial (ver §2). |
| `tools/lab.py` | Laboratorio (CLI) | Menú: 2 SITL + MAVProxy headless → UDP para Mission Planner; acciones y reportes. |
| `tools/mp_launch.py` | Lanzador MAVProxy headless | Sin wxPython en Windows; aplica el parche UDP de pymavlink. |
| `tools/sitl_bridge.py` | Puente SITL ↔ placa | Pipe serie↔TCP; `--tap-port N` expone un enlace **MAVLink directo a la placa** (config del periférico **sin WiFi**). |
| `tools/bench_suite.py` | Bench completo | Escenarios (link/params/formations/takeoff/straight/turn/safety) + reporte MD/JSON + **CSV** por escenario. |
| `tools/follow_sim.py` | Simulador de la ley (sin hardware) | Replica la ley cross-track + modelo de avión y **latencia del enlace**; escenarios trail/head_on/lateral y barridos de distancia/latencia. |

> Los antiguos `MEJORAS_RECOMENDADAS.md` y `FASE{1..4}_IMPLEMENTADA.md` se fusionaron en
> `docs/FLYWITHME.md` y se eliminaron de la raíz.

### Notas de coherencia docs ↔ código

Los documentos de fase se revisaron y alinearon con el código (el código sigue siendo la fuente
de verdad). Puntos a tener presentes:

- **Entorno `native`:** `platformio.ini` solo define `ttgo-lora32-v1-master` y
  `ttgo-lora32-v1-slave`. No hay `[env:native]`, así que `pio test` no corre tal cual; los docs
  ya lo indican como pendiente.
- **Tests:** `test/test_main.cpp` contiene **13** pruebas (`RUN_TEST`) y duplica sus funciones
  auxiliares en vez de enlazar con `src/`. Si se unifican, actualizar el conteo en los docs.

### Estado de git

- `src/*`, `platformio.ini`, `README.md` y `.vscode/` están **modificados sin commitear**;
  `test/test_main.cpp`, `AGENTS.md` y `docs/FLYWITHME.md` están **sin trackear** (`??`).
  No asumir que estos cambios están en el historial.

---

## 8. Reglas para agentes (importante)

1. **No editar** nada bajo `lib/mavlink/` (código generado).
2. **Ajustar los puertos COM** de `platformio.ini` al hardware real; no hardcodear otros.
3. **Compatibilidad de protocolo LoRa:** si cambias `LoraPacket_t`/`CompressedLoraPacket_t`,
   mantén la recepción de ambos formatos (backward compatibility entre versiones de firmware).
4. **Seguridad primero:** cualquier cambio en seguimiento debe respetar `isSafeToFollow()` y los
   límites de `config.h`. Nunca elimines validaciones sin justificarlo.
5. **Feature flags:** añade nuevas capacidades detrás de un `#define` en `config.h` con valor por
   defecto conservador, y documéntalo en la tabla de la sección 6.
6. **`config.h` es central:** la mayoría de structs, enums y la clase `Logger` viven ahí.
   Evita duplicar definiciones.
7. **Al cambiar comportamiento, actualiza el `.md` correspondiente** (y corrige las discrepancias
   de la sección 7). Mantén el `README.md` cuando se rellene.
8. **Antes de proponer un commit**, compila ambas variantes (`pio run -e ...-master` y `-slave`).
9. Este repositorio **no usaba** instrucciones de agente previas; este `AGENTS.md` es el punto
   de referencia.

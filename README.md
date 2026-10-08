# FlyWithMe

**The way to fly with your friends.**

Sistema de **vuelo en formación** para aviones con autopiloto **ArduPlane**. Un avión
**líder** transmite su posición por radio **LoRa**; un avión **seguidor** recibe esos
datos, calcula una posición de formación y envía las órdenes a su autopiloto vía
**MAVLink** (modo GUIDED). Está pensado para hardware de bajo coste basado en ESP32. PX4 no está
validado.

---

## Características

- 📡 **LoRa protocolo v2**: trama de 40 B con versión, BEACON/JOIN/REPLY, secuencia, posición válida,
  modo del líder y `netid`; el `netid` filtra sistemas distintos (no cifra).
- 🔁 **Sesión bidireccional por slots**: el líder usa discovery lento sin seguidor y abre una ventana
  de respuesta; con sesión activa vuelve a la cadencia configurada.
- 🛰️ **Modos** líder, seguidor, bridge MAVLink y off.
- 🧭 **Predicción acotada**: en TRAIL el predictor no puede cancelar el offset ni llevar el objetivo
  hasta el líder cuando se configura una separación corta.
- 🔁 **Reintentos con backoff** y **tasa de transmisión adaptativa** según distancia.
- 🛡️ **Seguridad**: watchdog, validación de datos GPS y límites de seguimiento.
- 🤖 **Máquina de estados** (INIT, SEARCHING, CONNECTING, FOLLOWING, LOST_LINK, EMERGENCY, LANDING).
- 🎯 **Predicción de movimiento**, **formaciones dinámicas** (trail/left/right/above/below) y
  **filtro de posición** paso bajo.
- 🖥️ **WebUI** de configuración en tierra (EN/ES), API REST y parámetros FWM por MAVLink.
- 📣 **STATUSTEXT** de distancia líder/seguidor con deduplicación; la captura de Mission Planner
  confirma el overlay HUD y la pestaña Messages.
- 🖥️ Display OLED SSD1306 para estado; menú por botones desactivado por conflicto de pines.
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
| Autopiloto | **ArduPlane** vía UART1 (57600 baudios; ESP32 RX=GPIO12, TX=GPIO13) |
| UART del autopiloto | Cruzada: FC TX → ESP32 GPIO12; ESP32 GPIO13 → FC RX; GND común |

> ⚠️ `USE_INTERACTIVE_MENU=0`: sus GPIO12/13/14/15 chocan con UART1, reset LoRa y OLED. No
> conectes botones de menú ni lo actives sin reasignar pines.

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

## 1. Compilar y flashear

Requiere [PlatformIO](https://platformio.org/) y el entorno Python de PlatformIO.

### Elige el entorno correcto

| Entorno | Uso | FC |
|---|---|---|
| `ttgo-lora32-v1-master-flight` | **Líder, vuelo real** | FC por UART1; `FC_EMULATION=0` |
| `ttgo-lora32-v1-slave-flight` | **Seguidor, vuelo real** | FC por UART1; `FC_EMULATION=0` |
| `ttgo-lora32-v1-master-sitl` | Líder en banco SITL | USB/UART0 hacia ArduPlane SITL; SF7/BW250k/5Hz |
| `ttgo-lora32-v1-slave-sitl` | Seguidor en banco SITL | USB/UART0 hacia ArduPlane SITL; SF7/BW250k/5Hz |
| `ttgo-lora32-v1-master` / `...-slave` | Emulación FC en banco sin autopiloto | **No usar para vuelo real** (`FC_EMULATION=1`) |

Los entornos `*-flight` son los recomendados para el avión real. Mantienen `FC_LINK_USB=0` y
desactivan la telemetría sintética. Los entornos `*-sitl` son solo para el banco.

### Identifica cada placa antes de subir

Los CP210x pueden **intercambiar/reenumerar COM** al desconectar o resetear. No asumas que COMx
siempre es líder o que COMx siempre es seguidor: identifica cada placa por el SYSID del FC o por el
SSID de tierra (`FWM AP 1` = líder, `FWM AP 2` = seguidor). Ajusta `upload_port` y `monitor_port` o
usa siempre `--upload-port` explícito.

En Windows, PlatformIO necesita UTF-8 para que esptool no falle al imprimir mensajes:

```powershell
$env:PYTHONIOENCODING = 'utf-8'
pio run -e ttgo-lora32-v1-master-flight
pio run -e ttgo-lora32-v1-slave-flight
```

Con los puertos ya identificados, flashea uno por uno:

```powershell
$env:PYTHONIOENCODING = 'utf-8'
pio run -e ttgo-lora32-v1-master-flight -t upload --upload-port COMx
pio run -e ttgo-lora32-v1-slave-flight  -t upload --upload-port COMy
```

Sustituye `COMx`/`COMy` por los puertos actuales del **líder/seguidor**, respectivamente. No flashees
si otro monitor, bridge o PlatformIO mantiene el puerto abierto. Monitor serie, a 57600 baudios:

```powershell
pio device monitor --port COMx --baud 57600
```

Al arrancar debe aparecer `SELFTEST PASS` (layout/checksum/secuencia/deduplicación). Comprueba ambos
firmwares antes de conectar los UART de vuelo.

---

## Configuración

Casi todo se configura con `#define` en [`src/config.h`](src/config.h); los parámetros con NVS también
se pueden cambiar en la WebUI/Mission Planner **en tierra**. Valores iniciales de firmware:

| Define | Por defecto | Descripción |
|---|---|---|
| `LORA_BAND` | `866E6` | Frecuencia (Europa). `433E6` Asia / `915E6` Norteamérica. |
| `LORA_SIGNAL_BANDWIDTH` | `125000` | Ancho de banda (Hz). Los profiles SITL usan BW250k; no asumir la misma latencia en vuelo real. |
| `LORA_SPREADING_FACTOR` | `12` | Spreading factor LoRa; más robusto y lento al subir SF. |
| `LORA_TX_POWER` | `20` | Potencia de transmisión (dBm). |
| `USE_COMPRESSED_PACKETS` | `0` | Protocolo v2 usa trama wire packed de 40 B. No activar el compresor antiguo. |
| `ADAPTIVE_RATE` | `1` | Tasa TX según distancia (cerca 2 s, media 1 s, lejos 0,5 s). |
| `USE_PREDICTION` / `PREDICTION_TIME_MS` | `1` / `1000` | Predicción del líder; en TRAIL limitada al 25 % del offset para evitar cancelarlo. |
| `USE_POSITION_FILTER` / `POSITION_FILTER_ALPHA` | `1` / `0.3` | Filtro paso bajo del objetivo. |
| `DEFAULT_FORMATION` | `0` | `0`=TRAIL, `1`=LEFT, `2`=RIGHT, `3`=ABOVE, `4`=BELOW. |
| `DIST_OFFSET` / `dist_offset` NVS | `96 m` | Separación TRAIL inicial. El valor NVS/WebUI prevalece sobre el define. |
| `netid` NVS/WebUI | `4660` (`0x1234`) | ID común del protocolo; debe coincidir en líder y seguidor. Filtra otros sistemas a nivel de paquete, **no cifra**. |
| `approach_dist` | `300 m` | Cerca de esta distancia exige modo estable del líder. Configurable en tierra. |
| `MAX_FOLLOW_DISTANCE` | `5000 m` | Límite máximo de seguimiento. |
| `MIN_SAFE_ALTITUDE` | `50000 mm` | No sigue si líder o seguidor están por debajo de50m AGL. |
| `FWM_DUAL_CORE` / `FOLLOWER_REPLY` | `1` / `1` | Core1 propietario del vuelo/radio; replies JOIN/REPLY en slot half-duplex. |
| `LORA_CRC` / `NETID_USE_SYNCWORD` | `0` / `0` | CRC RF/sync derivado experimentalmente desactivados; el default usa sync fijo y checksum del protocolo. |
| `TIGHT_FORMATION` | `0` | Opt-in: más beacons cerca. Requiere SF/bandwidth adecuados; probar en SITL antes de usarlo. |
| `USE_INTERACTIVE_MENU` | `0` | Menú por botones desactivado por conflicto de pines. |
| `USE_WEB_SERVER` / `USE_WEBSOCKET` | `1` / `1` | Servidor web (80) + WebSocket (81). |
| `WEB_AP_GROUND_ONLY` | `1` | AP Web solo en tierra; no configurar ni depender de WiFi en vuelo. |
| `MAVLINK_PARAM_SERVER` / `FWM_SELFTEST` | `1` / `1` | Parámetros FWM por componente158 y auto-test de protocolo al arrancar. |
| `STATUSTEXT` | INFO(6), dedupe exacto | `FWM: Follower Nm`; texto idéntico no se reenvía. La captura de MP mostró el overlay HUD y Messages. |
| `HEAD_ON_GUARD` | `0` | Experimental y no validada para vuelo real; evitar aproximaciones frontales. |

Los builds fijan los roles: líder `SYSID=1`, seguidor `SYSID=2`, ambos con `COMPID=158` para FWM.
El `netid` se guarda por placa en NVS: cambiarlo en una exige cambiarlo también en la otra.

### Acceso web

En tierra, conéctate al AP de la placa (`FWM AP 1` líder / `FWM AP 2` seguidor) y abre
`http://192.168.4.1`. La WebUI permite configurar `netid`, formación, offsets y `approach_dist`;
aplica persistencia NVS. El AP se apaga en vuelo. No edites parámetros al estar armado.

---

## 2. Configurar y preparar un vuelo

### A. Configura primero los ArduPlane

1. Usa **ArduPlane** en ambos aviones y asigna `SYSID_THISMAV=1` al líder y `=2` al seguidor.
   Deben coincidir con los firmwares `master-flight` y `slave-flight`.
2. Conecta UART1 de cada TTGO a un puerto TELEM libre del FC, cruzando TX/RX y compartiendo GND.
   En el `SERIALx` físicamente conectado configura MAVLink2 (`SERIALx_PROTOCOL=2`) y 57600
   (`SERIALx_BAUD=57`); identifica el índice `x` según el puerto TELEM usado y reinicia el FC.
3. Confirma primero el enlace FC↔ESP32 en tierra: el monitor a57600 debe mostrar el autotest y
   `Connected`/heartbeat, sin `No FC connection` persistente.

### B. Flashea y valida cada rol

Sigue §1 usando los entornos `*-flight`; identifica los puertos COM actuales, no copies el COM de
otro día. Tras flashear, confirma `SELFTEST PASS` en las dos placas y comprueba físicamente antenas,
alimentación, UART cruzada y SYSID de cada avión.

### C. Ajustes de tierra

1. Con ambos aviones desarmados, abre cada AP/WebUI y fija el **mismo `netid`** en ambos (por defecto
   `4660`/`0x1234`). Confirma el readback en los dos. El ID filtra sistemas ajenos; no es cifrado.
2. Empieza en formación TRAIL y offset conservador (el default inicial es96m). Ajusta también
   `approach_dist` (300m inicial) y deja `foll_enable=1`.
3. No empieces un vuelo real con `dist_offset=5/10/20m`: esos offsets solo se han probado en SITL
   con enlace SF7/BW250k/5Hz. En el build de vuelo normal, SF12 y la tasa cerca más lenta tienen otra
   latencia; aproxima en etapas y reduce el offset solo después de medir el enlace real.
4. El test `netid` cambia temporalmente NVS: solo ejecutarlo en tierra con `--netid-test`; el bench
   lo omite por defecto y restaura el ID con `finally`.

### D. Secuencia de vuelo inicial

1. Arranca ambos FC y FWM, espera fix GPS y confirma enlace LoRa (RX crece, RSSI/SNR razonables,
   `BEACON LOCK`/`FOLLOWING`). La telemetría FWM del líder anuncia al seguidor en STATUSTEXT; el
   texto idéntico no se repite hasta que cambie.
2. Despega con piloto/autopiloto supervisando; supera **50m AGL** (límite del firmware). El líder
   debe estar en `FBWA`, `FBWB`, `CRUISE`, `AUTO`, `RTL`, `LOITER`, `TAKEOFF` o `GUIDED` cuando el
   seguidor se acerque a menos de `approach_dist`.
3. Pon el seguidor en **GUIDED** para habilitar el seguimiento. A más de `approach_dist` puede acudir
   a buscar al líder; al acercarse, el gate suspende nuevas órdenes si el modo del líder no es estable.
   **No es un hold/loiter automático:** el autopiloto conserva la última consigna. Por eso el líder debe
   estar ya en un modo estable antes de entrar en la distancia de aproximación.
4. Mantén piloto listo para tomar control, prueba primero recta y giro amplio, y monitoriza separación,
   altitud, RSSI/SNR y `LOST_LINK` en el OSD/telemetría.
5. **No iniciar en frente-a-frente ni hacer inversión brusca cerca.** `HEAD_ON_GUARD` sigue opt-in y
   su escenario es solo SITL; no está aprobado como sistema anticolisión para vuelo real. En close
   formation, no iniciar el seguimiento si los aviones convergen de frente.

> El bench de campo simulado usa **200m** para librar el relieve al norte del home y rutas al sur.
> Las pruebas SITL no certifican el comportamiento con RF, viento, GPS ni FC reales. El desarrollo
> aún está en validación: realizar pruebas incrementales en zona amplia, con observador y failsafe
> independiente; no usar este firmware como único sistema anticolisión.

---

## Tests

```bash
# Builds para los dos roles de vuelo real
pio run -e ttgo-lora32-v1-master-flight
pio run -e ttgo-lora32-v1-slave-flight

# Simuladores de regresión, no requieren placas
python tools/proto_sim.py
python tools/follow_sim_test.py

# Banco SITL interactivo (dos ArduPlane + MAVProxy + Mission Planner)
python tools/lab.py --firmware --leader-com COMx --slave-com COMx

# Tests unitarios Unity (nota: falta [env:native])
pio test
```

En el menú de `lab.py`, primero levanta el banco (opción 1) y luego ejecuta el bench completo (opción9).
El bench guarda Markdown/JSON y CSV en `tools/reports/`. El test `netid` es opt-in (`--netid-test`)
porque cambia temporalmente NVS; el escenario head-on, que puede acercar los aviones a una colisión,
requiere `--head-on-test` y es **solo SITL**, nunca usarlo como autorización para un vuelo real.

`tools/proto_sim.py` cubre discovery/sesión, pérdidas, blackout y recuperación, netid y gate de modo.
`tools/follow_sim_test.py` comprueba predictor/offsets cortos, cruce y la guarda head-on. `src/selftest.h`
ejecuta al arranque pruebas de trama, checksum, secuencia y deduplicación de STATUSTEXT.

Si `python` no tiene `pymavlink`/dependencias del lab en Windows, usa el Python de PlatformIO:

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py tools\proto_sim.py
& $py tools\follow_sim_test.py
```

> **Limitación actual:** `platformio.ini` no define un entorno `[env:native]` y el archivo de
> tests incluye sus propias funciones auxiliares (no enlaza contra `src/`). Para ejecutarlos de
> forma nativa hay que añadir ese entorno y unificar los helpers con el código real.

---

## Documentación

| Documento | Contenido |
|---|---|
| [`docs/FLYWITHME.md`](docs/FLYWITHME.md) | Documentación de desarrollo consolidada: roadmap y Fases 1–4. |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | **Estado y roadmap vivo** (validación, fallos corregidos, fases A–E). |
| [`AGENTS.md`](AGENTS.md) | Guía para agentes de código / desarrolladores. |

---

## Seguridad

Este proyecto controla aeronaves y todavía está en desarrollo. Los resultados SITL **no certifican**
RF, GPS, viento, autopiloto ni evasión de terreno reales. Realiza pruebas progresivas (banco → campo
sin vuelo → vuelo), en zona amplia, con piloto listo para tomar control y failsafe independiente.
No uses FlyWithMe como único sistema anticolisión. Evita encuentros frente-a-frente y no vueles a
5–20m de separación hasta validar esa configuración en hardware real y condiciones controladas.

---

## Licencia

Distribuido bajo la **GNU General Public License v3.0**. Ver [`LICENSE`](LICENSE).

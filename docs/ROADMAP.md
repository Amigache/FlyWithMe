# FlyWithMe — Estado y roadmap

Documento **vivo** de seguimiento. Registra el estado de validación, los fallos corregidos y el
plan de trabajo por fases. Actualizar al completar cada tarea.

> Última actualización: suite completa del bench; 20 PASS, 0 FAIL, 0 SKIP en ArduPlane SITL; cancelación ordenada limpia y restaura las placas.

---

## 1. Estado actual

| Área | Estado |
|---|---|
| Compilación (perfil común `ttgo-lora32-v1-flight` y SITL) | ✅ SUCCESS |
| SITL runtime universal | ✅ flight/dev compilan; COMx/COMx aceptan `SIMCFG OK`; reporte `20261008-211545` valida suite HIL completa y Stop/cancel limpian y restauran UART1; solo ArduPlane SITL, no vuelo físico |
| Dependencias MAVProxy / puente TAP | ✅ `prompt-toolkit`; bridge abre COM sin reset, TAP de ambos FWM y diagnóstico RX/TX/enlace por MAVLink |
| SSID/identidad por placa | ✅ Flasheado y verificado físicamente en COMx (líder) y COMx (seguidor): SSID y BSSID coinciden con MAC SoftAP; `FWM ID` devuelve identidad correcta en ambos |
| Flasheo/provisión de rol | ✅ `tools/flash_firmware.py`; misma imagen, rol individual en NVS |
| Arranque | ✅ sin crashes ni errores I2C |
| Enlace LoRa líder → seguidor | ✅ **verificado** (`BEACON LOCK`, `FOLLOWING`, `Formation position`) |
| Integración con FC real | ⏳ pendiente; HIL actual usa ArduPlane SITL por USB, no un autopiloto físico |
| Arnés de pruebas HIL | ✅ `tools/hil_test.py` |

## 2. Fallos corregidos (histórico)

| # | Fallo | Causa | Solución |
|---|---|---|---|
| 1 | La subida se colgaba | PlatformIO crasheaba con `UnicodeEncodeError` (cp1252) al imprimir esptool | Ejecutar `pio` con `PYTHONIOENCODING=utf-8` |
| 2 | Crash al arrancar (IntegerDivideByZero) | `LoRa.setSignalBandwidth()` antes de `LoRa.begin()`; `setLdoFlag()` divide por cero | Setters **después** de `begin()` |
| 3 | GPS basura sin FC | `APdata` sin inicializar | `memset` en constructor de `Telem` (A1) |
| 4 | Diluvio de errores I2C | Botones del menú (12/13/14/15) chocan con UART/LoRa RST/OLED SCL | `USE_INTERACTIVE_MENU 0` |
| 5 | Baud equivocado y **WiFi muerta** | Cristal 26 MHz de la TTGO LoRa32 V1.0; el core asumía 40 MHz → UART ×0.65 y RF de WiFi/BT fuera de banda | Compilar con **`-DF_XTAL_MHZ=26`** (fija el cristal y reconfigura los relojes en `app_main()`) → UART 57600 y WiFi operativa |
| 6 | **El seguidor no enlazaba** (error ~100 km) | Compresión truncaba hacia cero: longitudes negativas mal reconstruidas | `floor()` en `compressPacket` |
| 7 | FSM bloqueada | `SEARCHING/LOST_LINK → FOLLOWING` no permitidas | Ampliadas transiciones válidas |
| 8 | Logs ilegibles (`2f`, `1fm`) | ArduinoLog no soporta `%f` | Formatear como enteros escalados |
| 9 | Aserción FreeRTOS al iniciar el servidor web con AP inhibido | `AsyncWebServer::begin()` se llamaba antes de inicializar WiFi/lwIP | Crear/iniciar AsyncWebServer solo desde `startAP()` después de `WiFi.softAP()` |
| 10 | El AP aparecía brevemente y desaparecía sin FC | `Web::run()` lo arrancaba por su cuenta y el gate lo apagaba al siguiente ciclo | `updateApGate()` es la única autoridad; sin enlace FC inicial, el timeout permite setup; tras un enlace previo, la pérdida mantiene el AP bloqueado. Reflasheado: COMx informa `Access Point Ready`, pero el escaneo local aún no ve `FWM C5C559` (pendiente comprobar alcance/otro cliente WiFi). |
| 11 | El bench SITL se detenía al iniciar MAVProxy | Faltaba la dependencia transitiva opcional `prompt-toolkit` en el entorno Python de desarrollo | Dependencia añadida; MAVProxy corrió en la suite HIL (`20261008-211545`). Verificación visual del GCS pendiente. |
| 12 | `role_setup` no detectaba heartbeat FWM por TAP | SIM se activaba >10 s antes de abrir bridges; sin heartbeat FC, el ticker expiraba durante el arranque de SITL | Arrancar bridges inmediatamente después de crear SITL; `role_setup` PASS con SYSID 1/2 y COMPID 158. Fallback NVS fuera de rango a `LINK_TIMEOUT` añadido como robustez. |
| 13 | El bridge podía resetear el ESP32 al abrir CP210x | DTR/RTS se cambiaban después de abrir el puerto | Configurarlos antes de `Serial.open()`; test unitario PASS |
| 14 | Bench medía logs de texto durante SIM, aunque el USB transporta MAVLink y el firmware silencia Log | Session/netid/head-on y mode gate no observaban RX ni eventos; generaba falsos FAIL | Añadir `NAMED_VALUE_INT` solo en runtime dev y STATUSTEXT para la guarda; suite completa PASS (`20261008-211545`). |
| 15 | El bench `head_on` requiere la guarda compilada | `HEAD_ON_GUARD` es 0 por defecto y el test exige eventos de ruptura | El perfil dev/HIL `ttgo-lora32-v1-sitl` habilita `HEAD_ON_GUARD=1`; flight de producción conserva 0. |
| 16 | CtrlBreak del bench podía omitir limpieza de SITL/placas | El proceso Python terminaba al recibir SIGBREAK sin ejecutar `finally` | `bench_suite` captura SIGBREAK y limpia; cancelación probada con exit 130, puertos liberados e identidades confirmadas. |

## 3. Entorno y notas de banco

- **Baud serie: 57600**. La placa lleva cristal de **26 MHz**; con `-DF_XTAL_MHZ=26` el core ajusta el
  reloj y el baud es correcto. Sin esa flag saldría ~37440 (×0.65) **y la WiFi/BT no funcionarían**.
- **Flashear/provisionar en Windows:** `python tools\flash_firmware.py --port COMx --role leader|follower`;
  carga la imagen común y persiste el rol sin compilar variantes.
- **Puertos observados históricamente:** `COMx` y `COMx`; no son identidad fija. **USB inestable** (cable/puerto).
- **`FC_EMULATION = 1`**: sintetiza telemetría en `APdata` sin UART1 y **mantiene el LoRa real**.
  Poner `0` para vuelo con FC.
- **`USE_INTERACTIVE_MENU = 0`** por conflicto de pines.

### Arnés HIL

```powershell
# Cargar la misma imagen de emulación y asignar roles en NVS
python tools\flash_firmware.py --environment ttgo-lora32-v1 --port COMx --role leader
python tools\flash_firmware.py --environment ttgo-lora32-v1 --port COMy --role follower

# Ejecutar el HIL con los roles ya provisionados
.\.platformio\penv\Scripts\python.exe tools\hil_test.py --master COMx --slave COMx --seconds 35
```

Comprueba: líder transmite, seguidor engancha beacon, entra en FOLLOWING, calcula formación y
ninguno entra en EMERGENCY. Salida `PASS`/`FAIL` (código 0/1).

### SITL (FC simulado) — MAVLink por USB

Prueba con autopilotos **simulados** (Mission Planner SITL) sin cablear el UART del FC. El proyecto
es para **aviones**, así que se usa `ArduPlane.exe` (no ArduCopter).

1. Lanzar los vehículos SITL (nativo Windows). Se usan los `plane.parm` del repo
   (`tools/sitl/leader.parm` y `tools/sitl/follower.parm`, copiados del de Mission Planner
   `sitl\models\plane.parm` + `SYSID_THISMAV`) y **`--sysid`** (el `SYSID_THISMAV` del parm NO se
   aplica por sí solo; sin él, el SITL arranca con sysid 1):
   ```
   ArduPlane.exe --instance 0 --serial0 tcp:5760 --sysid 1 --model plane --home 0.000000,0.000000,302,0 --defaults <repo>\tools\sitl\leader.parm
   ArduPlane.exe --instance 1 --serial0 tcp:5770 --sysid 2 --model plane --home 0.000000,0.000000,302,0 --defaults <repo>\tools\sitl\follower.parm
   ```
   Puertos por instancia: SERIAL0 `5760+10N` (ESP32), SERIAL1 `+2` (Mission Planner), SERIAL2 `+3` (control).
   Los `plane.parm` marcan el INS como calibrado (`INS_ACC*`, `INS_GYR_CAL 0`) → **arma sin calibrar**.
   ⚠️ Usar **`--model plane`**. El flag `-M+` de Mission Planner pone el modelo a `"+"` (inválido) y
   el avión **rebota en el suelo** (`SIM Hit ground`) sin despegar.
   El `yaw` del `--home` (4º campo) fija el rumbo de despegue: **180 = sur** (al norte hay montañas).
   `tools/sitl_start.ps1` ya lo hace por defecto.

   **Seguimiento (validado):** el firmware usa `MAV_CMD_DO_REPOSITION` (ArduPlane GUIDED ignora
   `NAV_WAYPOINT`). Los paquetes van **sin comprimir** (`USE_COMPRESSED_PACKETS 0`): el formato
   comprimido cuantiza lat/lon a ~1/255° (**~436 m**), lo que hacía el seguimiento errático.
2. Para el HIL runtime actual, flashear una sola vez ambas placas con el mismo perfil universal dev y
   activar SIM con `tools/lab.py --firmware` (`FWM SIM ON`); reset devuelve a UART1. La validación histórica
   anterior usaba `FC_LINK_USB=1` al arranque:
    ```
    pio run -e ttgo-lora32-v1-sitl -t upload --upload-port COMx
    ```
    El perfil dev no contiene el rol ni persiste el modo SIM. Asignar el rol en tierra; el bench confirma
    `SIMCFG OK` por USB antes de conectar los bridges.
3. Puente serie ↔ TCP (uno por placa):
   ```
   .\.platformio\penv\Scripts\python.exe tools\sitl_bridge.py --tcp 127.0.0.1:5760 --port COMx
   .\.platformio\penv\Scripts\python.exe tools\sitl_bridge.py --tcp 127.0.0.1:5770 --port COMx
   ```
4. Automatización (pymavlink): poner el seguidor en **GUIDED** y, opcional, despegar el líder:
   ```
   python tools\sitl_guided.py  --conn tcp:127.0.0.1:5773 --mode 15   # seguidor -> GUIDED
   python tools\sitl_takeoff.py --conn tcp:127.0.0.1:5763 --alt 120   # líder: GUIDED+arm+takeoff
   ```

En producción: `FC_LINK_USB=0` (MAVLink por UART1, GPIO12/13 → FC real).

#### Ver en Mission Planner en tiempo real
Mission Planner → **Connection: TCP** → host `127.0.0.1`:
- **Líder:** puerto **5762** · **Seguidor:** puerto **5772**.

Verás un **avión** (ArduPlane) con mapa/HUD/parámetros en vivo. (SERIAL0 lo ocupa el puente del
ESP32, por eso el GCS usa SERIAL1.)

> **Estado verificado (ArduPlane estable, ambas placas):** reciben MAVLink real por USB
> (`LINK TO FC OK`), enlazan por LoRa (`BEACON LOCK`), el seguidor pasa a **FOLLOWING** con el FC en
> GUIDED y **calcula la formación** (`Formation position: lat=..., lon=..., alt=...`).
>
> **Versión estable de SITL:** la local (`Documents\Mission Planner\sitl\ArduPlane.exe`) es
> `4.8.0-dev`. Versión **estable**: descargar de
> `https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/PlaneStable/` (`ArduPlane.elf` + DLLs
> cyg*) → es **4.7.2**.
>
> **Armado:** SITL lanzado "a pelo" pide calibración 3D de acelerómetros (Mission Planner lo evita
> porque pasa `--defaults`). `tools/sitl_takeoff.py` hace `param_set(ARMING_CHECK,0)` antes de armar.
>
> **Identidad:** el parámetro `role` determina el SYSID FWM/autopiloto esperado: líder=1, seguidor=2.
> Los builds de vuelo y SITL no fijan el rol en compile time; placa nueva queda `OFF` hasta provisionarla.
>
> **Formación en banco:** `FWM SIM ON` activa `MIN_SAFE_ALTITUDE=0`, SF7/BW250k y 200ms durante la
> sesión, para validar el avión SITL en el suelo. Reset y el firmware de producción mantienen 50 m/SF12.
>
> Nota: los puertos USB reenumeran (COMx→21→22); comprobar el puerto antes de cada prueba.

---

## 4. Roadmap

### Fase A — Robustez en banco (sin FC)
- [x] **A1** Defaults seguros de `APdata`/`commData` (evitar basura).
- [x] **A2** Recuperación de FSM con histéresis (`EMERGENCY → SEARCHING`; `LOST_LINK → FOLLOWING`).
- [x] **A3** Arnés HIL `tools/hil_test.py`.
- [x] **A4** Métricas de enlace (RSSI/SNR, pérdidas, distancia, estado) en log/OLED/web.
  - Log periódico `Link: ...` cada `LINK_METRICS_LOG_INTERVAL_MS` (5 s).
  - OLED `showStatsScreen()`: Uptime, RX/TX, Lost+%, RSSI/SNR, Distancia.
  - Web: `/api/stats` y WebSocket con `lost_packets`, `packet_loss`, `distance`, `state`; panel HTML con Pérdidas/Distancia/Estado.

### Fase B — Integración con FC real (`FC_EMULATION 0`)
- [x] **B5a** Enlace de pruebas con SITL por USB (históricamente `FC_LINK_USB=1`): **líder y seguidor verificados**
  (`LINK TO FC OK`, `BEACON LOCK`, `dist`). Añadidos `tools/sitl_bridge.py`; el rol/SYSID se determinan
  por NVS. El flujo runtime dev `FWM SIM ON` está implementado; HIL físico con esa ruta pendiente.
- [ ] **B5b** Verificar/ajustar el baud del UART1 con FC real (posible desfase por cristal).
- [ ] **B6** Validar MAVLink real: RX (HEARTBEAT/GLOBAL_POSITION_INT), `request_data_streams`,
  `nav_waypoint` en GUIDED, `do_change_speed`. **Requiere el FC en modo GUIDED** para seguir.
- [ ] **B7** Robustez de conexión: sysid/compid configurables, reconexión (hoy `check_link` hace
  `detach()` del ticker y no reintenta) y timeouts.

### Fase C — Algoritmo de seguimiento
- [x] **C8** Afinar predicción/filtro y offsets de formación.
- [x] **C9** Control de velocidad/altitud: **guiado por rumbo cross-track** (§4.1).
- [ ] **C10** Tasa adaptativa del líder (hoy usa 500 m fijos).

### Fase C.1 — Nuevo guiado por rumbo (2026-10, validado en SITL)

Se sustituye el reposicionamiento "carrot"/`DO_REPOSITION` por un **guiado por rumbo
cross-track** (estilo `plane_follow.lua`), que elimina el loiter y el zigzag:

- **Rumbo**: `MAV_CMD_GUIDED_CHANGE_HEADING` (43002, dentro de `COMMAND_INT`) = dirección de la
  traza del líder (de `vx/vy` del paquete) **+ corrección proporcional al error lateral**
  (`CROSS_TRACK_GAIN_DEG_PER_M`, tope `MAX_HEADING_CORR_DEG`).
- **Velocidad**: `MAV_CMD_GUIDED_CHANGE_SPEED` (43000). ⚠️ ArduPlane **solo admite airspeed**
  (`param1=0`); con groundspeed responde `DENIED`. Corrección por error longitudinal
  (`ALONG_GAIN_CMS_PER_M`).
- **Altitud**: cada `GUIDED_ALT_REFRESH_MS` se envía un `DO_REPOSITION` al punto de formación, que
  fija `next_WP_loc` (la altitud objetivo). `GUIDED_CHANGE_ALTITUDE` (43001) **no** funciona en
  Plane GUIDED (depende de `guided_state.target_location`, que nunca se inicializa). El **rumbo se
  reafirma cada ciclo y manda sobre la posición**.
- **Formación**: TRAIL/LEFT/RIGHT a la **misma altitud** que el líder; `ALT_OFFSET` solo en
  ABOVE/BELOW.

Validación end-to-end (firmware real + LoRa, SITL ArduPlane, `tools/hil_sitl_validate.py`):

| Prueba | Distancia líder-seguidor | `|roll|` en régimen | Resultado |
|---|---|---|---|
| Recta | 105–112 m (objetivo 100) | 0–3° | **PASS**, sin loiter |
| Giro 90° | 102–118 m | 0–11° (pico en el giro) | **PASS**, sin zigzag |

Tras afinar (`CROSS_TRACK_GAIN_DEG_PER_M=0.6`, `MAX_HEADING_CORR_DEG=40`, `DIST_OFFSET=92`):

| Formación | Separación real (objetivo) | Alabeo giro | Resultado |
|---|---|---|---|
| TRAIL | ~100 m (100) | ~11° | **PASS** |
| LEFT | ~54 m (50) | recupera tras el giro | **PASS** |
| ABOVE | +20 m alt, 5–15 m horiz | ~0° | **PASS** |
| BELOW | −20 m alt | — | **PASS** |
| RIGHT | simétrico a LEFT | — | PASS (arnés PC) |

> El error de seguimiento en recta es de pocos metros (±3–10 m). En formaciones **laterales** el
> punto de formación "barre" al girar; con la corrección reforzada (0.6/40) el seguidor recupera
> tras el giro. La línea integral se descartó por inestable.

Herramientas: `tools/bench_restart.ps1` (SITL+bridges), `tools/follow_law_test.py` (afinado de la
ley en PC sin reflashear; soporta las 5 formaciones), `tools/hil_sitl_validate.py` (recta + `--target2`
giro, alabeo y velocidades). `tools/sitl_takeoff.py` robusto (reintentos de armado + streams).

**Cambio de formación en vuelo (web):** la API `POST /api/config` aplica `formation` (0–4),
`prediction` y `filter` en caliente y los **persiste en NVS** (`FWM::setFormation/...`);
`GET /api/config` los devuelve y la página web los carga al abrir. AP del seguidor:
`http://192.168.4.1`. La validación histórica usó el SSID anterior `FWM AP 2`; el firmware actual
deriva `FWM XXXXXX` de la MAC SoftAP y publica la identidad por serie.

**Config del periférico SIN WiFi (tap del bridge):** para el banco HIL, `sitl_bridge.py --tap-port N`
expone un enlace **MAVLink directo a cada placa** por el mismo cable USB (el bench lee/escribe los
parámetros FWM del componente `158` y cambia la formación) **sin conectar el PC a la WiFi del ESP32**.
Validado: lectura de los 11 parámetros y round-trip de `dist_offset` y `formation` por el tap.

**Bench completo (`tools/bench_suite.py`):** escenarios `preflight`, `link`, `setup`, `params`, `formations`
(EN TIERRA, son *ground-only*), `takeoff`, `straight`, `turn`, `mode_gate`, `head_on`, `safety`;
reporte MD+JSON y **series temporales
CSV** por escenario, con escenarios aislados (un fallo no aborta el reporte). `lab.py` reinicia las
placas por DTR/RTS al levantar el banco (las CP210x/ESP32 a veces se cuelgan entre sesiones).
Incluye **preflight** (por el tap: comprueba que el líder tiene FC y no está en "No FC connection")
y `--dist-offset` para fijar el offset de formación EN TIERRA.

**Reunión frente a frente y vuelo cercano (`tools/follow_sim.py`, sin hardware):** replica la ley
cross-track + un modelo de avión y la **latencia del enlace**. Hallazgos:
- Separación real ≈ `dist_offset` + penalización por latencia (~20–30 m por cada 1 s a 20 m/s).
- **Frente a frente sin guarda: riesgo de colisión** (pasan a 0.1–1 m al activar seguir desde 200–500 m).
  Con la **guarda** (`HEAD_ON_GUARD`, ruptura perpendicular a la visual + frenado) se mantiene
  ≥ ~25–50 m para separaciones iniciales ≥200 m; por debajo de ~100–150 m es inevitable.
- Para **10 m** de formación hace falta latencia ≲0.2 s (`TIGHT_FORMATION` + `LORA_SPREADING_FACTOR=7`);
  5 m es el límite físico con la tasa y precisión actuales. La guarda va tras un flag (default off).

**Regresión del predictor TRAIL:** `millis()` del líder y del seguidor son relojes independientes; no se
puede calcular la edad del paquete restando sus timestamps. Además, el horizonte de 1 s adelantaba
20 m a 20 m/s y cancelaba exactamente `dist_offset=20 m` (la recta SITL llegó a **0.2 m**). Se eliminó
esa resta de relojes y el avance predictivo en TRAIL queda limitado a `PREDICTION_MAX_LEAD_FRACTION`
(0.25) del offset. `tools/follow_sim_test.py` → **5/5 PASS** (incluye la regresión sin cap y offsets
5/10/20/96 m).

**Validación SITL a 200 m:** con predictor capado, `dist_offset=20 m`: TAKEOFF a 200 m, recta
**17.8 m media / 15.2 m mínimo**, roll máximo 3.6°, error medio de altitud 0.6 m; giro **16.0 m
mínimo**, roll pico transitorio 44.6°. Los escenarios del banco ya imponen mínimo horizontal de 10 m
para offset20 (no aceptar un casi-choque como PASS). El *home* evita el relieve al norte; rutas al sur.

**Bisección del fallo de seguimiento:** el pre-v2 enlazaba; el v2 inicial tenía (1) `LoraPacket_t` con
padding después del checksum (suma de un byte no inicializado; el autotest también fallaba su propia
aserción de layout), y (2) `Comm::run()` llamaba `LoRa.parsePacket()` en el líder mientras el Ticker
transmitía desde otra tarea, aunque el retorno estaba desactivado. La solución es trama **packed de
40 bytes** con checksum como último byte (assert + selftest byte-a-byte), y escucha del líder solo bajo
`FOLLOWER_REPLY`. Verificado en hardware: RX del seguidor **11→68 en 56 s**, RSSI −49…−56 dBm.

**Matriz hardware (protocolo/base):** `netid` PASS (mismo ID enlaza, distinto congela RX, restore PASS);
preflight/setup/params/formations PASS; modo ACRO cercano suspende actualizaciones y GUIDED reanuda;
recta y giro a 200 m PASS. `tools/proto_sim.py` → **17/17**; `tools/follow_sim_test.py` → **8/8**.
El test de netid es opt-in (`--netid-test`) porque cambia NVS durante la ejecución.

**Progreso en vivo del bench:** `bench_suite.py` imprime con hora, `[i/N]` por escenario, ETA inicial
y trazas periódicas (distancia/distancia mínima) durante recta/giro/head-on/safety.

**Hardening del banco:** `sitl_bridge.py` **reconecta** solo si el SITL cierra SERIAL0 (antes moría y
dejaba a la placa sin FC → el líder dejaba de emitir beacons).

**Protocolo v2 + `netid`:** `src/protocol.h` define una trama wire **packed de 40 bytes**, con
`version/type/netid/mode/seq/flags` (slot de reply y posición válida) y checksum final. `netid` filtra en paquete; por defecto el radio usa
el sync word fijo conocido-bueno y CRC de radio apagado (`NETID_USE_SYNCWORD=0`, `LORA_CRC=0`). El
`netid` es parámetro NVS + WebUI. El autottest comprueba el layout y corrupción de cada byte.

**Gate de modo:** `approach_dist=300 m` param; cerca, el firmware suspende nuevas órdenes si el líder
no está en FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED; bench ACRO→GUIDED PASS. Nota: actualmente
la suspensión conserva el último setpoint del FC; aún falta decidir/validar un modo hold explícito.

**Sesión JOIN/REPLY + OSD del líder (activo):** el líder envía discovery cuando no hay sesión y pide
una respuesta con `LORA_FLAG_REPLY_SLOT`. El REPLY/JOIN del seguidor se envía tras el slot; el líder
abre una ventana RX limitada (`REPLY_WINDOW_MS`) y el beacon se difiere durante ella. El Ticker solo
marca TX pendiente: **todas las operaciones SX1276 se ejecutan secuencialmente desde el loop de vuelo**.
Si faltan REPLYs, `SESSION_TIMEOUT_MS` expira y vuelve discovery; al regresar el peer, re-JOIN/reactiva
la tasa normal. Hardware SITL: reply bidireccional, timeout, discovery y rejoin **PASS**; el FC/GCS
recibe `STATUSTEXT` del líder. El envío ya usa **deduplicación exacta** (texto prefijado + severidad):
si no cambió, no vuelve a transmitirlo; la distancia se anuncia solo al variar al menos 5 m. Los
mensajes de distancia usan `MAV_SEVERITY_INFO` (6). La captura enviada por el usuario confirma que
Mission Planner muestra `158: FWM: Follower 48m` tanto en el overlay HUD como en Messages. El test de
sesión además verifica severidad/origen cuando aparece un STATUSTEXT nuevo; puede no ver uno si el
deduper ya envió exactamente el mismo texto antes. `src/selftest.h` prueba igualdad y cambios de texto
y severidad. El líder publica distancia solo si ambos paquetes declaran posición válida, evitando el
mensaje inicial absurdo antes de recibir GPS.

**Dual-core (`FWM_DUAL_CORE=1`, activo):** core 1 ejecuta el loop de vuelo y es propietario de LoRa;
core 0 ejecuta web/pantalla/logger. `FWM::send_packet_ticker_callback` ya no toca SPI: solo pone una
bandera byte-alineada que consume el core 1. Probado con REPLY por slot: RX crece, recta, giro y gate
pasan. No habilitar `NETID_USE_SYNCWORD`/`LORA_CRC` sin volver a validar RF.

**Simuladores:** `tools/proto_sim.py` → **17/17** (discovery, sesión, timeout, pérdidas, blackout,
recuperación, netid y gate). `tools/follow_sim_test.py` → **8/8** (offset/predicción, cruce y head-on
con/sin guarda).

**Fix del cristal (WiFi):** las placas TTGO LoRa32 V1.0 llevan cristal de **26 MHz**. El core de
Arduino 3.x asumía 40 MHz → la **WiFi/BT quedaban fuera de banda** (no emitían AP ni escaneaban).
La solución es compilar con **`-DF_XTAL_MHZ=26`** (ya en `platformio.ini`): el core lo aplica en
`app_main()` antes de `initArduino()`/WiFi, fija el cristal y reconfigura los relojes. Validado
end-to-end: la placa **escanea 16 redes** y el PC ve/conecta al SSID histórico `FWM AP 2`; el firmware
actual deriva el SSID de la MAC SoftAP. `GET/POST /api/config`
funcionan y la formación **persiste** tras reiniciar. (No requiere recompilar las libs.)

> **Puertos USB no son identidades:** los CP210x reenumeran/intercambian COM al reconectar. En la
> última prueba se identificó líder=SYSID 1 en COMx y seguidor=SYSID 2 en COMx; identificar por
> SYSID/tap antes de flashear, no asumir COM fijo.

### Fase D — Interfaz y observabilidad
- [ ] **D11** Web: config en **tierra** (AP solo en tierra + tabla de parámetros). Ver plan en
  `docs/PLAN_CONFIG_TIERRA.md`. Telemetría en vivo plegada al WebSocket (diagnóstico en tierra).
  - [x] **Fase 1** — Gate del AP solo en tierra + bloqueo de cambios en vuelo (commit `bb97e60`).
  - [x] **Fase 2** — Tabla de parámetros FWM + `/api/params` + UI dark generada (commit `31525e7`).
  - [x] **Fase 3** — Parámetros FWM por MAVLink (`PARAM_REQUEST_LIST/READ/SET`) para Mission Planner (commit `dcbe5d3`).
- [ ] **D12** Menú OLED: reasignar botones a GPIOs libres (o encoder).
- [ ] **D13** Pantalla: estado de enlace, modo, distancia.

### Fase E — Calidad
- [ ] **E14** Tests unitarios con entorno `native` real + CI.
- [ ] **E15** Documentar validaciones en `docs/FLYWITHME.md`.
- [ ] **E16** Hardware: cables/puertos USB fiables; reset estable.

---

## 5. Cómo mantener este documento

Al terminar una tarea: marcar su casilla, y si se corrige un fallo, añadir una fila a §2 con
síntoma, causa y solución.

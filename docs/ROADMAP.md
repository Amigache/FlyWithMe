# FlyWithMe — Estado y roadmap

Documento **vivo** de seguimiento. Registra el estado de validación, los fallos corregidos y el
plan de trabajo por fases. Actualizar al completar cada tarea.

> Última actualización: validación en banco con `FC_EMULATION=1` (dos TTGO LoRa32 V1, sin FC).

---

## 1. Estado actual

| Área | Estado |
|---|---|
| Compilación (`master` + `slave`) | ✅ SUCCESS (Flash ~35.6 %, RAM ~15 %) |
| Flasheo | ✅ (requiere `PYTHONIOENCODING=utf-8`, ver §3) |
| Arranque | ✅ sin crashes ni errores I2C |
| Enlace LoRa líder → seguidor | ✅ **verificado** (`BEACON LOCK`, `FOLLOWING`, `Formation position`) |
| Integración con FC real | ⏳ pendiente (por ahora `FC_EMULATION=1`) |
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

## 3. Entorno y notas de banco

- **Baud serie: 57600**. La placa lleva cristal de **26 MHz**; con `-DF_XTAL_MHZ=26` el core ajusta el
  reloj y el baud es correcto. Sin esa flag saldría ~37440 (×0.65) **y la WiFi/BT no funcionarían**.
- **Flashear en Windows:** `$env:PYTHONIOENCODING='utf-8'; pio run -e <env> -t upload`.
- **Puertos actuales:** `COMx` (master) / `COMx` (slave). **USB inestable** (cable/puerto).
- **`FC_EMULATION = 1`**: sintetiza telemetría en `APdata` sin UART1 y **mantiene el LoRa real**.
  Poner `0` para vuelo con FC.
- **`USE_INTERACTIVE_MENU = 0`** por conflicto de pines.

### Arnés HIL

```powershell
# Ambas placas flasheadas con FC_EMULATION=1
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
2. Flashear la placa con el entorno SITL (`FC_LINK_USB=1`, MAVLink por USB/UART0):
   ```
   pio run -e ttgo-lora32-v1-master-sitl -t upload        # COMx (líder)
   pio run -e ttgo-lora32-v1-slave-sitl  -t upload        # COMx (seguidor)
   ```
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
> ⚠️ **sysid:** SITL emite `sysid=1`; el seguidor espera `TARGET_SYSID=2`, por eso `slave-sitl`
> añade `-D TARGET_SYSID=1`.
>
> **Formación en banco:** `slave-sitl` añade `-D MIN_SAFE_ALTITUDE=0` para validar `Formation position`
> con el avión SITL en el suelo. Producción mantiene 50 m.
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
- [x] **B5a** Enlace de pruebas con SITL por USB (`FC_LINK_USB=1`): **líder y seguidor verificados**
  (`LINK TO FC OK`, `BEACON LOCK`, `dist`). Añadidos `tools/sitl_bridge.py` y entornos `*-sitl`
  (el de seguidor con `-D TARGET_SYSID=1`).
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
`FWM AP 2` / `http://192.168.4.1`.

**Config del periférico SIN WiFi (tap del bridge):** para el banco HIL, `sitl_bridge.py --tap-port N`
expone un enlace **MAVLink directo a cada placa** por el mismo cable USB (el bench lee/escribe los
parámetros FWM del componente `158` y cambia la formación) **sin conectar el PC a la WiFi del ESP32**.
Validado: lectura de los 11 parámetros y round-trip de `dist_offset` y `formation` por el tap.

**Bench completo (`tools/bench_suite.py`):** escenarios `link`, `params`, `formations` (EN TIERRA,
son *ground-only*), `takeoff`, `straight`, `turn`, `safety`; reporte MD+JSON y **series temporales
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

**Validación EN VUELO (SITL a 200 m, `dist_offset=20 m`):** recta **15–17 m ±7**, giro **19 m ±15**,
`safety` OK. El **head-on** (media vuelta del líder) dio **min 0.5 m con `guard_hits=9`, `face_max=175`,
`rng_min=0`** → la guarda **se activa pero NO evita** el casi-choque a 20 m (solo ~0.5 s de cierre).
Con `dist_offset ≥ 96 m` no hay colisión. La ruptura **vertical** (trepar durante la guarda, modelada)
mejora poco a corta distancia (falta tiempo). **Conclusión: en formación cerrada una media vuelta
brusca del líder es intrínsecamente insegura** → mantener offset ≳100 m, o no invertir el rumbo
apretado, o disparar la guarda mucho antes (por tiempo-de-colisión). Nota: el bench a **200 m** evita
el relieve al norte del *home* (a 80 m se metía en la montaña).

**Progreso en vivo del bench:** `bench_suite.py` imprime con hora, `[i/N]` por escenario, ETA inicial
y trazas periódicas (distancia/distancia mínima) durante recta/giro/head-on/safety.

**Hardening del banco:** `sitl_bridge.py` **reconecta** solo si el SITL cierra SERIAL0 (antes moría y
dejaba a la placa sin FC → el líder dejaba de emitir beacons).

**Fix del cristal (WiFi):** las placas TTGO LoRa32 V1.0 llevan cristal de **26 MHz**. El core de
Arduino 3.x asumía 40 MHz → la **WiFi/BT quedaban fuera de banda** (no emitían AP ni escaneaban).
La solución es compilar con **`-DF_XTAL_MHZ=26`** (ya en `platformio.ini`): el core lo aplica en
`app_main()` antes de `initArduino()`/WiFi, fija el cristal y reconfigura los relojes. Validado
end-to-end: la placa **escanea 16 redes** y el PC ve/conecta a `FWM AP 2`; `GET/POST /api/config`
funcionan y la formación **persiste** tras reiniciar. (No requiere recompilar las libs.)

> ⚠️ **Pendiente de banco:** al terminar las pruebas el puerto `COMx` (CP210x) quedó bloqueado
> (`semaphore timeout`), así que la placa seguidora conserva el firmware de la prueba BELOW. El
> **código del repo ya está en TRAIL por defecto**; reenchufar/rebootear y reflashear
> `-e ttgo-lora32-v1-slave-sitl` cuando el puerto se libere.

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

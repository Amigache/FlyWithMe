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
| 5 | Baud equivocado | Cristal 26 MHz; el core 3.x genera ~38400 al configurar 57600 | `monitor_speed = 38400` |
| 6 | **El seguidor no enlazaba** (error ~100 km) | Compresión truncaba hacia cero: longitudes negativas mal reconstruidas | `floor()` en `compressPacket` |
| 7 | FSM bloqueada | `SEARCHING/LOST_LINK → FOLLOWING` no permitidas | Ampliadas transiciones válidas |
| 8 | Logs ilegibles (`2f`, `1fm`) | ArduinoLog no soporta `%f` | Formatear como enteros escalados |

## 3. Entorno y notas de banco

- **Baud serie: 38400** (no 57600). Cristal de 26 MHz → el core genera ~x0.66.
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

1. Lanzar los vehículos SITL (nativo Windows, en `Documents\Mission Planner\sitl`):
   ```
   ArduPlane.exe --instance 0 --serial0 tcp:5760 -M+ -s1 --home -35.363261,149.165230,584,353 -P SERIAL1_PROTOCOL=2 -P SERIAL2_PROTOCOL=2
   ArduPlane.exe --instance 1 --serial0 tcp:5770 -M+ -s1 --home -35.3633006,149.165230,584,353 -P SERIAL1_PROTOCOL=2 -P SERIAL2_PROTOCOL=2
   ```
   Puertos por instancia: SERIAL0 `5760+10N` (ESP32), SERIAL1 `+2` (Mission Planner), SERIAL2 `+3` (control).
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
- [ ] **C8** Afinar predicción/filtro y offsets de formación.
- [ ] **C9** Control de velocidad/altitud (`calculate_dynamic_speed`, `ALT_OFFSET`).
- [ ] **C10** Tasa adaptativa del líder (hoy usa 500 m fijos).

### Fase D — Interfaz y observabilidad
- [ ] **D11** Web: telemetría en vivo, config persistente, descarga de logs.
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

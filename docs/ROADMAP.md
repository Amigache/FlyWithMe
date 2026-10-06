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
- [ ] **B5** Verificar/ajustar el baud del UART1 (posible desfase por cristal).
- [ ] **B6** Validar MAVLink real: RX (HEARTBEAT/GLOBAL_POSITION_INT), `request_data_streams`,
  `nav_waypoint` en GUIDED, `do_change_speed`.
- [ ] **B7** Robustez de conexión: sysid/compid configurables, reconexión y timeouts.

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

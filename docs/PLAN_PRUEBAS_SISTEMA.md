# Plan de pruebas y validación integral — FlyWithMe

Plan vivo para validar código, firmware, herramientas CLI, SITL/HIL y releases. La validación avanza
por niveles; un PASS de simulación software no sustituye HIL, y un PASS HIL no autoriza vuelo real.

## 1. Reglas de seguridad y alcance

- **Producción:** `ttgo-lora32-v1-flight`, `FC_LINK_USB=0`, UART1 y `FWM_ALLOW_RUNTIME_SITL=0`.
  El firmware debe rechazar `FWM SIM ON`; los bundles/releases solo contienen `flight`.
- **Desarrollo HIL:** `ttgo-lora32-v1-sitl` arranca como flight, pero permite `FWM SIM ON` solo después
  del interlock de tierra. El modo es volátil: reset vuelve a UART1. No volar con esta imagen; cargar
  `ttgo-lora32-v1-flight` antes de operación real.
- No ejecutar `head_on` ni `netid_test` por defecto. `head_on` solo con ArduPlane SITL, nunca con FC real;
  `netid_test` altera NVS temporalmente y exige readback/restauración.
- HIL requiere placas y aeronaves en tierra/desarmadas, antenas LoRa conectadas y puertos COM correctos.
  No compartir COM entre monitor, provisión y bridge.
- No iniciar una etapa destructiva o de movimiento sin confirmación actual del usuario.

## 2. Niveles y criterios

| Nivel | Área | Criterio para PASS |
|---|---|---|
| L0 | Compilación y estática | Flight + dev/HIL compilan; scripts Python compilan; `git diff --check` limpio. |
| L1 | Código puro/modelos | Self-test de protocolo, simuladores de protocolo/guiado y tests unitarios pasan. |
| L2 | Herramientas CLI | Scripts, bridges serie, provisión, MAVProxy, reportes y empaquetado de firmware pasan. |
| L3 | SITL sin placas | Dos ArduPlane (SYSID 1/2), MAVProxy inicia, heartbeats visibles en UDP 14550 y cierre limpio. |
| L4 | HIL | SIMCFG OK en ambas, heartbeats FWM `COMPID=158` visibles por ambos TAP, roles/SYSID correctos y escenarios seleccionados terminan con reporte. |
| L5 | Distribución/FC real | Release contiene solo firmware `flight`; firmware de producción rechaza SIM; validación de FC real requiere plan/autorización aparte. |

Un fallo bloquea el siguiente nivel hasta tener causa y corrección documentadas.

## 3. Matriz ejecutable

### Código y firmware

| ID | Prueba | Comando / método | Estado |
|---|---|---|---|
| FW-01 | Build flight de producción | `pio run -e ttgo-lora32-v1-flight` | PASS (2026-10-08) |
| FW-02 | Build universal dev/HIL | `pio run -e ttgo-lora32-v1-sitl` | PASS (2026-10-08) |
| FW-03 | `SELFTEST` de protocolo/layout/checksum/netid/SIM identidad | Arranque; exigir `SELFTEST PASS` | PASS observado (62 checks) |
| FW-04 | SIM bloqueado en firmware flight | En placa flight segura, `FWM SIM ON` debe responder ERR y conservar UART1 | Pendiente de prueba física; build define `FWM_ALLOW_RUNTIME_SITL=0` |
| FW-05 | SIM runtime solo tras timeout inicial sin FC | En placa dev, probar WAIT antes del timeout, luego SIMCFG OK; un enlace previo enclavado debe bloquearlo | SIMCFG OK probado en COMx/COMx; WAIT/link-lock pendiente |
| FW-06 | SIM no persistente | Después de SIMCFG OK, reset y consultar `FWM ID`; comprobar el arranque normal por UART1 | PASS funcional: Stop/reset y `FWM ID` responden en ambos; roles leader/follower conservados |
| FW-07 | Perfil dinámico del banco | Confirmar SF7/BW250, intervalo 200 ms y mínimo de altitud 0 solo durante SIM; reset recupera SF12/BW125, intervalo/altitud flight | Pendiente HIL |
| FW-08 | Compatibilidad NVS | Reflashear sin borrar NVS; role/netid/formación sobreviven; `runtimeSitlMode` nunca aparece en NVS | Parcial: roles persistieron en COMx/COMx |
| FW-09 | Unity/native de módulos puros | `pio test -e native` (12 pruebas contra `src/protocol.h`, `status_text.h`, `wifi_identity.h`, `selftest.h`) | Cerrado en código y en CI; ver `docs/VALIDACION_PRE_RELEASE.md` §1 |

### Simuladores software (L1)

| Prueba | Resultado |
|---|---|
| `python tools/proto_sim.py` — matriz de protocolo, pérdidas, blackout, netid y gate | PASS 17/17 |
| `python tools/follow_sim_test.py` — predictor, offsets y escenarios geométricos | PASS 8/8; simulación matemática, no maniobras SITL |

### Herramientas CLI, simuladores y distribución

| ID | Prueba | Criterio | Estado |
|---|---|---|---|
| CLI-01 | Simuladores software | `python tools/proto_sim.py` y `python tools/follow_sim_test.py` | PASS: 17/17 y 8/8 |
| CLI-02 | Bridge serial CP210x | DTR/RTS deben configurarse antes de `Serial.open()` para evitar reset espurio | PASS unitario en `tools/tests/test_sitl_bridge_serial.py` |
| CLI-03 | MAVProxy headless | `python tools/mp_launch.py --help`; daemon y salida UDP en el bench | PASS en HIL; visualización GCS en Mission Planner pendiente |
| REL-01 | Bundle de firmware | Manifest/hash válidos; solo perfil publicable `flight` | Empaquetado definido por `tools/package_firmware_release.py`; release pública pendiente |

### SITL/HIL y comportamiento

| ID | Prueba | Criterio | Estado |
|---|---|---|---|
| HIL-01 | Inicio del lab | No hay puertos ocupados; roles preconfigurados/readback; SIMCFG OK antes de abrir bridges | PASS en reporte completo `20261008-211545`; bridge conecta inmediatamente mientras inicia SITL |
| HIL-02 | TAP de cada placa | HEARTBEAT del sistema FWM, `COMPID=158`, visible en TCP 5790/5791; decodificación de parámetros funciona | PASS: roles/SYSID 1/2, parámetros y diagnósticos MAVLink leídos en ambos TAP |
| HIL-03 | Preflight/link FC y LoRa | FC SYSID 1/2 desarmados, round-trip MAVLink, beacon/RX y REPLY por LoRa | PASS: preflight/link, `FWM_RX/TX/LINK`, sesión bidireccional y recuperación observados |
| HIL-04 | Setup/params FWM | Leer/escribir/readback de parámetros sin WiFi; no reiniciar si valor no cambia | PASS: `dist_offset` 20→111→20; 5 formaciones y TRAIL restaurado; netid restaurado |
| HIL-05 | Session/formations en tierra | JOIN/REPLY, pérdida/recuperación; cambios de formación y restauración | PASS: sesión/rejoin y cinco formaciones; NVS restaurada |
| HIL-06 | Takeoff/recta/giro/mode gate/safety | PASS/FAIL por escenario y CSV; ArduPlane SITL únicamente, no FC real | PASS en SITL: takeoff, recta, giro, mode gate y safety; separación mínima 13.9 m en safety |
| HIL-07 | `netid_test` opt-in | Guardar netid original, aplicar/medir/restaurar y readback PASS | PASS: enlace con mismo ID, filtro con ID distinto y restauración a `4660` |
| HIL-08 | `head_on` opt-in | Solo dos SITL; imagen dev con `HEAD_ON_GUARD=1`; confirmar guarda y separación mínima | PASS: 4 muestras activas de guarda, cara máxima 178°, separación mínima 24.8 m; solo SITL |
| HIL-09 | Stop/fallo/cancelación | Cerrar solo hijos lanzados por esta sesión, liberar COM/puertos y resetear dev boards a UART1 | PASS: Stop normal, CtrlBreak (`exit=130`) y fallo SIMCFG; `finally`/rollback liberan procesos/puertos y `FWM ID` confirma roles |

### Web/AP/FC real y vuelo

- Validar `/api/params`, `/api/ap`, parámetros MAVLink, autenticación WebUI y apertura externa del navegador.
- Verificar `AUTO/ON/OFF`, AP visible solo bajo política de tierra, AP apagado al armar/moverse y bloqueo
  tras pérdida de un enlace FC previamente establecido.
- Verificar SSID `FWM XXXXXX`/BSSID/MAC `FWM_ID`, consulta `FWM ID` y escaneo desde más de un cliente WiFi.
- Probar con FC real solo en tierra primero: heartbeat, GPS válido, armed state, UART1 y toma de control.
  Cualquier vuelo real requiere plan y autorización separados; no es parte automática del bench SITL.

## 4. Ejecución iniciada y bloqueos actuales

- Builds flight y dev/HIL pasan; los simuladores software pasan 17/17 y 8/8. MAVProxy arranca dentro del bench.
- Las placas COMx (líder) y COMx (seguidor) tienen ahora la imagen universal dev. Ambas respondieron
  `SIMCFG OK`; tras reset, `FWM ID` devolvió sus MAC/roles. **No volar con esta imagen**: cargar flight
  bloqueado antes de producción.
- Los reportes anteriores `20261008-160517`–`20261008-172447` detectaron el TAP y el preflight frágiles.
  Causa: esperar 11 s antes del bridge dejaba vencer el ticker FWM; además SIM silencia logs de texto,
  por lo que los tests no podían medir RX/TX. Se corrigieron el orden de arranque y la instrumentación
  MAVLink estructurada.
- Suite completa del bench con confirmación de netid/head-on:
  `AppData/Local/FlyWithMe/reports/20261008-211545/report.md` → **PASS=20, FAIL=0, SKIP=0**.
  Incluye session/rejoin, netid y restauración, parámetros/formaciones, takeoff SITL, recta/giro,
  mode gate, head-on guard y safety.
- Takeoff/head-on fueron solo en **ArduPlane SITL**; no hubo FC real ni vuelo físico. La guarda se
  compiló habilitada solo en el perfil dev/HIL; flight de producción conserva `HEAD_ON_GUARD=0`.
- Las placas terminaron en UART1; `FWM ID` volvió a confirmar COMx líder/SYSID 1 y COMx follower/SYSID 2.
  Siguen con imagen dev/HIL; **no volar con ella**.
- La prueba HIL completa se ejecutó con autorización; cualquier repetición de `netid`/`head_on` sigue
  siendo opt-in y exclusiva de SITL.

## 5. Orden recomendado

1. L0/L1/L2 en cada cambio; actualizar reporte de pruebas y guardar evidencia.
2. Lanzar SITL headless sin firmware/HIL; comprobar MAVProxy UDP 14550 y limpieza.
3. La suite HIL completa desde `tools/bench_suite.py` ya pasó; repetirla ante cambios relevantes, siempre
   dejando `netid`/`head_on` como opt-in y restringidos a SITL.
4. Completar pruebas de Web/AP, Mission Planner visual, bundle de release y Unity/native.
5. Reflashear `ttgo-lora32-v1-flight`, verificar SIMCFG ERR, SELFTEST, roles y AP; solo entonces preparar
   cualquier prueba de FC real.
6. Publicar únicamente el firmware `flight`; nunca incluir el bundle dev/HIL.

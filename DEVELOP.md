# FlyWithMe — desarrollo y banco de pruebas

Este documento reúne los detalles técnicos, perfiles PlatformIO, SITL, simuladores y bench. La guía
para el usuario final está en [`README.md`](README.md); el estado de validación vivo, en
[`docs/ROADMAP.md`](docs/ROADMAP.md).

## Arquitectura

Sistema para **ArduPlane**: el líder publica estado por SX1276/LoRa; el seguidor valida la trama,
calcula su posición de formación y envía guiado MAVLink en `GUIDED`.

```text
[LÍDER]    FC --MAVLink/UART--> Telem --> FWM protocol v2 --> LoRa
[SEGUIDOR] FC <--MAVLink/UART-- Telem <-- Comm <-- LoRa
```

- `src/main.cpp`: arranque y tareas FreeRTOS.
- `src/FWM.cpp`: modos, sesión, scheduler de beacons, gate, parámetros y UI.
- `src/Comm.cpp`: único propietario de las operaciones LoRa en el loop de vuelo; RX/TX, validación,
  slots JOIN/REPLY y secuencias.
- `src/Telem.cpp`: MAVLink con el FC, guiado, predictor, formación y `STATUSTEXT`.
- `src/protocol.h`: wire-format v2 **packed de 40 bytes**, sin padding en checksum.
- `src/selftest.h` / `src/status_text.h`: auto-test de protocolo y deduplicación pura testeable.
- `src/config.h`: flags, límites, parámetros y tablas de modo.
- `lib/mavlink/`: cabeceras generadas; **no editar**.

## Perfiles PlatformIO

| Entorno | Rol/uso | FC |
|---|---|---|
| `ttgo-lora32-v1-flight` | Imagen común para todas las placas de vuelo | `FC_EMULATION=0`, UART1 |
| `ttgo-lora32-v1-sitl` | Imagen común para placas conectadas a ArduPlane SITL | `FC_EMULATION=0`, `FC_LINK_USB=1`, SF7/BW250k/200ms |
| `ttgo-lora32-v1` | Banco con FC emulado localmente | `FC_EMULATION=1`; no usar para vuelo real |

Los perfiles usan Arduino ESP32 core 3.x, `-DF_XTAL_MHZ=26`, C++17 y `huge_app.csv`. El perfil
`ttgo-lora32-v1-flight` no define el rol en build flags: todas las placas reciben el mismo binario.
`role` se almacena por placa en NVS (`OFF=0`, `FOLLOWER=1`, `LEADER=2`); placas sin ese dato (incluidas
las que conservan NVS de firmware antiguo) arrancan en `OFF` por seguridad y deben provisionarse.
Cambiar el rol en tierra guarda NVS y reinicia la placa. SYSID FWM/FC: líder 1 y seguidor 2.
Una placa nueva en `OFF` puede provisionarse antes de detectar el FC; al cambiar un rol activo se exige
telemetría con posición válida, desarmado y en tierra (un FC desconectado durante el vuelo no cuenta
como prueba de estar en tierra).

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
pio run -e ttgo-lora32-v1-flight
pio run -e ttgo-lora32-v1-sitl
pio run -e ttgo-lora32-v1

# Cargar la misma imagen de vuelo y provisionar cada rol; sustituir los COM
& $py tools\flash_firmware.py --port COMx --role leader
& $py tools\flash_firmware.py --port COMy --role follower

# Solo reprovisionar, sin volver a cargar firmware
& $py tools\flash_firmware.py --port COMx --role follower --provision-only

# Monitor serie
pio device monitor --port COMx --baud 57600
```

En Windows, el script configura `PYTHONIOENCODING=utf-8` para PlatformIO y usa USB serie a 57600 para
provisionar el rol. Requiere `pyserial`. Los CP210x reenumeran; identificar el COM actual. Los perfiles
de vuelo fijan `FC_LINK_USB=0`; SITL lo pone en `1` para MAVLink por UART0. No conectar dos dueños al
mismo puerto.

## Configuración efectiva

- Radio: `LORA_BAND=866E6`, SF12/BW125k en vuelo por defecto, sync word fijo `0x34`.
- Banco SITL: SF7/BW250k y 5Hz para reducir latencia; estos tiempos **no representan** SF12 real.
- `LORA_CRC=0`, `NETID_USE_SYNCWORD=0` por defecto. El frame tiene checksum aditivo de un byte,
  `version`, `type`, `netid`, `mode`, `seq` y flags. `netid=4660 (0x1234)` filtra redes a nivel de
  paquete, pero **no cifra ni autentica**.
- `USE_COMPRESSED_PACKETS=0`: el compresor anterior no contiene todos los campos v2.
- `FWM_DUAL_CORE=1`: core1 es propietario del loop de vuelo/radio; core0 ejecuta Web/OLED/logger. El
  callback del Ticker solo pone una bandera; no accede al SPI/SX1276.
- `FOLLOWER_REPLY=1`: JOIN/REPLY solo cuando un BEACON solicita un slot. El líder escucha una ventana
  RX limitada; timeout de sesión y vuelta a discovery están modelados en `proto_sim.py`.
- `netid`, `formation`, offsets y `approach_dist` se persisten en NVS. WebUI y `PARAM_SET` son solo
  configurables en tierra (salvo parámetros marcados explícitamente).
- La WebUI ordena `role` primero y filtra campos con `ParamDef_t.scope`: líder muestra los comunes
  (`role`, `netid`, `link_timeout`); seguidor añade formación, offsets, ganancias, predicción, filtro,
  `foll_enable` y `approach_dist`. Los valores ocultos se conservan en NVS; el servidor MAVLink sigue
  publicando la tabla completa.
- `approach_dist=300m`: por debajo se exige modo líder FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED.
  Si se suspende por modo inestable, el FC conserva el último setpoint: **no es un LOITER/hold**.
- `dist_offset=96m` por defecto. Predictor TRAIL limitado a `PREDICTION_MAX_LEAD_FRACTION=0.25`;
  no se resta `millis()` de dos ESP32 no sincronizados.
- `MIN_SAFE_ALTITUDE=50m` compara `relative_alt` MAVLink con el home del autopiloto; no es AGL ni
  una comprobación de terreno. El AP web solo se usa en tierra.
- `HEAD_ON_GUARD=0` por defecto: el escenario head-on sigue siendo experimental y solo SITL.

## Banco SITL y MAVProxy

Se usa **ArduPlane** (no ArduCopter). `tools/lab.py` levanta dos SITL y MAVProxy, que combina ambos
vehículos en una salida UDP para Mission Planner. SERIAL0 del SITL va a la placa por `sitl_bridge.py`;
SERIAL1 va a MAVProxy/MP y SERIAL2 es el enlace de control.

```powershell
# SITL sin placas: abre menú interactivo
python tools\lab.py

# Opcional: conecta las placas SITL por puentes serie; los COM son ejemplos
python tools\lab.py --firmware --leader-com COMx --slave-com COMx
# Menú: 1 iniciar/reiniciar banco; 2 estado; 9 bench completo; 8 parar banco; 0 salir
```

Con `--tap-port`, cada bridge expone MAVLink directo al periférico (TCP localhost 5790 líder / 5791
seguidor) por el mismo USB, sin WiFi. El tap se usa para parámetros FWM y observación serie. El bridge
reconecta si el SITL cierra SERIAL0. El lab aplica DTR/RTS reset a las placas al comenzar.

`mp_launch.py` arranca MAVProxy headless en Windows sin wxPython y cambia el bind UDP a efímero para
no chocar con Mission Planner en el mismo PC. Mission Planner escucha por defecto en `UDP 14550`.

### Pasos del banco

1. Compila y flashea en ambas placas la misma imagen `ttgo-lora32-v1-sitl`. No mezcles el perfil
   `*-flight` con la configuración UART0 del banco. Si los roles están en `OFF`, el bench los asigna
   automáticamente por TAP al comenzar; fuera del bench, configúralos desde la WebUI/MAVLink.
2. Arranca `tools/lab.py` y elige la opción 1. La aplicación abre dos ArduPlane SITL con SYSID1/2,
   MAVProxy y, si se usó `--firmware`, bridges serie para las placas.
3. Conecta Mission Planner a `UDP 14550` y confirma que aparecen los dos vehículos.
4. Ejecuta primero los checks de enlace y sesión; después takeoff, recta y giro. La opción 9 ejecuta
   el bench completo. La opción 8 detiene el banco; la opción 0 sale y también realiza limpieza.
5. El bench provisiona `role=LEADER` y `role=FOLLOWER` mediante el tap si hace falta; por ello puede
   reiniciar ambas placas y deja esos roles persistidos en NVS. Los informes se guardan bajo
   `tools/reports/`. Los escenarios de red que cambian NVS y los potencialmente colisivos están
   desactivados por defecto; revisa las opciones antes de ejecutar un bench.

## Tests y escenarios

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py tools\proto_sim.py                 # matriz protocolo (17 checks)
& $py tools\follow_sim_test.py           # predictor, offsets y geometría (8 checks)
& $py tools\bench_suite.py --dist-offset 20 --only preflight,link,setup,session,takeoff,straight
& $py tools\bench_suite.py --netid-test  # ground-only; cambia y restaura NVS
& $py tools\bench_suite.py --head-on-test # solo SITL; maniobra potencialmente colisiva
```

El bench genera `report.md`, `report.json` y CSV por escenario en `tools/reports/<fecha>/`. TAKEOFF
posiciona ambos a 200 m antes de medir y las rutas normales van al sur porque al norte del home hay
relieve. Los umbrales de recta/giro fallan si la separación horizontal mínima baja de 10 m al probar
offset 20 m. El head-on está **omitido por defecto** y requiere opción explícita.

Escenarios: preflight, link, setup/netid, round-trip params, cinco formaciones en tierra, TAKEOFF,
recta, giro, gate ACRO→GUIDED, head-on (opt-in) y safety. `session` comprueba REPLY/RX, expiración,
discovery y re-JOIN; `leader_osd` registra STATUSTEXT recibido por MAVLink. La captura del usuario
confirmó el overlay HUD de Mission Planner.

## Plan de aceptación SITL: imagen común, rol y WebUI

**Objetivo:** validar que ambas placas ejecutan el mismo binario SITL, que el rol solo cambia en NVS y
que la WebUI muestra los controles correspondientes sin perder los valores ocultos.

### 1. Preflight del banco

- Confirmar ambos aviones en tierra/desarmados y que no haya `ArduPlane`, `sitl_bridge` ni monitores
  ocupando COM.
- Identificar los puertos CP210x en cada sesión. Las últimas placas confirmadas fueron COMx líder y
  COMx seguidor, pero volver a comprobarlos.
- Guardar el estado actual de los parámetros NVS relevantes (`role`, `netid`, `formation`, offsets).

### 2. Mismo firmware de prueba

- Compilar una sola vez `ttgo-lora32-v1-sitl` y cargar el mismo
  `.pio/build/ttgo-lora32-v1-sitl/firmware.bin` en las dos placas. No usar perfiles separados por rol.
- Mantener `FC_LINK_USB=1` para que cada placa reciba el SITL por USB/UART0; este perfil es solo banco,
  no se deja en los aviones para vuelo real.
- Carga del perfil de prueba (misma configuración en los dos puertos):

  ```powershell
  $env:PYTHONIOENCODING = 'utf-8'
  pio run -e ttgo-lora32-v1-sitl -t upload --upload-port COMx
  pio run -e ttgo-lora32-v1-sitl -t upload --upload-port COMx
  ```

- Iniciar `tools/lab.py --firmware --leader-com COMx --slave-com COMx`, opción 1. Con los TAP activos,
  `bench_suite.py` asegura `role=LEADER` en el TAP del líder y `role=FOLLOWER` en el del seguidor,
  reinicia si hace falta y verifica el readback.

### 3. Criterios de aceptación de rol y enlace

- El parámetro MAVLink/Web `role` devuelve `2` en líder y `1` en seguidor después del reinicio.
- El firmware deriva SYSID 1/2 y peer esperado según el rol; solo el líder transmite BEACON y el
  seguidor recibe, responde JOIN/REPLY y entra en seguimiento al estar en `GUIDED`.
- El rol cambia solo en tierra; al cambiarlo, NVS conserva el valor y la placa reinicia. Una placa
  nueva o antigua sin clave `role` debe iniciar `OFF` hasta provisionarla.
- Ejecutar el bench completo, sin `--head-on-test` ni `--netid-test`. Exigir PASS en preflight, link,
  role setup, session, parámetros, formaciones, takeoff, recta, giro, mode gate y safety. Revisar los
  CSV y reportes bajo `tools/reports/`.

### 4. Criterios de aceptación de la WebUI

- En ambos AP, `role` es el primer control y presenta OFF/FOLLOWER/LEADER.
- En LEADER solo aparecen `role` y los ajustes comunes (`netid`, `link_timeout`). En FOLLOWER aparecen
  además formación, offsets, ganancias, predicción, filtro, `foll_enable` y `approach_dist`.
- Cambiar el selector filtra el formulario antes de guardar. Cambiar de rol y volver no borra los
  valores de seguidor guardados en NVS. Guardar un rol reinicia la placa y mantiene el rol seleccionado.
- Este filtro es de presentación de la WebUI; los parámetros siguen disponibles en la tabla MAVLink
  para diagnóstico y compatibilidad.

### 5. Restauración y cierre

- Tras SITL, cargar `ttgo-lora32-v1-flight` en ambas placas con `tools/flash_firmware.py`, manteniendo
  `--role leader` para COMx y `--role follower` para COMx.
- Confirmar `SELFTEST PASS`, AP/rol esperados y que el perfil final sea UART1 (`FC_LINK_USB=0`,
  `FC_EMULATION=0`). No considerar los resultados SITL una autorización de vuelo real.

## Limitaciones de tests

`tools/follow_sim.py` permite observar la ley de guiado con diferentes distancias y latencias sin
placas; `tools/hil_test.py` comprueba el enlace por serie con FC emulado (cargar el perfil común y
provisionar roles distintos con `flash_firmware.py`). `platformio.ini` no define
`[env:native]`; `pio test` no corre todavía contra `src/`, y `test/test_main.cpp` duplica helpers en
vez de enlazar la implementación del firmware. El auto-test `SELFTEST PASS` se ejecuta en cada
arranque y comprueba layout wire, checksum, versión, netid, secuencia y deduplicación de status text.
Los simuladores son modelos deterministas, no sustituyen SITL ni pruebas RF.

Documentación de desarrollo complementaria: [`docs/FLYWITHME.md`](docs/FLYWITHME.md) (diseño y fases)
y [`docs/ROADMAP.md`](docs/ROADMAP.md) (estado de validación y tareas pendientes).

# Validación pre-release — FlyWithMe

Fecha: 2026-10-08 · Rama `develop` (historial reescrito; el commit de publicación es el último de `develop`).
Entorno: Windows, PlatformIO Core 6.2.0, Python 3.13.2, Node 24.12 / npm 11.6.2, git 2.55.

**Sin hardware conectado, sin ArduPlane SITL, sin navegador con Web Serial y sin compilador de host.**
Las pruebas en placa y en SITL de las modificaciones de esta ronda no se han podido ejecutar aquí.

Criterios de niveles: `docs/PLAN_PRUEBAS_SISTEMA.md` §2.

## 1. Resultados

| ID | Nivel | Prueba | Comando | Resultado |
|---|---|---|---|---|
| FW-01 | L0 | Build producción | `pio run -e ttgo-lora32-v1-flight` | **PASS** · `firmware.bin` 1 164 544 B · plataforma `espressif32@55.3.37` · framework 3.3.7 |
| FW-02 | L0 | Build dev/HIL | `pio run -e ttgo-lora32-v1-sitl` | **PASS** · mismas versiones |
| FW-10 | L0 | `default_envs` | `pio project config` | **PASS** · `default_envs = ttgo-lora32-v1-flight` |
| F11-01 | L0 | Versiones fijadas | comparación del log de build | **PASS** · GFX 1.12.6, SSD1306 2.5.17, BusIO 1.17.4, ArduinoLog 1.1.1, LoRa 0.8.0, ESPAsyncWebServer 3.12.1, AsyncTCP 3.5.0 |
| L0-3 | L0 | Compilación de scripts Python | `python -m py_compile` | **PASS** |
| L0-4 | L0 | Whitespace y conflictos | `git diff --check` | **PASS** |
| L0-5 | L0 | Sintaxis de workflows | `yaml.safe_load` | **PASS** (sin cambios de estructura; se pinó `platformio==6.2.0`) |
| L0-6 | L0 | JavaScript de la WebUI del firmware | `node --check` sobre el `<script>` de `src/Web.cpp` | **PASS** (comprobado antes del último cambio; el cambio de `checkPw` se revalida en el siguiente build de CI) |
| CLI-02 | L1 | Bridge serie, DTR/RTS antes de `open()` | `python -m unittest discover -s tools/tests -p "test_*.py"` | **PASS** 1/1 |
| CLI-01a | L1 | Matriz de protocolo | `python tools/proto_sim.py` | **PASS** 17/17 (sin cambios) |
| CLI-01b | L1 | Predictor y offsets | `python tools/follow_sim_test.py` | **PASS** 8/8 (sin cambios) |
| L1-4 | L1 | Ley de guiado contra SITL | `python tools/follow_law_test.py` | **No ejecutable aquí**: necesita SITL en TCP 5760/5770 |
| FW-09 | L1 | Pruebas Unity en firmware | `pio test` | **FAIL** (`ERRORED`), F-13 abierto |
| FW-11 | L1 | Self-test (checks de protocolo + validador de clave) | compilado; se ejecuta al arrancar | **Compilado**. Pendiente en placa: `SELFTEST PASS` con **66 checks** |
| REL-01 | L2 | Bundle de release `flight` | `python tools/package_firmware_release.py --version v0.0.0-test --commit final --output <tmp>` | **PASS** · 742 802 B · SHA-256 `580bfacb…` |
| WEB-01 | L2 | Flasheador web | `python tools/build_web_flasher.py --version v0.0.0-test --commit final --output <tmp>` | **PASS** · 4 particiones en `0x1000`, `0x8000`, `0xE000`, `0x10000` |
| WEB-02 | L2 | Instalación desde el navegador | `web/index.html` con ESP Web Tools 10.4.0 | **No validado** con placa |
| WEB-03 | L2 | Asignación de rol por Web Serial | panel «Asignar rol» | **No validado** con placa ni navegador |
| PW-01 | L2 | Cambio de clave desde la WebUI y MAVLink | `POST /api/ap/pass`, `WIFI_CONFIG_AP` | **Compilado**. Pendiente en placa; MAVLink no validado con Mission Planner |
| PW-02 | L2 | Bloqueo opcional con clave de fábrica | `FWM_FORCE_AP_PASS_CHANGE=0` (por defecto) | **No aplica** por decisión: la clave de fábrica es válida |
| F05-01 | L2 | Escrituras fail-closed (`canWriteConfig`) | `groundOnly`, `/api/config`, `PARAM_SET` | **Compilado**. Pendiente en placa: con FC enlazado y luego desconectado en vuelo, las escrituras deben denegarse; en banco sin FC nunca enlazado, deben permitirse |
| FLASH-01 | L2 | Carga y provisión por script | `tools/flash_firmware.py` | **No ejecutado**: requiere placa |
| L3 | L3 | SITL sin placas | `python tools/lab.py` | **No ejecutado**: sin ArduPlane |
| L4 | L4 | HIL completo | `tools/bench_suite.py` | **No repetido**. Último reporte documentado: `20261008-211545`, PASS 20/20. Debe repetirse con el firmware nuevo |
| L5 | L5 | Distribución y FC real | — | **Pendiente** |

## 2. Cambios de esta ronda

- **Clave WiFi de fábrica:** el bloqueo de configuración queda desactivado (`FWM_FORCE_AP_PASS_CHANGE=0`). La
  clave puede usarse; el cambio sigue disponible en la WebUI (sección siempre visible) y por MAVLink.
- **F-05:** `canWriteConfig()` reemplaza a `isOnGround()` en las escrituras. Reutiliza `canChangeApMode()`:
  fail-closed tras haber tenido enlace con el FC, y permisivo en banco sin FC nunca enlazado.
- **F-11:** plataforma `espressif32@55.3.37` (la que produce el framework 3.3.7 que compila el proyecto),
  librerías con versión exacta, `requirements-dev.txt` con `==` y `platformio==6.2.0` en los workflows.
  Primer intento con `espressif32@6.12.0`: resolvía el framework 2.0.17 y se descartó.

## 3. Hallazgos de validación

1. **FW-09:** `pio test` no funciona; no debe presentarse como cobertura.
2. **L1-4:** `follow_law_test.py` necesita un SITL en marcha; debe ejecutarse dentro de `lab.py`.
3. **Incidencia de entorno:** tras cambiar `platformio.ini` (y en un intento con la plataforma equivocada) hubo
   que recompilar; uno de los intentos falló con `FileExistsError` de PlatformIO al reinstalar el framework.
   Recompilar tras fijar la plataforma correcta resolvió el problema. Los artefactos de `.pio/` son regenerables.

## 4. Historial y repositorio

- Reescritura con `git filter-repo` y mapeo `[email antiguo]` → `Amigache@users.noreply.github.com`
  (autor y committer). Comprobado: ninguna aparición del email antiguo en el historial, el árbol y las referencias.
- Bundle de respaldo previo a la reescritura en `%LOCALAPPDATA%\Temp\opencode\backup\`; contiene el email
  antiguo y debe borrarse.
- Push forzado de `develop` y `main` a `origin` (ver §5).

## 5. Criterio para la release

Antes de crear el primer tag `v*`:

- [ ] Verificar en placa FW-11 (66 checks), PW-01 y F05-01.
- [ ] Cerrar F-08: reflashear COMx/COMx con `ttgo-lora32-v1-flight` y comprobar `SIMCFG ERR`.
- [ ] Repetir L4 (`tools/bench_suite.py`) con el firmware nuevo.
- [ ] Probar el flasheador web en Chrome/Edge contra una placa (instalar y asignar rol).
- [ ] Activar en GitHub los ajustes de la auditoría §6 (incluida la fuente de Pages).

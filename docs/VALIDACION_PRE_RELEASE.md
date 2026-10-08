# Validación pre-release — FlyWithMe

Fecha: 2026-10-08 · Rama `develop` (el commit de publicación es el último de `develop`).
Entorno: Windows, PlatformIO Core 6.2.0, Python 3.13.2, Node 24.12 / npm 11.6.2, git 2.55.

**Sin hardware conectado, sin ArduPlane SITL y sin navegador con Web Serial.** Las pruebas en placa y en
SITL de los cambios de firmware no se han podido ejecutar aquí. En Windows no hay compilador de host:
las pruebas `native` se han ejecutado con `ziglang` (clang 21) expuesto como `gcc`/`g++`/`ar` solo en esta
sesión. En CI se usa el `gcc` del runner Ubuntu.

Criterios de niveles: `docs/PLAN_PRUEBAS_SISTEMA.md` §2.

## 1. Resultados

| ID | Nivel | Prueba | Comando | Resultado |
|---|---|---|---|---|
| FW-01 | L0 | Build producción | `pio run -e ttgo-lora32-v1-flight` | **PASS** · `firmware.bin` 1 164 544 B · `espressif32@55.3.37` · framework 3.3.7 |
| FW-02 | L0 | Build dev/HIL | `pio run -e ttgo-lora32-v1-sitl` | **PASS** · mismas versiones |
| FW-10 | L0 | `default_envs` | `pio project config` | **PASS** · `default_envs = ttgo-lora32-v1-flight` |
| F11-01 | L0 | Versiones fijadas | log de build | **PASS** · GFX 1.12.6, SSD1306 2.5.17, BusIO 1.17.4, ArduinoLog 1.1.1, LoRa 0.8.0, ESPAsyncWebServer 3.12.1, AsyncTCP 3.5.0 |
| F12-01 | L0 | Acciones fijadas por SHA | `grep` sobre `.github/workflows/*.yml` | **PASS** · todas las `uses:` tienen SHA de 40 caracteres con comentario de versión |
| F12-02 | L0 | Sintaxis de workflows y Dependabot | `yaml.safe_load` sobre `.github/**/*.yml` | **PASS** |
| L0-3 | L0 | Compilación de scripts Python | `python -m py_compile` sobre `tools/*.py` y `tools/tests/*.py` (16 ficheros) | **PASS** |
| L0-4 | L0 | Whitespace y conflictos | `git diff --check` | **PASS** |
| L0-6 | L0 | JavaScript de la WebUI del firmware | `node --check` sobre el `<script>` de `src/Web.cpp` | **PASS** |
| F13-01 | L1 | Pruebas unitarias de módulos puros | `pio test -e native` (`native@1.2.1`, Unity 2.6.1) | **PASS** · 12/12 contra `src/protocol.h`, `status_text.h`, `wifi_identity.h`, `selftest.h` |
| CLI-02 | L1 | Bridge serie, DTR/RTS antes de `open()` | `python -m unittest discover -s tools/tests -p "test_*.py"` | **PASS** 1/1 |
| CLI-01a | L1 | Matriz de protocolo | `python tools/proto_sim.py` | **PASS** 17/17 |
| CLI-01b | L1 | Predictor y offsets | `python tools/follow_sim_test.py` | **PASS** 8/8 |
| L1-4 | L1 | Ley de guiado contra SITL | `python tools/follow_law_test.py` | **No ejecutable aquí**: necesita SITL en TCP 5760/5770 |
| FW-11 | L1 | Self-test en placa (protocolo + validador de clave) | se ejecuta al arrancar | **Compilado y verificado en host** (check 11 vía `pio test -e native`). Pendiente en placa: `SELFTEST PASS` con **66 checks** |
| REL-01 | L2 | Bundle de release `flight` | `python tools/package_firmware_release.py --version v0.0.0-test --commit final --output <tmp>` | **PASS** · 742 802 B · SHA-256 `580bfacb…` |
| WEB-01 | L2 | Flasheador web | `python tools/build_web_flasher.py --version v0.0.0-test --commit final --output <tmp>` | **PASS** · 4 particiones en `0x1000`, `0x8000`, `0xE000`, `0x10000` |
| WEB-02 | L2 | Instalación desde el navegador | `web/index.html` con ESP Web Tools 10.4.0 | **No validado** con placa |
| WEB-03 | L2 | Asignación de rol por Web Serial | panel «Asignar rol» | **No validado** con placa ni navegador |
| PW-01 | L2 | Cambio de clave desde la WebUI y MAVLink | `POST /api/ap/pass`, `WIFI_CONFIG_AP` | **Compilado**. Pendiente en placa; MAVLink no validado con Mission Planner |
| PW-02 | L2 | Bloqueo opcional con clave de fábrica | `FWM_FORCE_AP_PASS_CHANGE=0` (por defecto) | **No aplica** por decisión |
| F05-01 | L2 | Escrituras fail-closed (`canWriteConfig`) | `groundOnly`, `/api/config`, `PARAM_SET` | **Compilado**. Pendiente en placa: enlazado y luego desconectado en vuelo → denegado; banco sin FC nunca enlazado → permitido |
| CI-01 | L2 | Workflows en GitHub | `ci.yml` en `push` y `pull_request` | **Pendiente**: se verifica al primer run tras el push |
| FLASH-01 | L2 | Carga y provisión por script | `tools/flash_firmware.py` | **No ejecutado**: requiere placa |
| L3 | L3 | SITL sin placas | `python tools/lab.py` | **No ejecutado**: sin ArduPlane |
| L4 | L4 | HIL completo | `tools/bench_suite.py` | **No repetido**. Último reporte documentado: `20261008-211545`, PASS 20/20. Debe repetirse con el firmware nuevo |
| L5 | L5 | Distribución y FC real | — | **Pendiente** |

## 2. Cambios de esta ronda

- **F-12 (CI):** `permissions` por job (`contents: read` por defecto; `contents: write` solo en `publish`);
  acciones fijadas por SHA; `persist-credentials: false`; `ci.yml` nuevo con compilación de ambos perfiles,
  `pio test -e native`, comprobación del flasheador y pruebas Python; `dependabot.yml` para acciones y pip.
  Los SHA se obtuvieron de la API pública de GitHub en esta sesión.
- **F-13 (pruebas):** `test/test_main.cpp` reescrito (12 pruebas contra las cabeceras reales) y
  `[env:native]` con `platform = native@1.2.1`. Se eliminaron los helpers duplicados. El límite se
  documenta: la lógica de `Comm`/`Telem` no está cubierta.

## 3. Hallazgos de validación

1. **Pruebas `native` en esta máquina:** requieren un compilador C/C++ en el PATH. En Windows, sin él, la
   compilación falla con `gcc` no reconocido. Documentado en `DEVELOP.md`.
2. **Cobertura parcial:** `pio test -e native` no prueba la validación de tramas de `Comm`, la predicción ni
   la distancia de `Telem`, porque dependen de Arduino. Queda como trabajo pendiente (F-13 §7 de la auditoría).
3. **Incidencia de entorno (anterior):** el directorio `.pio/build/ttgo-lora32-v1-flight` quedó sin binarios
   tras cambiar `platformio.ini`. Se regeneró con `pio run`; los artefactos de `.pio/` son regenerables.

## 4. Historial y repositorio

- Historial reescrito con `git filter-repo` (autor y committer noreply). Sin apariciones de `[dominio antiguo]` en el
  historial, el árbol ni las referencias.
- Push forzado de `develop` y `main` ya realizado (`da945b8` → `f0f93b8` en `develop`; `85b924a` → `71b98bb`
  en `main`). Los commits de esta ronda se añaden a `develop` con un push normal.
- Bundle de respaldo previo a la reescritura en `%LOCALAPPDATA%\Temp\opencode\backup\`; contiene el email
  antiguo y debe borrarse.

## 5. Criterio para la release

Antes de crear el primer tag `v*`:

- [ ] CI verde en GitHub (`ci.yml`) tras el push.
- [ ] Verificar en placa FW-11 (66 checks), PW-01 y F05-01.
- [ ] Cerrar F-08: reflashear COMx/COMx con `ttgo-lora32-v1-flight` y comprobar `SIMCFG ERR`.
- [ ] Repetir L4 (`tools/bench_suite.py`) con el firmware nuevo.
- [ ] Probar el flasheador web en Chrome/Edge contra una placa (instalar y asignar rol).
- [ ] Activar en GitHub los ajustes de la auditoría §6 (incluida la fuente de Pages).

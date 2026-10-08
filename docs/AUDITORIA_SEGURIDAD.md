# Auditoría de seguridad y de repositorio — FlyWithMe

Fecha: 2026-10-08 · Rama auditada: `develop` · Historial reescrito (autor noreply); el commit de publicación
es el último de `develop` (`git log -1`).
Alcance: firmware (`src/`), herramientas (`tools/`), CI (`.github/`), configuración (`platformio.ini`,
`.gitignore`), documentación y **historial de git completo**.

## 1. Resumen ejecutivo

- **No hay secretos en el repositorio ni en su historial.** Las coincidencias de `password`/`token`/`secret`
  son cabeceras MAVLink generadas (`lib/mavlink/`).
- **Clave WiFi de fábrica (F-01): riesgo aceptado y documentado.** La clave `12345678` es pública y común
  a todas las placas, pero es válida. Se puede cambiar en la WebUI o por MAVLink; no se obliga a cambiarla.
- **WebUI y API sin autenticación (F-02): aceptado temporalmente.** El sistema está en pruebas; se
  securizará más adelante.
- **Enlace LoRa sin autenticación (F-03): riesgo documentado** en el README y en `SECURITY.md`.
- **Escrituras de configuración fail-closed (F-05): corregido.** Sin enlace con el FC ya no se considera
  «tierra» para editar parámetros.
- **Build reproducible (F-11): corregido.** Plataforma, framework y librerías fijadas; PlatformIO y las
  herramientas Python de desarrollo también.
- **Configuración de build (F-04): corregido.** `pio run` solo construye el perfil de vuelo.
- **CI y pruebas (F-12, F-13): corregidos.** Workflows con permisos mínimos y acciones fijadas por SHA;
  `ci.yml` compila ambos perfiles y ejecuta `pio test -e native` y las pruebas Python en cada push y PR.
- **Historial reescrito (F-14): corregido**, con push forzado de `develop` y `main`.

## 2. Escala

| Severidad | Significado |
|---|---|
| **Alta** | Puede afectar a la seguridad de vuelo o permitir el control de una placa por un tercero. |
| **Media** | Expone datos, facilita un ataque combinado o rompe una garantía declarada. |
| **Baja** | Higiene, reproducibilidad o endurecimiento. |
| **Info** | Observación sin acción inmediata. |

Estados: **Corregido** (implementado y compilado), **Aceptado** (decisión explícita de no corregir por ahora),
**Documentado** (riesgo conocido comunicado al usuario), **Abierto** (pendiente).

## 3. Hallazgos

| ID | Sev. | Estado | Hallazgo | Ubicación |
|---|---|---|---|---|
| F-01 | Alta | **Aceptado** | Clave WiFi de fábrica (`12345678`) pública e igual en todas las placas. Se mantiene como válida por decisión (algunas placas no tienen OLED). Hay cambio de clave en la WebUI (`/api/ap/pass`) y por MAVLink (`WIFI_CONFIG_AP`). Bloqueo opcional con `FWM_FORCE_AP_PASS_CHANGE`, desactivado. | `src/config.h`, `src/wifi_identity.h`, `src/FWM.cpp` (`setApPassphrase`), `src/Web.cpp`, `src/Telem.cpp` (`handle_wifi_config_ap`) |
| F-02 | Alta | **Aceptado** | WebUI y API REST sin autenticación: `/api/params`, `/api/config`, `/api/ap`, `/api/logs`, `/api/stats`. Sistema en pruebas. | `src/Web.cpp` |
| F-03 | Alta | **Documentado** | Protocolo LoRa sin autenticación ni cifrado. El checksum es una suma de bytes y el `netid` es un filtro público. Un atacante puede inyectar posiciones y modos falsos del líder. | `src/protocol.h`, `src/Comm.cpp`, README («Límites importantes») y `SECURITY.md` |
| F-04 | Alta | **Corregido** | `pio run` y `pio run -t upload` procesaban todos los entornos, incluido `ttgo-lora32-v1` con `FC_EMULATION=1`. Añadido `[platformio] default_envs = ttgo-lora32-v1-flight`. | `platformio.ini` |
| F-05 | Media | **Corregido** | `isOnGround()` falla abierto: sin enlace con el FC devolvía «tierra» y `groundOnly` permitía editar parámetros en vuelo. Nuevo permiso `canWriteConfig()` (fail-closed, basado en `canChangeApMode()`) para `groundOnly`, `/api/config` y `PARAM_SET`. Se mantiene el comportamiento en banco sin FC nunca enlazado. `isOnGround()` queda solo para informar en `/api/stats`. | `src/FWM.cpp` (`canWriteConfig`, `setParamByKey`), `src/Telem.cpp` (`PARAM_SET`), `src/Web.cpp` (`/api/config`) |
| F-06 | Media | **Corregido** | La clave del AP se escribía en el log serie en cada arranque. Eliminado; la OLED sigue mostrando la clave en modo AP (diseño original). | `src/Web.cpp` (`startAP`) |
| F-07 | Media | **Aceptado** | `/api/logs` descarga `flight.log` sin autenticación (contiene lat/lon). Se trata junto con F-02. | `src/Web.cpp` |
| F-08 | Media | Abierto | Según `docs/PLAN_PRUEBAS_SISTEMA.md`, COMx/COMx quedaron con la imagen dev/HIL. Hay que reflashear con `ttgo-lora32-v1-flight` y comprobar `SIMCFG ERR` antes de volar. | `docs/PLAN_PRUEBAS_SISTEMA.md` |
| F-09 | Baja | **Corregido** | Eliminado el código legacy (`#if !USE_WEB_SERVER`) con `POST /save` sin autenticación, `getPostParam`/`urlDecode` y el servidor TCP antiguo. | `src/Web.cpp`, `src/Web.h` |
| F-10 | Baja | Abierto | Puertos COM, coordenadas del campo de pruebas y rutas de entorno hardcodeados en herramientas. | `tools/bench_restart.ps1`, `tools/sitl_start.ps1`, `platformio.ini`, `tools/lab.py` |
| F-11 | Baja | **Corregido** | Versiones fijadas: `platform = espressif32@55.3.37` (framework Arduino 3.3.7, el mismo que compila el proyecto), `lib_deps` con versión exacta (incluida Adafruit BusIO), `requirements-dev.txt` con `==` y `platformio==6.2.0` en los workflows. Verificado: ambos perfiles compilan con las mismas versiones. | `platformio.ini`, `requirements-dev.txt`, `.github/workflows/*.yml` |
| F-12 | Baja | **Corregido** | Permisos mínimos por job (`contents: read` por defecto; `contents: write` solo en la publicación de la release). Acciones fijadas por SHA de commit con su versión en comentario. `persist-credentials: false` en los checkouts. Dependabot para acciones y pip. Nuevo `ci.yml` que compila ambos perfiles, ejecuta `pio test -e native` y las pruebas Python en cada push y PR. | `.github/workflows/*.yml`, `.github/dependabot.yml` |
| F-13 | Baja | **Corregido** | `pio test -e native` existe y pasa (12/12) enlazando con las cabeceras reales de `src/` (protocolo, STATUSTEXT, identidad WiFi, self-test). Se eliminaron los helpers duplicados. Límite documentado: la lógica de `Comm`/`Telem` depende de Arduino y no está cubierta por este entorno. | `test/test_main.cpp`, `platformio.ini` (`[env:native]`) |
| F-14 | Info | **Corregido** | El email personal del autor aparecía en todos los commits. Historial reescrito con `git filter-repo` y mapeo al noreply de GitHub; push forzado a `origin`. | `git log --all` |
| F-15 | Info | **Corregido** | El README indicaba un cambio de clave WiFi desde la WebUI que no existía. Corregido y ampliado con el flujo real. | `README.md` |
| F-16 | Info | **Corregido** | No existía `SECURITY.md`. | `SECURITY.md` |
| F-17 | Info | **Corregido** | Salidas de release y del flasheador podían entrar en el repo. | `.gitignore` (`site/`, `release-assets/`) |

## 4. Decisiones tomadas

- **F-01 (clave WiFi).** La clave de fábrica es válida y se puede usar. No se bloquea la configuración por
  tenerla. El cambio está disponible en la WebUI y por MAVLink (8–63 caracteres ASCII imprimibles, distinta de
  la de fábrica, solo en tierra, reinicio tras guardar). Si más adelante se quiere obligar al cambio, basta con
  `FWM_FORCE_AP_PASS_CHANGE=1`.
- **F-02 y F-07 (WebUI, API y logs).** Aceptados temporalmente: es un sistema en pruebas.
- **F-03 (LoRa).** Riesgo documentado. Un protocolo con MAC y clave compartida queda como trabajo futuro, con
  validación HIL y compatibilidad con v2.
- **F-14 (historial).** Reescrito. Autor y committer: `Amigache <Amigache@users.noreply.github.com>`.

## 5. Lo que está bien

- **Sin secretos** en el árbol de trabajo ni en el historial.
- **Perfil de producción bloqueado:** `ttgo-lora32-v1-flight` fija `FC_EMULATION=0`, `FC_LINK_USB=0` y
  `FWM_ALLOW_RUNTIME_SITL=0`.
- **AP solo en tierra:** `updateApGate()` y `canChangeApMode()` apagan el AP en vuelo.
- **Escritura protegida:** rol y modo AP exigen `canChangeRole()`/`canChangeApMode()`; la configuración exige
  `canWriteConfig()`, también por MAVLink.
- **Validación de tramas LoRa:** versión, netid, tipo, checksum, rangos GPS/altitud/velocidad, anti-replay por
  `seq` modular y filtro de sysid.
- **Liberación reproducible:** `tools/package_firmware_release.py` publica SHA-256 y solo empaqueta `flight`.
- **Herramientas:** sin `shell=True` ni `eval` sobre datos externos.
- **Licencia y repositorio:** GPLv3; `.gitignore` excluye `.pio/`, `tools/reports/`, `__pycache__`.

## 6. Pendiente antes de la release

1. **Verificar en placa:** cambio de clave WiFi (WebUI y MAVLink), reinicio, y el bloqueo de escrituras
   en vuelo (`canWriteConfig`) con el FC desconectado tras haber enlazado.
2. **F-08:** reflashear COMx/COMx con `ttgo-lora32-v1-flight` y comprobar `SIMCFG ERR`.
3. **Ajustes de GitHub** (no se pueden aplicar desde el repositorio local):
   - *Settings → Code security:* activar *Secret scanning* con *Push protection*, *Dependabot alerts* y
     *Private vulnerability reporting* (referenciado en `SECURITY.md`).
   - *Settings → Actions → General:* *Workflow permissions* en **Read repository contents**.
   - *Settings → Rules:* PR obligatorio en `main` y ruleset sobre `refs/tags/v*`.
   - *Settings → Pages:* *Source* = **GitHub Actions** (necesario para el flasheador web).
4. **Limpieza local:** borrar el bundle de respaldo previo a la reescritura
   (`%LOCALAPPDATA%\Temp\opencode\backup\flywithme-pre-rewrite.bundle`), que contiene el email antiguo.

## 7. Endurecimiento posterior (no bloqueante)

- **F-02 / F-07:** autenticación de la WebUI y de `/api/logs`.
- **F-10:** parametrizar puertos y coordenadas (variables de entorno o un fichero local ignorado por git).
- **Cobertura de `Comm`/`Telem`:** la validación de tramas, la predicción y la distancia no tienen pruebas
  en host (dependen de Arduino). Extraer su lógica pura a cabeceras permitiría probarla en `native`.

Ver `docs/VALIDACION_PRE_RELEASE.md` para los resultados de compilación, pruebas y del flasheador web.

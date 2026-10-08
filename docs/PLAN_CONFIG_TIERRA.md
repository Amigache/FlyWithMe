# Plan — Configuración en TIERRA (AP/WiFi solo en tierra + parámetros FWM)

> Documento de planificación. **No implementado todavía.** Abordar en otra sesión.
> Al implementar cada punto, marcar la casilla y actualizar `docs/ROADMAP.md` y `AGENTS.md`.

---

## 1. Objetivo y decisiones

**Objetivo:** que el AP/WiFi del ESP32 sea **exclusivamente una herramienta de configuración en
tierra** para FWM, y que sus parámetros se puedan **leer/descargar y modificar** cómodamente
(similar a Mission Planner con el autopiloto).

**Decisiones (acordadas):**
- El AP/WiFi **no** se usa en vuelo. Nunca telemetría en vivo ni cambios de parámetros en vuelo.
- El AP se levanta **solo en tierra** (o cuando no hay FC, para banco/montaje).
- La web sirve para **configurar FWM**, no para operar el vuelo.
- Se **bloquean** los cambios en vuelo (aunque por error el AP estuviera activo).
- Todo lo configurable se **persiste en NVS**.
- Objetivo "tipo Mission Planner" (fase opcional): exponer los parámetros FWM por **MAVLink**.

---

## 2. Estado actual (punto de partida)

- `Web::startAP()` (`src/Web.cpp`) hace `WiFi.mode(WIFI_AP)` + `WiFi.softAP(params.ssid, params.pass)`.
- Se llama desde `FWM::begin()` si `WEB_START_AP_IMMEDIATELY=1` y desde `Web::run()` al detectar
  `mav->linkTimeout` (sin FC) — ese 2º caso ya encaja con "config en tierra/banco".
- `Params_t` (`src/config.h`) + `Preferences` (`FWM::loadParams/saveParams`): `foll_enable`,
  `foll_ofs_type`, `foll_alt_type`, `link_timeout`, `ssid`, `pass`.
- Config en caliente ya implementada y **validada**: `/api/config` (GET/POST) aplica `formation`,
  `prediction`, `filter`; `FWM::setFormation/setPrediction/setFilter`; persistencia en NVS; la web
  carga la config al abrir. AP: SSID `FWM XXXXXX` derivado de la MAC SoftAP / `http://192.168.4.1`.
- Hay un **WebSocket de telemetría** (`/ws`, puerto 81) — a revisar/retirar del flujo normal.
- Fix del cristal ya aplicado (`-DF_XTAL_MHZ=26`) → la WiFi funciona.

---

## 3. Fase 1 — Gate del AP: solo en tierra (+ override)

> ✅ **IMPLEMENTADO** y validado en banco (commit `bb97e60`). Resultados al final de esta sección.

- [x] **Detección "en tierra"**: función que combine estado del FC:
  - `!armed` **y** `groundspeed < WEB_AP_GS_MAX` (p. ej. 2 m/s) **y** opcionalmente `rel_alt` bajo.
  - Fuente: `APdata` (heartbeat/GLOBAL_POSITION_INT/VFR_HUD) ya recibidos por MAVLink.
- [x] **Lógica de arranque/parada**:
  - Si **no hay FC** (`linkTimeout` **o** `link` caído) → **AP ON** (configuración en banco/tierra).
  - Si hay FC: **AP ON** solo si "en tierra"; **AP OFF** si armado/en movimiento.
  - Parar el AP con `WiFi.softAPdisconnect(true)` (**sin** `WiFi.mode(WIFI_OFF)`; el teardown
    impedía re-levantarlo) y volver a levantarlo si se vuelve a tierra.
- [x] **Override manual** para forzar el AP en banco aunque el FC diga "armed":
  - Hecho con `#define WEB_AP_FORCE 1` en `config.h`. Botón/comando MAVLink: pendiente/opcional.
- [x] **Bloqueo de cambios en vuelo**: en `/api/config` (POST) se rechaza (HTTP 403) si "no en tierra".
- [x] `#define` nuevos en `config.h` + documentados en `AGENTS.md` §6:
  - `WEB_AP_GROUND_ONLY` (1), `WEB_AP_FORCE` (0), `WEB_AP_GS_MAX_CMS` (200),
    `WEB_AP_ALT_MAX_MM` (3000), `WEB_AP_GATE_INTERVAL_MS` (1000).
- **Criterios de aceptación**:
  - En banco (FC sin armar): AP levantado y accesible. ✅
  - Armado/moviéndose: AP **no** disponible; cambios rechazados. ✅
  - Sin FC (o link caído): AP levantado (config). ✅
  - Con `WEB_AP_FORCE=1`: AP forzado. (no re-probado, lógica trivial)

**Resultados (banco, SITL):** en tierra → `on_ground=true`, AP ON; armado/despegando → AP OFF
(desaparece `FWM AP 2` del escaneo); se mata el FC → `!link` → AP ON (ping OK). Nota: `lock_ap`
se pone a `true` al conectar el FC, así que **no** sirve `linkTimeout` como criterio; se usa
`mav->link`.

---

## 4. Fase 2 — Tabla de parámetros FWM única (web de configuración)

> ✅ **IMPLEMENTADO** (commit `31525e7`). Resultados al final.

- [x] **Tabla de parámetros** (fuente de verdad) en `config.h` (`ParamDef_t`):
  - Campos: `key`, `label`, `type` (int/float/bool/enum), `min`, `max`, `unit`, `groundOnly`.
  - Incluye: `formation`, `dist_offset`, `lateral_offset`, `vertical_offset`, `cross_gain`,
    `heading_corr_max`, `along_gain`, `prediction`, `filter`, `foll_enable`, `link_timeout`.
  - ⚠️ Pendiente: `ssid`/`pass` (strings) siguen fuera de la tabla (se manejan aparte).
- [x] **Acceso runtime**: los offsets/ganancias pasan a `Params_t` y `Telem` los usa
  (`fwm->params.*`); `getParamByIndex/setParamByIndex/paramsJson` en `FWM`. `setFormation/
  setPrediction/setFilter` unificados sobre la tabla.
- [x] **API**:
  - `GET /api/params` → lista completa (key, type, value, min/max, unit, groundOnly).
  - `POST /api/params` → set por clave (valida rango y "en tierra" → 403 en vuelo).
  - `GET/POST /api/config` se mantienen (alias).
- [x] **Página web** generada desde la tabla (num/checkbox/enum), dark y compacta (~4.8 KB),
  con badge "EN TIERRA/EN VUELO". Sin streaming continuo.
- [x] **Persistencia** en NVS (`saveParams` con los nuevos campos).
- [x] **WebSocket de telemetría retirado** del flujo normal (`WEB_TELEMETRY_WS=0`).
- **Criterios de aceptación**:
  - Descargar/editar/guardar cualquier parámetro desde la web. ✅
  - Los cambios se rechazan en vuelo. ✅ (mismo guard que Fase 1)

**Resultados (banco):** `GET /api/params` devuelve las 11 entradas con sus valores;
`POST` cambia y persiste (`dist_offset=110`, `cross_gain=0.7`…); la UI dark se sirve y muestra el
estado. Nota de test: PowerShell en locale ES envía `0,7` (coma) → usar cadenas con punto; el
navegador manda punto siempre.

---

## 5. Fase 3 — Parámetros FWM por MAVLink (tipo Mission Planner)

> ✅ **IMPLEMENTADO** (commit `dcbe5d3`). Resultados al final.

Objetivo: que **Mission Planner** liste los parámetros de FWM como los de cualquier autopiloto.

- [x] Presentarse como **componente MAVLink** propio: el ESP32 ya envía `HEARTBEAT` con
  (`SYSID`, `COMPID`=158); responde a los mensajes dirigidos a ese componente.
- [x] Protocolo de parámetros en `Telem` (`handle_param_message`):
  - `PARAM_REQUEST_LIST` → envía todos (`send_all_params`).
  - `PARAM_REQUEST_READ` → por `param_index` o por `param_id`.
  - `PARAM_SET` → aplica+persiste **solo en tierra** (si no, devuelve el valor sin cambios).
  - Codificación `PARAM_VALUE` (`MAV_PARAM_TYPE_REAL32`, `param_count`, `param_index`).
- [x] Mapeo tabla Fase 2 ↔ `PARAM_VALUE` (`paramCount/paramDefAt/getParamByIndex/setParamByIndex`).
- [x] Claves ≤15 chars (`heading_corr_max` → `hdg_corr_max`).
- [x] Probado en banco con un GCS (pymavlink como Mission Planner): listar, leer por id, set y
  set bloqueado en vuelo. (En MP real: componente `158` bajo el `SYSID` del FC.)
- **Criterios de aceptación**: el GCS lista y edita los parámetros FWM; los rechaza en vuelo. ✅

**Resultados (banco):** `PARAM_REQUEST_LIST` → las 11 entradas con sus valores; `PARAM_SET`
(`dist_offset=100`, `cross_gain=0.55`) aplica y confirma; en vuelo el `PARAM_SET` devuelve el valor
sin cambios (bloqueado). Flag `MAVLINK_PARAM_SERVER` (config.h).

---

## 6. Riesgos y consideraciones

- **Fiabilidad de "en tierra"**: si el FC falta o da estado erróneo → por defecto **AP ON sin FC**;
  con `WEB_AP_FORCE` como red de seguridad.
- **Seguridad**: AP con contraseña, solo en tierra; rechazar escrituras en vuelo.
- **Consumo/timing**: WiFi solo en tierra evita sorpresas (LoRa es 868 MHz, no compite en banda,
  pero sí suma consumo).
- **AP OFF al armar**: asegurar que no corta nada crítico (el web server async puede quedar sin
  radio; documentar reconexión).
- **Migración**: mantener compatibilidad con `/api/config` actual mientras se migra a `/api/params`.
- **NVS**: versionar/cuidar claves para no romper configs previas.

---

## 7. Preguntas abiertas

1. Trigger del override: ¿`#define`, botón físico, o comando MAVLink? (¿hay GPIO libre para botón?)
2. ¿Mantener el AP siempre accesible **en tierra** o levantarlo **bajo demanda** (p. ej. al pulsar
   un botón o al detectar "sin FC")?
3. ¿La web debe permitir también **descargar el log** (`/flight.log`) y exportar/importar config?
4. ¿Exponer también los parámetros FWM por MAVLink (Fase 3) o basta con la web?
5. Umbrales exactos de "en tierra" (groundspeed/altitud) y tiempo de histéresis.

---

## 8. Archivos previstos a tocar

- `src/config.h` — defines nuevos + tabla de parámetros.
- `src/FWM.h/.cpp` — gate del AP, load/save de la tabla, helpers get/set.
- `src/Web.h/.cpp` — `/api/params`, gate del AP, retirar WS de telemetría, UI generada.
- `src/Telem.h/.cpp` — (Fase 3) protocolo de parámetros MAVLink.
- `src/Screen.h/.cpp` — (opcional) mostrar estado AP / "en tierra".
- `AGENTS.md`, `docs/ROADMAP.md` — documentar flags y resultados.

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
  carga la config al abrir. AP: `FWM AP 2` / `http://192.168.4.1`.
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

- [ ] **Definir una tabla de parámetros** (fuente de verdad), p. ej. en `config.h`:
  - Campos: `clave`, `etiqueta`, `tipo` (int/float/bool/enum), `min/max`, `unidad`, `valor`,
    `persistente`, `solo-tierra`.
  - Incluir: formación, offsets (`DIST_OFFSET`, lateral/vertical), `prediction`, `filter`,
    `foll_enable/ofs_type/alt_type`, `link_timeout`, `ssid`, `pass`, velocidades/ganancias de guiado,
    etc.
- [ ] **Acceso**: `Params_t` como struct tipado + helpers `get/set/load/save` en `FWM`
  (mover ahí lo que hoy hace `setFormation/setPrediction/setFilter` para unificarlo).
- [ ] **API**:
  - `GET /api/params` → lista completa (clave, tipo, valor, min/max, unidad, solo-tierra).
  - `POST /api/params` → set por clave (valida rango y "en tierra").
  - Mantener `GET/POST /api/config` como alias/compatibilidad.
- [ ] **Página web** generada desde la tabla (controles según tipo), con botón "Guardar" y estado
  "en tierra / en vuelo". Sin streaming continuo.
- [ ] **Persistencia** en NVS de todos los parámetros marcados como persistentes.
- [ ] **Retirar el WebSocket de telemetría** del flujo normal (dejar, si acaso, solo en modo
  diagnóstico en tierra).
- **Criterios de aceptación**:
  - Descargar/editar/guardar cualquier parámetro desde la web y verificar que persiste tras reinicio.
  - Los cambios se rechazan en vuelo.

---

## 5. Fase 3 (opcional) — Parámetros FWM por MAVLink (tipo Mission Planner)

Objetivo: que **Mission Planner** liste los parámetros de FWM como los de cualquier autopiloto.

- [ ] Presentarse como **componente MAVLink** propio (ya enviamos MAVLink al FC; definir COMPID).
- [ ] Implementar el protocolo de parámetros en `Telem`:
  - Responder a `PARAM_REQUEST_LIST` y `PARAM_REQUEST_READ` con `PARAM_VALUE`.
  - Aceptar `PARAM_SET` (validando rango y "en tierra").
  - Codificación de nombres (≤16 chars) y de floats (param_id + param_value).
- [ ] Mapear la tabla de parámetros de la Fase 2 ↔ `PARAM_VALUE`.
- [ ] Probar con Mission Planner (conexión al FC): ver componente FWM, listar, editar, guardar.
- **Criterios de aceptación**: Mission Planner muestra y permite editar los parámetros FWM; los
  rechaza en vuelo.

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

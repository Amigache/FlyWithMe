# AGENTS.md — Coding agent guide (FlyWithMe)

This document describes the project, its architecture, conventions and commands. It is intended so
that any AI agent (or developer) can work safely and consistently. The project and its documentation
are written in **English**.

---

## 1. What FlyWithMe is

A **formation flight** system for aircraft running **ArduPlane** (PX4 not validated). A **leader**
aircraft transmits its position over a **LoRa** radio link; a **follower** aircraft receives that
data, computes a formation position and sends the resulting waypoint to its own autopilot over
**MAVLink** (`GUIDED` mode).

- **Target hardware:** TTGO LoRa32 V1 (ESP32 + SX1276), SSD1306 128x64 OLED, LoRa 866 MHz (Europe).
- **Framework:** Arduino on **PlatformIO**.
- A **single production firmware** for all boards (`ttgo-lora32-v1-flight`, UART1, `FC_EMULATION=0`,
  `FWM_ALLOW_RUNTIME_SITL=0`). The `OFF/FOLLOWER/LEADER` role is an NVS parameter, not a build flag.
  The dev profile `ttgo-lora32-v1-sitl` boots in flight mode but accepts `FWM SIM ON` per HIL
  session; a reset returns to UART1. The dev profile is never published in production releases.

---

## 2. Commands

Always run from the repository root. Requires PlatformIO (`pio`).

```bash
# Build a single image for real flight
pio run -e ttgo-lora32-v1-flight

# Universal dev/HIL profile (initially boots in flight mode; SIM only by command)
pio run -e ttgo-lora32-v1-sitl
pio run -e ttgo-lora32-v1

# Load the same binary and provision the individual role in NVS
python tools/flash_firmware.py --port COMx --role leader
python tools/flash_firmware.py --port COMy --role follower

# Dev/bench only: universal image with temporary FWM SIM ON for HIL
python tools/flash_firmware.py --environment ttgo-lora32-v1-sitl --port COMx --role leader
python tools/flash_firmware.py --environment ttgo-lora32-v1-sitl --port COMy --role follower

# Serial monitor (57600 baud; see note below)
pio device monitor

# Clean
pio run -t clean
```

- `COMx` is just PlatformIO's default port placeholder, not a role identity. CP210x adapters can
  reenumerate; identify each board and pass the port explicitly to the script. The SSID is
  `FWM XXXXXX`, derived from the last 3 bytes of the SoftAP MAC, not from the role. The board prints
  `FWM_ID ap_mac=... ap_ssid="..." role=... sysid=...` at boot and answers `FWM ID` over USB serial.
- `monitor_speed = 57600`. ⚠️ These boards (TTGO LoRa32 V1.0) carry a **26 MHz** crystal, so the
  firmware **must be compiled with `-DF_XTAL_MHZ=26`** (already in `platformio.ini`). That define
  makes the Arduino core pin the crystal and reconfigure the clocks in `app_main()`; without it the
  core assumes 40 MHz, the UART comes out ×0.65 (~37440) and **WiFi/BT fall out of band** (no AP,
  no scanning). With the flag, correct baud (57600) and working WiFi. LoRa is unaffected (the
  SX1276 has its own crystal). Do not confuse this with `monitor_speed`: with the crystal corrected,
  it is 57600.
- ⚠️ **On Windows, flashing requires UTF-8**: `tools/flash_firmware.py` sets `PYTHONIOENCODING=utf-8`
  for the PlatformIO process. The script also writes the role to NVS over USB serial; it requires
  `pyserial`. Later changes can be made on the ground from the WebUI/MAVLink and will reboot the
  board.
- **Environment compatibility:** built with **Arduino core 3.x / ESP-IDF 5**. Implications:
  - The async libraries are the maintained forks `esp32async/ESPAsyncWebServer` and
    `esp32async/AsyncTCP`. The original `me-no-dev/*` **fail to link** with core 3.x
    (`undefined reference to pxCurrentTCB`).
  - The watchdog uses the IDF ≥ 5 API (`esp_task_wdt_config_t` + `esp_task_wdt_reconfigure`) and
    keeps the IDF < 5 path under `ESP_IDF_VERSION_MAJOR`.
  - `Web.h` **must not** include `<WebServer.h>`: the project uses `AsyncWebServer`, and that Arduino
    header breaks with the `#undef F` that `config.h` does for MAVLink.
  - `config.h` includes `<string>` (used by `FlightModeInfo`).
  - **Partitioning:** `board_build.partitions = huge_app.csv` (app **3 MB** + SPIFFS 896 KB). The
    common firmware (~1.15 MB) uses ~**36.7 %**. With the default partition (`default.csv`, app
    1.25 MB) it would reach ~92 %; **do not change it** without checking the size with `pio run`.
  - Verified: builds `ttgo-lora32-v1-flight` and `ttgo-lora32-v1-sitl` → **SUCCESS**.
  - `ttgo-lora32-v1-flight` blocks `FWM SIM ON`; the dev profile accepts the command temporarily.
  - **OLED menu disabled** (`USE_INTERACTIVE_MENU 0`): its pins (12/13/14/15) collide with the
    MAVLink UART (12/13), LoRa RST (14) and OLED SCL (15). Do not re-enable without reassigning to
    free GPIOs.
  - **NVS keys are limited to 15 characters.** `heading_corr_max` was 16 and silently failed on
    every save/load (`KEY_TOO_LONG` / `NOT_FOUND`); it is now `hdg_corr_max`. Check the length
    before adding a key.
  - **LoRa compression:** `Comm::compressPacket` must use `floor()` for lat/lon. With truncation
    toward zero, **negative longitudes** (western hemisphere, e.g. Spain) were corrupted by ~100 km.
  - **LoRa:** the setters (`setSignalBandwidth`, etc.) go **after** `LoRa.begin()`; before that,
    `setLdoFlag()` divides by zero (registers not initialised).
  - **LoRa pins are `FWM_LORA_*`.** Do not reintroduce bare `SCK`/`MISO`/`MOSI`/`SS` macros: they
    collide by name with the `static const uint8_t` of the same name in `pins_arduino.h` and only
    compile because of include order (F-23).
  - **Board pin maps are selected with `-D FWM_BOARD=...`** (`FWM_BOARD_V1` default, `FWM_BOARD_V21`
    for the V1.6/V2.0/V2.1.6). A wrong map compiles fine and leaves the radio mute, so never rely on
    the Arduino variant macros: `ttgo-lora32-v2` ships `LORA_RST 12` with the comment `// GPIO14` and
    both are wrong (it is **23**). See `docs/PROJECT.md` §2.3 and `DEVELOP.md` for the table.
  - **A leader must have a working radio.** `Comm::begin()` sets `radioHealthy`; `canBecomeLeader()`
    refuses the role when it is false and `FWM_ID` reports `radio=ok|fail`. Never spin forever on a
    failed `LoRa.begin()`: the watchdog turns it into a reboot loop (F-22).
  - **Route order in `src/Web.cpp`:** ESPAsyncWebServer matches routes registered without a
    wildcard with the `BackwardCompatible` predicate
    `(url == ruta) || url.startsWith(ruta + "/")` (`WebServer.cpp:336`), and `_attachHandler` serves
    the **first** match. Therefore `/api/ap/pass` must be registered **before** `/api/ap`, or the
    latter swallows it. This caused a real bug in `v1.0.0` where the WiFi password could not be
    changed. Keep the ordering and its explanatory comment.

### Platform pin

The platform is pinned by URL to the pioarduino fork release `55.03.37`:

```
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.37/platform-espressif32.zip
```

`espressif32@55.3.37` does **not** work: that version does not exist in the official registry, which
only publishes 1.x–7.x (Arduino core 2.x).

### Tests

```bash
pio test -e native
python -m unittest discover -s tools/tests -p "test_*.py" -v
python tools/proto_sim.py
python tools/follow_sim_test.py
```

- **Emulated FC (SITL):** universal dev image `ttgo-lora32-v1-sitl` + `FWM SIM ON` over USB,
  `tools/sitl_bridge.py` (serial↔TCP bridge). `tools/lab.py --firmware` enables SIM at start and
  resets the boards on stop; production only distributes `ttgo-lora32-v1-flight`.
- **HIL harness:** `tools/hil_test.py` validates the leader/follower link over serial
  (`FC_EMULATION=1`).
- **Lab (CLI):** `python tools/lab.py` — interactive menu that brings up **2 SITL** (ArduPlane) and a
  **headless MAVProxy** that merges both vehicles into a single **UDP** stream so Mission Planner sees
  them with one connection. With `--firmware` it also connects the boards (serial bridges). Actions:
  takeoff, GUIDED and benchmarks (straight/turn) with a **report** in `tools/reports/`; option **9**
  runs the **full bench** (`tools/bench_suite.py`: link, params, formations, takeoff, straight,
  turn, safety + CSV per scenario).
  `tools/sitl_bridge.py` joins SITL↔board and, with `--tap-port N`, gives a **direct MAVLink link to
  the board** over the same USB: the bench reads/writes FWM parameters (component 158), provisions
  the role and changes the formation **without WiFi**. `lab.py` reboots the boards (DTR/RTS) when
  bringing up the bench.
- `tools/mp_launch.py` starts MAVProxy **without wxPython** on Windows (`has_wxpython=False` +
  dummy `rline`) and applies a **pymavlink patch** (UDP outputs bound the destination port and
  collide with Mission Planner on the same PC → changed to an ephemeral bind).

> ⚠️ `bench_suite.py --start-bench --stop-bench-after --firmware` is the supported way to run the
> full bench. The interactive `lab.py` bridges die when stdin closes.
>
> **State:** resolved. `[env:native]` builds `test/test_main.cpp` against the pure headers in `src/`
> (protocol, STATUSTEXT, WiFi identity and self-test). Run it with `pio test -e native`; CI runs it
> on every push and PR. `Comm`/`Telem` logic depends on Arduino and is not covered by this
> environment.

---

## 3. Repository structure

```
src/
  main.cpp     Entry point (setup/loop). Global `FWM fwm` instance.
  FWM.h/.cpp   Orchestrator: watchdog, state machine, parameters, send ticker.
  Comm.h/.cpp  LoRa: send/receive, checksum, compression, retry, auto-calibration.
  Telem.h/.cpp MAVLink with the autopilot: heartbeat, waypoints, distance, prediction/formation.
  Screen.h/.cpp OLED display + interactive button menu.
  Web.h/.cpp   Async web server + REST API + telemetry WebSocket.
  config.h     ALL configuration, constants, structs, enums and the Logger class.
  protocol.h   Pure, host-testable v2 wire format.
  lora_band.h  Pure, host-testable LoRa band table (index -> frequency, labels).
  selftest.h   Pure, host-testable boot self-test.
  status_text.h Pure, host-testable STATUSTEXT deduplication.
  wifi_identity.h Pure, host-testable SSID derivation and WPA2 passphrase validation.
lib/mavlink/   Generated MAVLink headers (v2). DO NOT edit by hand.
test/          test_main.cpp (Unity) — 15 tests against the pure headers.
include/ lib/  Standard PlatformIO folders (README).
docs/          PROJECT.md (consolidated reference).
```

Documentation: `README.md` (user manual), `DEVELOP.md` (development, SITL and testing), `AGENTS.md`
(this guide), `SECURITY.md` (vulnerability reporting) and `docs/PROJECT.md` (audit, validation,
testing strategy, roadmap).

---

## 4. Data flow

```
[LEADER]   autopilot --MAVLink--> Telem --(LoraPacket)--> Comm --> LoRa --> air
[FOLLOWER] air --> LoRa --> Comm --> (validate/decompress) -->
           Telem --> (prediction + formation + filter) --> nav_waypoint --> autopilot
```

- The **leader** sends packets with a periodic `Ticker` (`send_packet_ticker`).
- The **follower** only follows when in `FOLL_MODE_FOLLOWER` and the autopilot is in `MODE_GUIDED`.
- The state machine (`FWM::transitionState`) is synchronised with `stage_follow` and the beacon.

---

## 5. Code conventions

- **C++17** (`-std=c++17`), Arduino style.
- Classes with access to `FWM*` and, where applicable, `static ClassName* self` + a static `Ticker`
  callback (e.g. `FWM::send_packet_ticker_callback`).
- **Mixed inherited naming**: methods and functions in `snake_case` (`nav_waypoint`, `status_text`,
  `sendPacket`), members in `camelCase`/`snake_case` (`commData`, `follow_mode`, `stage_follow`).
  Keep the style of the file being edited.
- **Feature flags** via `#define` in `config.h` (e.g. `USE_COMPRESSED_PACKETS`, `ADAPTIVE_RATE`,
  `USE_PREDICTION`, `USE_POSITION_FILTER`, `USE_WEB_SERVER`, `USE_INTERACTIVE_MENU`,
  `SIMULATION_MODE`). Use `#if` to enable/disable blocks.
- **Runtime role**: `params.role` is stored in NVS; `FWM::fwmSystemId()`, `targetSystemId()` and
  `peerSystemId()` derive the MAVLink/LoRa identity from that parameter. Do not reintroduce
  compilation macros that generate different images for leader and follower.
- **Logging**:
  - `Log.*` (ArduinoLog) → serial console (requires `DEBUG_MODE`).
  - `logger->info/debug/warning/...` → persistent CSV log in SPIFFS (`/flight.log`).
  - Own levels `FWM_LOG_LEVEL_*` (prefix `FWM_` to avoid clashing with ArduinoLog).
  - ⚠️ **ArduinoLog does not support `%f`**: do not use `%f`/`%.2f` in `Log.*` (it prints garbage
    like `2f`). Format as a scaled integer, e.g. `Log.notice("d=%dm", (int)distance)`. C `snprintf`
    **does** support floats.
- **Errors**: prefer `isSafeToFollow()` and `validatePacket()` before applying a waypoint. Never
  follow with invalid GPS data.

---

## 6. Key configuration (`src/config.h`)

| Define | Default | Description |
|---|---|---|
| `LORA_BAND` | `FWM_LORA_FREQ_868` | Compile-time default frequency, used on boards with no NVS value. Must be an integer literal in Hz (not `866E6`) so `#if` comparisons work. |
| `LORA_SPREADING_FACTOR` | `12` | LoRa SF. |
| `LORA_TX_POWER` | `20` | dBm. |
| `F_XTAL_MHZ` (build flag) | `26` | TTGO LoRa32 V1.0 crystal. **Essential** (`-DF_XTAL_MHZ=26`): without it the core assumes 40 MHz → UART ×0.65 and **dead WiFi/BT**. |
| `USE_COMPRESSED_PACKETS` | `0` | The v2 protocol uses a packed 40 B frame; the old compression does not yet include all v2 fields. |
| `ADAPTIVE_RATE` | `1` | TX rate by distance (2 s / 1 s / 0.5 s). |
| `MAX_LORA_RETRIES` | `3` | Retries with backoff on send. |
| `USE_PREDICTION` / `PREDICTION_TIME_MS` | `1` / `1000` | Leader prediction; in TRAIL the lead is capped by `PREDICTION_MAX_LEAD_FRACTION`. |
| `PREDICTION_MAX_LEAD_FRACTION` | `0.25` | Cap on the predictive lead as a fraction of the TRAIL offset: avoids cancelling short offsets and pulling the target toward the leader. |
| `USE_POSITION_FILTER` / `POSITION_FILTER_ALPHA` | `1` / `0.3` | Low-pass position filter. |
| `DEFAULT_FORMATION` | `0` (TRAIL) | 0=TRAIL, 1=LEFT, 2=RIGHT, 3=ABOVE, 4=BELOW. |
| `USE_HEADING_GUIDANCE` | `1` | 1 = cross-track heading guidance (`GUIDED_CHANGE_*`); 0 = `DO_REPOSITION` (carrot). |
| `CROSS_TRACK_GAIN_DEG_PER_M` / `MAX_HEADING_CORR_DEG` | `0.6` / `40` | Heading correction per lateral error (degrees per metre / cap). |
| `ALONG_GAIN_CMS_PER_M` / `MAX_SPEED_SLOW` | `12` / `400` | Speed correction per longitudinal error (cm/s per m / max braking). |
| `GUIDED_ALT_REFRESH_MS` | `2000` | How often `DO_REPOSITION` is sent to pin the altitude (`next_WP_loc`). |
| `GUIDED_AIRSPEED_MIN` / `GUIDED_AIRSPEED_MAX` | `10` / `30` | Commanded airspeed limits (m/s). |
| `MAX_FOLLOW_DISTANCE` | `5000` | m — safety limit. |
| `HEAD_ON_GUARD` | `0` in `config.h`/flight; `1` in the dev/HIL profile | 1 = **head-on collision guard**: if the leader comes head-on and < `HEAD_ON_RANGE` (500 m), break perpendicular to the line of sight and brake. The flight profile keeps `0`; the SITL dev profile enables it to validate the scenario. |
| `TIGHT_FORMATION` | `0` | 1 = **fast LoRa rate when close** (200/500/1000 ms) for 10–20 m flight. Pair with a low SF (`-D LORA_SPREADING_FACTOR=7`). |
| `netid` | `4660` (0x1234) | v2 protocol network: filters traffic from other systems **at packet level**. Must match on both; configurable by parameter and WebUI. Distance/OSD are only published if the packet indicates a valid position. |
| `band` NVS/WebUI/MAVLink/USB | `FWM_DEFAULT_BAND` (868) | `0=433`, `1=868`, `2=915`. **Must match on both boards**; a mismatch means no link and no error. Ground-only (restarting the SX1276). Mapping and labels live in the pure header `src/lora_band.h`. USB console `FWM BAND 433\|868\|915`. |
| `role` NVS/WebUI/MAVLink | `OFF` on a new board | `0=OFF`, `1=FOLLOWER`, `2=LEADER`; editable on the ground only, change with reboot. FWM/FC SYSID: leader 1, follower 2. Provision with `tools/flash_firmware.py`; older boards without this key must be reassigned. |
| `ap_mode` NVS/WebUI/REST/USB | `AUTO` | `0=AUTO`, `1=ON (ground only)`, `2=OFF`; the ground-only policy always prevails in flight. REST `/api/ap`; USB console `FWM AP auto|on|off`. |
| WiFi SSID | Derived from MAC | `FWM XXXXXX`, where `XXXXXX` are the last 3 bytes of the SoftAP MAC in hexadecimal; read over serial with `FWM_ID` at boot or the `FWM ID` command. Not manually configurable. |
| `approach_dist` | `300` | m — below this distance the follower **requires a stable leader mode** (FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED); above it, it still goes looking for it. |
| `FWM_DUAL_CORE` | `1` | **Core 1** owns the flight loop; **core 0** runs UI/log. The Ticker callback only sets a TX-pending flag; it does not touch SPI/LoRa. Validated in SITL. |
| `FOLLOWER_REPLY` | `1` | REPLY/JOIN only in slots requested by a BEACON; the leader opens an RX window with timeout, the follower responds from the single loop. Validated with loss/timeout/rejoin in SITL. |
| `NETID_USE_SYNCWORD` / `LORA_CRC` | `0` / `0` | Experimental RF options. Default: fixed known-good sync word and `netid`/checksum filtering at packet level. |
| `MIN_SAFE_ALTITUDE` | `50000` | mm (50 m) — minimum altitude. |
| `AUTO_CALIBRATE_LORA` | `0` | LoRa auto-calibration at startup. |
| `USE_INTERACTIVE_MENU` | `0` | OLED button menu. **Disabled** due to pin conflicts. |
| `USE_WEB_SERVER` / `USE_WEBSOCKET` | `1` / `1` | Async server (80) + WS (81). |
| `WEB_TELEMETRY_WS` | `0` | Live telemetry over WebSocket (removed from the normal flow; ground diagnostics only). |
| `MAVLINK_PARAM_SERVER` | `1` | The ESP32 answers `PARAM_REQUEST_LIST/READ/SET` as its own component (`SYSID`, `COMPID=158`) → FWM parameters visible/editable in Mission Planner. `PARAM_SET` is ground-only. |
| `STATUSTEXT` | exact dedupe | Unique `FWM:` prefix; identical text+severity are not resent. Distances use INFO(6); the Mission Planner capture confirmed HUD overlay + Messages. |
| `WEB_AP_GROUND_ONLY` | `1` | The AP/WiFi is only brought up **on the ground**; if there was never an FC link, the initial timeout enables setup on the bench. After linking once (`lock_ap`), losing the FC does not re-enable the AP. |
| `WEB_AP_FORCE` | `0` | 1 = force the AP always (bench; ignores ground detection). |
| `WEB_AP_GS_MAX_CMS` / `WEB_AP_ALT_MAX_MM` | `200` / `3000` | "In flight" thresholds: speed (cm/s) and altitude (mm) above which it is not ground. |
| `SIMULATION_MODE` | `0` | Hardware-free simulation on the **follower** (generates local packets, does not use LoRa). |
| `FC_EMULATION` | `1` (bench) | Emulates the FC: synthesises telemetry into `APdata` without UART1, **keeps real LoRa**. Set `0` for flight with an FC. |
| `FC_LINK_USB` | `0` | 1 = fixed UART0 route (legacy); current profiles boot on UART1. In dev, `FWM SIM ON` switches to UART0 in RAM only; a reset returns to UART1. |
| `FWM_ALLOW_RUNTIME_SITL` | `0` | Build guard: only the dev profile `ttgo-lora32-v1-sitl` accepts `FWM SIM ON`; production pins it to `0`. Not persisted in NVS. |
| `WEB_START_AP_IMMEDIATELY` | `1` | Start the WiFi AP at boot. |

---

## 7. Documentation (`*.md`)

| File | Type | Status |
|---|---|---|
| `README.md` | User manual | Feature overview, hardware, flashing, ground configuration, formation flight procedure, limits, badges. No development/testing content. |
| `DEVELOP.md` | Development and bench | Architecture, PlatformIO profiles, SITL, MAVProxy, simulators, bench and tests. |
| `docs/PROJECT.md` | Consolidated reference | Project overview, repository layout, security audit, validation results, testing strategy, roadmap and detailed findings F-01…F-21. |
| `AGENTS.md` | Agent guide | This document. |
| `SECURITY.md` | Security policy | Vulnerability reporting, scope, known risks. |
| `tools/build_web_flasher.py` | Web flasher site generator | One manifest per board. `--binaries-root` reuses the release binaries so the site and release hashes match. The beta flag comes from the shared `BOARDS` table. |
| `tools/package_firmware_release.py` | Release packager | Owns the shared `BOARDS` table (board → env, label, `beta`). Writes one zip per board and per-segment SHA-256 into `release-manifest.json`. |
| `tools/verify_site_hashes.py` | CI deploy gate | Re-hashes the **actual site binaries** and compares them with each release zip, per board, plus the site's `SHA256SUMS`. Fails the Pages deploy on any mismatch. |
| `tools/hil_test.py` | HIL test harness | Validates the leader/follower link over serial (see §2). |
| `tools/flash_firmware.py` | Flash and provision | Uploads the common flight image and stores `role` over USB serial; does not build a different binary per role. |
| `tools/lab.py` | Lab (CLI) | Menu: 2 SITL + headless MAVProxy → UDP for Mission Planner; actions and reports. |
| `tools/mp_launch.py` | Headless MAVProxy launcher | No wxPython on Windows; applies the pymavlink UDP patch. |
| `tools/sitl_bridge.py` | SITL ↔ board bridge | Serial↔TCP pipe; `--tap-port N` exposes a **direct MAVLink link to the board** (peripheral config **without WiFi**). |
| `tools/bench_suite.py` | Full bench | Scenarios (preflight/link/setup/params/formations/takeoff/straight/turn/mode_gate/head_on/safety) + MD/JSON report + **CSV** per scenario. `netid` requires `--netid-test`. |
| `requirements-dev.txt` | Python dependencies for the bench | MAVProxy, pymavlink, pyserial and prompt_toolkit for the CLI tools. |
| `tools/tests/` | Python tests for the CLI tools | Regression of DTR/RTS ordering when opening the serial bridge. |
| `tools/follow_sim.py` | Guidance law simulator (no hardware) | Replicates the cross-track law plus an aircraft model and **link latency**; trail/head_on/lateral scenarios and distance/latency sweeps. |
| `tools/follow_sim_test.py` | Predictor/offset regression | Checks that prediction does not cancel TRAIL offsets of 5/10/20/96 m. |
| `tools/proto_sim.py` | **Protocol** simulator (no hardware) | Models discovery by default, optional reply/session, timeouts, losses, blackout, netid filter and mode gate; **case/failure matrix**. |

> The former `MEJORAS_RECOMENDADAS.md`, `FASE{1..4}_IMPLEMENTADA.md`, `AUDITORIA_SEGURIDAD.md`,
> `VALIDACION_PRE_RELEASE.md`, `PLAN_PRUEBAS_SISTEMA.md`, `PLAN_CONFIG_TIERRA.md` and `ROADMAP.md`
> were consolidated into `docs/PROJECT.md` and removed from the repository to avoid duplicates.

### Documentation consistency notes

- **Environment `native`:** `platformio.ini` defines `[env:native]` for `pio test -e native`.
  `default_envs` is `ttgo-lora32-v1-flight`, so `pio test` without `-e` is not the right command.
- **Tests:** `test/test_main.cpp` contains **15** tests that link against the real `src/` headers.
- **Documentation:** user-facing flight steps go in `README.md`; SITL profiles, bench, test commands
  and internals go in `DEVELOP.md`.

Git state changes between tasks; check `git status` before editing, deleting or preparing commits.

---

## 8. Rules for agents (important)

1. **Do not edit** anything under `lib/mavlink/` (generated code).
2. **COM ports, coordinates and SITL paths do not belong in the repository.** They are configured in
   `tools/bench.local.json` (git-ignored; template in `tools/bench.example.json`). The Python and
   PowerShell scripts read them with `tools/bench_config.py` and `tools/bench_config.ps1`.
   **Never commit `tools/bench.local.json`.**
3. **LoRa protocol compatibility:** if you change `LoraPacket_t`/`CompressedLoraPacket_t`, keep
   reception of both formats (backward compatibility between firmware versions).
4. **Safety first:** any change to following must respect `isSafeToFollow()` and the limits in
   `config.h`. Never remove validations without justifying it.
5. **Feature flags:** add new capabilities behind a `#define` in `config.h` with a conservative
   default, and document it in the table in section 6.
6. **`config.h` is central:** most structs, enums and the `Logger` class live there. Avoid duplicate
   definitions.
7. **When changing behaviour, update the corresponding `.md`** (and correct any discrepancies in
   section 7). Keep `README.md` aimed at the end user; put technical and testing content in
   `DEVELOP.md`.
8. **Before proposing a commit**, build `ttgo-lora32-v1-flight`; if you change transport or
   simulation flags, also build `ttgo-lora32-v1-sitl`.
9. All project documentation is in **English**.
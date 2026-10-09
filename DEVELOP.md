# FlyWithMe — development and test bench

Technical details, PlatformIO profiles, SITL, simulators and bench procedures. The end-user guide is
in [`README.md`](README.md). Consolidated project reference (security audit, validation results,
testing strategy, roadmap) is in [`docs/PROJECT.md`](docs/PROJECT.md).

## Architecture

System for **ArduPlane**: the leader publishes state over SX1276/LoRa; the follower validates the
frame, computes its formation position and sends guidance via MAVLink in `GUIDED`.

```text
[LEADER]    FC --MAVLink/UART--> Telem --> FWM protocol v2 --> LoRa
[FOLLOWER]  FC <--MAVLink/UART-- Telem <-- Comm <-- LoRa
```

- `src/main.cpp`: startup and FreeRTOS tasks.
- `src/FWM.cpp`: modes, session, beacon scheduler, gate, parameters and Web/OLED coordination.
- `src/Comm.cpp`: sole owner of LoRa operations in the flight loop; RX/TX, validation, JOIN/REPLY
  slots and sequences.
- `src/Telem.cpp`: MAVLink with the FC, guidance, predictor, formation and `STATUSTEXT`.
- `src/protocol.h`: v2 wire format, **packed 40 bytes**, with no padding in the checksum.
- `src/selftest.h` / `src/status_text.h`: protocol self-test and pure testable deduplication.
- `src/config.h`: flags, limits, parameters and mode tables.
- `lib/mavlink/`: generated headers; **do not edit**.

## PlatformIO profiles

| Environment | Role / use | FC |
|---|---|---|
| `ttgo-lora32-v1-flight` | Locked production firmware; UART1 | `FC_EMULATION=0`, `FWM_ALLOW_RUNTIME_SITL=0` |
| `ttgo-lora32-v1-sitl` | Universal **dev/HIL only** image; boots in flight mode, `FWM SIM ON` switches temporarily to SITL over USB. A reset returns to UART1 | `FC_EMULATION=0`, `FC_LINK_USB=0`, `FWM_ALLOW_RUNTIME_SITL=1` |
| `ttgo-lora32-v1` | Bench with a locally emulated FC | `FC_EMULATION=1`; do not use for real flight |
| `ttgo-lora32-v21-flight` | Same firmware for the TTGO LoRa32 **V1.6 / V2.0 / V2.1.6** (LILYGO "T3"). **Not yet validated on hardware.** | `FWM_BOARD=2`, `FC_EMULATION=0`; **no** `-DF_XTAL_MHZ=26` |
| `native` | Host unit tests of the pure modules | no firmware build |

### Board pin maps

`config.h` selects the pin map with `-D FWM_BOARD=...` (`FWM_BOARD_V1` is the default, so the V1
targets are unchanged). A wrong map does **not** fail to compile: it leaves the radio mute. The
V1.6/V2.0 differs from the V1 in four ways that matter:

| | V1 (`FWM_BOARD_V1`) | V1.6/V2.0 (`FWM_BOARD_V21`) |
|---|---|---|
| Crystal | external 26 MHz (`-DF_XTAL_MHZ=26`) | none exposed; PICO-D4 internal 40 MHz → drop the flag |
| OLED SDA / SCL | 4 / 15 | 21 / 22 |
| OLED reset | 16 | none (`OLED_RST = -1`) |
| LoRa RST | 14 | **23** |
| UART1 RX / TX | 12 / 13 | **32 / 33** |

Sources and the full reasoning are in [`docs/PROJECT.md` §2.3](docs/PROJECT.md#23-board-compatibility-only-the-ttgo-lora32-v1-is-supported).
Note that the Arduino variant `ttgo-lora32-v2` defines `LORA_RST 12` with the comment `// GPIO14`:
**both are wrong**, which is why this target builds `board = ttgo-lora32-v21` instead.

Profiles use Arduino ESP32 core 3.x, C++17 and `huge_app.csv`. The
`ttgo-lora32-v1-flight` profile does **not** define the role in build flags: every board receives the
same binary. `ttgo-lora32-v1-sitl` shares behaviour and protocol, but includes the temporary SIM
command for bench use only; it is never packaged or published in production releases.

`role` is stored per board in NVS (`OFF=0`, `FOLLOWER=1`, `LEADER=2`); boards without that key
(including those still holding NVS from an old firmware) boot in `OFF` and must be provisioned.
Changing the role on the ground saves NVS and reboots the board. FWM/FC SYSID: leader 1, follower 2.
A new board in `OFF` can be provisioned before an FC is detected; changing an active role requires
telemetry with a valid position, disarmed and on the ground (an FC disconnect during flight does not
count as proof of being on the ground).

`default_envs = ttgo-lora32-v1-flight`, so a bare `pio run` never builds the emulation or bench
profiles by accident.

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py -m pip install -r requirements-dev.txt  # Python dependencies for the CLI bench
pio run -e ttgo-lora32-v1-flight
pio run -e ttgo-lora32-v1-sitl
pio run -e ttgo-lora32-v1

# Flash the same flight image and provision each role; substitute the COM ports
& $py tools\flash_firmware.py --port COMx --role leader
& $py tools\flash_firmware.py --port COMy --role follower

# Once only, to prepare development/HIL boards; afterwards SIM is toggled by command
& $py tools\flash_firmware.py --environment ttgo-lora32-v1-sitl --port COMx --role leader
& $py tools\flash_firmware.py --environment ttgo-lora32-v1-sitl --port COMy --role follower

# Re-provision only, without reflashing
& $py tools\flash_firmware.py --port COMx --role follower --provision-only

# Serial monitor
pio device monitor --port COMx --baud 57600
```

On Windows the script sets `PYTHONIOENCODING=utf-8` for PlatformIO and uses USB serial at 57600 to
provision the role. It requires `pyserial`. CP210x adapters reenumerate; identify the current COM.
Both current profiles boot with the FC on UART1 (`FC_LINK_USB=0`); the dev profile only allows
`FWM SIM ON` to switch temporarily to UART0. Do not attach two owners to the same port.

## Effective configuration

- Radio: `LORA_BAND` (compile-time default) resolves to the runtime `params.band`; SF12/BW125k in
  flight, fixed sync word `0x34`.
- **Band**: runtime parameter `band` (0=433, 1=868, 2=915), NVS + WebUI + MAVLink + USB
  (`FWM BAND 433|868|915`). Default derives from `LORA_BAND` via `FWM_DEFAULT_BAND`. Changing it
  is ground-only (`canWriteConfig()`) and does `LoRa.end()` + `LoRa.begin(freq)` + re-applies every
  setter, because changing frequency needs the PLL reprogrammed and the setters must come *after*
  `begin()`. `Comm::applyBand()` only raises a flag: the core 1 loop applies it, because only that
  loop touches the SX1276. **Both boards must match**; there is no detection of a mismatch, so each
  board publishes its band in the OSD header, in `FWM_ID ... band=868` and as a STATUSTEXT when it
  enters SEARCHING. 2.4 GHz is impossible on the SX1276 (137-1020 MHz).
- HIL in the dev profile: `FWM SIM ON` applies SF7/BW250k and 5 Hz for that session; those timings
  **do not represent** real SF12.
- `LORA_CRC=0`, `NETID_USE_SYNCWORD=0` by default. The frame has a one-byte additive checksum,
  `version`, `type`, `netid`, `mode`, `seq` and flags. `netid=4660 (0x1234)` filters networks at
  packet level, but **does not encrypt or authenticate**.
- `USE_COMPRESSED_PACKETS=0`: the older compressor does not contain all v2 fields.
- `FWM_DUAL_CORE=1`: core 1 owns the flight/radio loop; core 0 runs Web/OLED/logger. The Ticker
  callback only sets a flag; it does not touch SPI/SX1276.
- `FOLLOWER_REPLY=1`: JOIN/REPLY only when a BEACON requests a slot. The leader listens in a limited
  RX window; session timeout and return to discovery are modelled in `proto_sim.py`.
- `FWM SIM ON` exists only in `ttgo-lora32-v1-sitl` (dev/HIL): it requires a safe interlock,
  temporarily switches UART1→UART0, silences USB text and applies bench radio/altitude. It is not
  saved to NVS; for testing it publishes RX/TX/link counters as `NAMED_VALUE_INT`; a reset returns to
  flight. `ttgo-lora32-v1-flight` pins `FWM_ALLOW_RUNTIME_SITL=0`.
- `role` and `ap_mode` are also persisted in NVS: `ap_mode=0 AUTO`, `1 ON (ground only)`, `2 OFF`.
  The firmware never leaves the AP active in flight; it can be queried/controlled via `/api/ap` or
  the USB console (`FWM AP auto|on|off`). `band` is persisted the same way (`FWM BAND`).
- The AP SSID is computed as `FWM XXXXXX` using the last 3 bytes of the SoftAP MAC. The board prints
  `FWM_ID ap_mac=... ap_ssid="..." role=... sysid=...` on UART0 at boot and returns the same line on
  receiving `FWM ID` (57600 baud).
- `netid`, `formation`, offsets and `approach_dist` are persisted in NVS. The WebUI and `PARAM_SET`
  are ground-only configurable (except for parameters explicitly marked otherwise).
- The WebUI lists `role` first and filters fields with `ParamDef_t.scope`: the leader shows the
  common ones (`role`, `netid`, `link_timeout`); the follower adds formation, offsets, gains,
  prediction, filter, `foll_enable` and `approach_dist`. Hidden values are preserved in NVS; the
  MAVLink server keeps publishing the full table.
- `approach_dist=300m`: below it the leader must be in FBWA/FBWB/CRUISE/AUTO/RTL/LOITER/TAKEOFF/GUIDED.
  If suspended because of an unstable mode, the FC holds the last setpoint: **this is not a
  LOITER/hold**.
- `dist_offset=96m` by default. TRAIL prediction limited to `PREDICTION_MAX_LEAD_FRACTION=0.25`;
  `millis()` is not subtracted between two unsynchronised ESP32s.
- `MIN_SAFE_ALTITUDE=50m` compares MAVLink `relative_alt` against the autopilot home; it is not AGL
  nor a terrain check. The web AP is only used on the ground.
- `HEAD_ON_GUARD=0` in `config.h` and in the production flight profile. The dev/HIL profile
  `ttgo-lora32-v1-sitl` enables it (`1`) to validate the head-on scenario in ArduPlane SITL only.

## SITL bench and MAVProxy

### Local bench configuration

Test field coordinates, COM ports and the ArduPlane path are **not versioned**. Create
`tools/bench.local.json` from `tools/bench.example.json` (git-ignored):

```json
{
  "sitl_exe": "C:/path/to/ArduPlane.exe",
  "leader_com": "COMx",
  "slave_com": "COMy",
  "leader_home": "lat,lon,alt,yaw",
  "follower_home": "lat,lon,alt,yaw",
  "bench_target": "lat,lon,alt",
  "bench_turn_target": "lat,lon,alt"
}
```

Keys you omit fall back to `bench.example.json`. `lab.py`, `bench_suite.py`, `hil_test.py`,
`bench_restart.ps1` and `sitl_start.ps1` read this file. Without it, `lab.py` warns and uses generic
values. COM ports are not pinned in `platformio.ini`: use `tools/flash_firmware.py --port COMx` or
`--upload-port`.

> Never commit `tools/bench.local.json`.

**ArduPlane** is used (not ArduCopter). `tools/lab.py` starts two SITL instances and MAVProxy, which
combines both vehicles into a single UDP output for Mission Planner. The SITL SERIAL0 goes to the
board through `sitl_bridge.py`; SERIAL1 goes to MAVProxy/MP and SERIAL2 is the control link.

```powershell
# SITL without boards: opens the interactive menu
python tools\lab.py

# Optional: connect the boards to SITL through serial bridges; COM ports are examples
python tools\lab.py --firmware --leader-com COMx --slave-com COMy
# Menu: 1 start/restart bench; 2 status; 9 full bench; 8 stop bench; 0 exit
```

With `--tap-port`, each bridge exposes MAVLink directly to the peripheral (TCP localhost 5790 leader
/ 5791 follower) over the same USB, without WiFi. The tap is used for FWM parameters and serial
observation. The bridge reconnects if SITL closes SERIAL0. The lab applies a DTR/RTS reset to the
boards when it starts.

`mp_launch.py` starts headless MAVProxy on Windows without wxPython and switches the UDP bind to an
ephemeral port to avoid clashing with Mission Planner on the same PC. Mission Planner listens by
default on `UDP 14550`.

### Bench steps

1. Once on the bench, build/flash both boards with `ttgo-lora32-v1-sitl` (universal dev profile),
   keeping the NVS roles. This is not a production image. To load it manually use
   `python tools/flash_firmware.py --environment ttgo-lora32-v1-sitl --port COMx --role leader|follower`.
2. Start `tools/lab.py --firmware --leader-com COMx --slave-com COMy` and choose option 1. The lab
   resets each board, requests `FWM SIM ON` over USB (without persisting it) and aborts if the
   firmware is locked or does not confirm SIM. It opens two ArduPlane SITL with SYSID 1/2 and starts
   the bridges immediately (SITL can take longer than the FWM search timeout); then it starts
   MAVProxy.
3. Connect Mission Planner to `UDP 14550` and confirm both vehicles appear.
4. Run the link and session checks first; then takeoff, straight and turn. Option 9 runs the full
   bench. Option 8 stops the bench; option 0 exits and also cleans up.
5. SIM mode is volatile; when the bench stops, the lab terminates its processes and reboots the
   boards to restore the UART1 boot path. The bench can provision `role=LEADER` and `role=FOLLOWER`
   over the TAP; those roles **do** persist in NVS. Reports are written under `tools/reports/`.
   Scenarios that change NVS and potentially colliding scenarios are disabled by default; review the
   options before running a bench.
6. Ctrl+C/CtrlBreak cancels the bench in an orderly way; `bench_suite.py` runs its `finally` to close
   SITL/bridges and reset the boards.

In SIM the textual ArduinoLog logs are silenced so they do not mix with MAVLink. The `FWM_RX`,
`FWM_TX` and `FWM_LINK` counters and the head-on metrics travel as `NAMED_VALUE_INT`; mode gate/guard
events are published as `STATUSTEXT`. Tests must read the TAP as MAVLink, not search for
`Link: rx=...` lines in serial text.

> Note: `lab.py` bridges die when stdin closes, which is why the full bench is driven by
> `bench_suite.py --start-bench --stop-bench-after` rather than from the interactive menu.

## Tests and scenarios

Python bench tools (`pyserial`, `pymavlink`, MAVProxy and `prompt_toolkit`):

```powershell
python -m pip install -r requirements-dev.txt
```

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"
& $py tools\proto_sim.py                 # protocol matrix (17 checks)
& $py tools\follow_sim_test.py           # predictor, offsets and geometry (8 checks)
& $py -m unittest discover -s tools\tests -p "test_*.py" -v  # CLI utilities
& $py tools\bench_suite.py --dist-offset 20 --only preflight,link,setup,session,takeoff,straight
& $py tools\bench_suite.py --netid-test  # ground-only; changes and restores NVS
& $py tools\bench_suite.py --head-on-test # SITL only; potentially colliding manoeuvre
```

The bench generates `report.md`, `report.json` and a CSV per scenario in `tools/reports/<date>/`.
TAKEOFF positions both aircraft at 200 m before measuring, and the normal routes go south because
there is terrain north of the home point. The straight/turn thresholds fail if the minimum horizontal
separation drops below 10 m when testing a 20 m offset. Head-on is **skipped by default** and
requires an explicit option.

Scenarios: preflight, link, setup/netid, round-trip params, five formations on the ground, TAKEOFF,
straight, turn, ACRO→GUIDED gate, head-on (opt-in) and safety. `session` checks REPLY/RX, expiry,
discovery and re-JOIN; `leader_osd` records STATUSTEXT received over MAVLink.

Acceptance criteria for each validation level are in
[`docs/PROJECT.md` §5](docs/PROJECT.md#5-testing-strategy).

## Test limitations

`tools/follow_sim.py` lets you observe the guidance law at different distances and latencies without
boards; `tools/hil_test.py` checks the link over serial with an emulated FC (load the shared profile
and provision different roles with `flash_firmware.py`). Host unit tests run with
`pio test -e native` (`test/test_main.cpp` links against the pure headers in `src/`; on Windows a C/C++
compiler must be on the PATH). The `SELFTEST PASS` self-test runs on every boot and checks wire
layout, checksum, version, netid, sequence and status-text deduplication. The simulators are
deterministic models; they do not replace SITL or RF testing.
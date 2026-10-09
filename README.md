# FlyWithMe

![Release](https://img.shields.io/github/v/release/Amigache/FlyWithMe?display_name=tag&sort=semver)
![CI](https://github.com/Amigache/FlyWithMe/actions/workflows/ci.yml/badge.svg)
![CodeQL](https://github.com/Amigache/FlyWithMe/actions/workflows/codeql.yml/badge.svg)
![License](https://img.shields.io/badge/license-GPLv3-blue.svg)
![Language](https://img.shields.io/badge/docs-English-lightgrey.svg)

![Web flasher](https://img.shields.io/badge/flasher-ESP%20Web%20Tools-ff7043?logo=espressif&logoColor=white)
[![Web flasher](https://img.shields.io/badge/web-flasher-open-blue)](https://amigache.github.io/FlyWithMe/)

**Formation flight with ArduPlane over LoRa.** A leader aircraft broadcasts its position and a
follower aircraft computes and executes a relative position through MAVLink. The follower must be in
`GUIDED` mode. PX4 is not validated.

> **FlyWithMe is under active development.** It does not replace the pilot, the autopilot failsafes,
> or a certified collision-avoidance system. Perform early flights with a pilot in command, an
> observer, ample airspace, and an independent plan to recover control.

---

## Contents

- [What it does](#what-it-does)
- [Hardware](#hardware)
- [Flashing the firmware](#flashing-the-firmware)
- [Ground configuration](#ground-configuration)
- [Formation flight](#formation-flight)
- [Important limits](#important-limits)
- [Security](#security)
- [Documentation](#documentation)
- [License](#license)

---

## What it does

Two aircraft fly in formation using a LoRa radio link:

1. The **leader** reads its position from the autopilot over MAVLink and broadcasts it over LoRa.
2. The **follower** receives those packets, applies prediction, a position filter and the
   formation offset, then feeds the resulting waypoint to its own autopilot in `GUIDED` mode.

Features:

- **Formations**: `TRAIL`, `LEFT`, `RIGHT`, `ABOVE`, `BELOW`, with configurable offsets.
- **Guidance by heading** (cross-track error correction) rather than naive carrot chasing, which
  keeps the follower from oscillating.
- **Predictive tracking** so the follower aims where the leader will be, not where it was.
- **Leader mode gating**: the follower only follows while the leader is in a stable flight mode
  (FBWA, FBWB, CRUISE, AUTO, RTL, LOITER, TAKEOFF, GUIDED). Near the leader it requires a stable
  mode and otherwise stops issuing new waypoints.
- **Session protocol** with discovery, request/reply windows, timeouts and rejoin, so a follower
  can join a running leader without restarting.
- **Adaptive LoRa rate**: transmission interval scales with distance (2 s / 1 s / 0.5 s).
- **OLED display** showing link state, formation, distance and warnings.
- **Configuration on the ground** over Wi-Fi (access point) or over MAVLink parameters, visible in
  Mission Planner as component 158.
- **Flight log** written to SPIFFS as CSV.

A single firmware image serves both roles. The leader/follower role is a runtime parameter stored in
the board's NVS, **not** a compile-time flag, so there is only ever one flight binary.

---

## Hardware

- Two aircraft with ArduPlane and a **TTGO LoRa32 V1** (ESP32 + SX1276).
- LoRa antennas matched to your region's frequency and regulations. **Connect the antennas before
  powering the boards.**
- A MAVLink UART cable between each TTGO and a free TELEM port on the autopilot.
- A computer with PlatformIO to flash the firmware.

| UART connection | TTGO LoRa32 V1 |
|---|---|
| FC TX → ESP32 RX | GPIO12 |
| ESP32 TX → FC RX | GPIO13 |
| Ground | common GND |

Configure the chosen TELEM port in ArduPlane for MAVLink2 (`SERIALx_PROTOCOL=2`) and 57600 baud
(`SERIALx_BAUD=57`), where `x` depends on the physical port you use. Keep the wiring crossed, and do
not connect buttons to the OLED menu: its pins conflict with signals already used on this board.

> The TTGO LoRa32 V1 carries a **26 MHz crystal**. The firmware must be built with
> `-DF_XTAL_MHZ=26`, which is already set in `platformio.ini`. Without it the Arduino core assumes
> 40 MHz, the UART runs at roughly ×0.65 and WiFi falls out of band entirely.

---

## Flashing the firmware

Install PlatformIO and open a terminal in the project folder. Every flight board uses the same
profile and the same firmware file: `ttgo-lora32-v1-flight`.

On Windows, identify the current COM port of each board first. CP210x adapters can change COM
numbers when reconnected, so never assume a given number always belongs to the leader or the
follower.

```powershell
$py = "$env:USERPROFILE\.platformio\penv\Scripts\python.exe"

# Each command flashes THE SAME image and stores the given role on that board
& $py tools\flash_firmware.py --port COMx --role leader
& $py tools\flash_firmware.py --port COMy --role follower
```

Replace the COM numbers with the ones you identified. The command builds/uploads the shared profile
and then provisions the role over USB serial; it requires `pyserial` in the Python being used.

Without PlatformIO you can install the flight profile from the
[web flasher](https://amigache.github.io/FlyWithMe/) using desktop Chrome or Edge, then assign the
role from the same panel.

Notes:

- Do not flash while a serial monitor or another program holds that port.
- Set `SYSID_THISMAV=1` on the leader autopilot and `SYSID_THISMAV=2` on the follower. The profiles
  assign those same SYSIDs from the `role` parameter.
- A newly flashed board stays in `OFF` until you provision it: `OFF=0`, `FOLLOWER=1`, `LEADER=2`.
- To change only the role of an already flashed board, add `--provision-only`.
- The role can also be changed from the WebUI or from Mission Planner while on the ground. Changing
  it reboots the board so the new identity takes effect.
- If you are upgrading from an older firmware, the board boots in `OFF` until you assign the role.

On boot, check each board for errors and confirm the autopilot is connected.

---

## Ground configuration

1. Power each aircraft separately and join its TTGO Wi-Fi network. Each board announces
   `FWM XXXXXX`, where `XXXXXX` are the last three bytes of the SoftAP MAC in hexadecimal; for
   example `30:AE:A4:07:0D:64` produces `FWM 070D64`. The initial password is `12345678`.
2. Open `http://192.168.4.1` and review the parameters of **both** boards. The access point is
   disabled in flight; do not depend on the WebUI while formation flying.
3. Confirm `role=LEADER` on the leader and `role=FOLLOWER` on the follower, and leave `foll_enable`
   on for the follower. The SSID is unique per MAC and does not depend on the role. The line
   `FWM_ID ap_mac=... ap_ssid="..." role=... sysid=...` appears on the serial console at boot; you
   can also request it by sending `FWM ID` at 57600 baud.
4. Set the same `netid` on both (initial value `4660`). `netid` separates networks; it does not
   encrypt or authenticate anything.
5. On the follower choose a formation: **TRAIL**, **LEFT**, **RIGHT**, **ABOVE** or **BELOW**.
6. For first flights use **TRAIL**, keep the configured initial separation (96 m) and leave
   `approach_dist` at 300 m. Do not reduce the separation until you have verified the link and the
   aircraft response under real, controlled conditions.
7. Check antennas, power, GPS, control directions, modes, limits and failsafes on each autopilot.
   Verify the pilot can leave `GUIDED` and take over at any moment.

### Changing the Wi-Fi password

The factory password `12345678` is public and identical on every board. It can be used as-is, but
changing it is recommended from the **Wi-Fi password** section of the WebUI. You can also change it
over MAVLink with the `WIFI_CONFIG_AP` message (`password` field, empty SSID); the confirmation
arrives as `STATUSTEXT`. The new password must be 8 to 63 printable ASCII characters.

While the factory password is in place, anyone who joins a board's access point on the ground can
change its configuration. Do not leave the access point enabled where other people can connect.

Parameters are stored on each board, so review their effective values: changing one does not
automatically change the other.

---

## Formation flight

1. Take off both aircraft independently and keep manual/autopilot supervision under your control.
   Wait until both have a valid GPS position, are above the firmware's altitude threshold (50 m),
   and a link exists between leader and follower.
2. Establish a wide separation and a predictable trajectory. Keep the leader in a stable mode
   supported by the firmware (FBWA, FBWB, CRUISE, AUTO, RTL, LOITER, TAKEOFF or GUIDED), especially
   when getting closer than `approach_dist`.
3. Once the follower is settled and at a safe distance, select `GUIDED` on the follower to enable
   following. Verify it holds the TRAIL formation before continuing.
4. Start with straight legs and wide turns. Continuously watch separation, altitude, link state and
   autopilot messages. The pilot must be ready to leave `GUIDED` if anything behaves unexpectedly.
5. To end formation following, the pilot must switch the follower to an approved mode and fly it
   independently before approaching or landing. FlyWithMe performs no automatic transition to
   `LOITER`, no recovery and no landing.

---

## Important limits

- The 50 m threshold is a firmware altitude check, **not** a terrain-avoidance system. Always
  respect your autopilot's and local legal altitudes and limits.
- If the leader's mode stops being stable near the threshold, FlyWithMe stops issuing new commands
  and the autopilot may hold the last target. This **does not** amount to position hold.
- Do not fly head-on approaches, sharp reversals up close, or at 5–20 m separations. The head-on
  guard is experimental, is **disabled in the production firmware**, and must not be treated as a
  collision-avoidance system.
- The LoRa link is **neither authenticated nor encrypted**. A third party with a compatible modem and
  the same `netid` can transmit false position data, which the follower would use to fly the
  aircraft. This is a known limitation of this version: do not fly where others may transmit on your
  network, and always watch the formation.
- The WebUI and REST API have **no authentication**. They are only available on the ground.
- Behaviour depends on the radio link, GPS, wind, configuration and autopilot. There is no airworthiness
  certification for real flight.

---

## Security

See [`SECURITY.md`](SECURITY.md) for how to report a vulnerability, and
[`docs/PROJECT.md`](docs/PROJECT.md#3-security-audit) for the full audit, including the accepted
risks above.

---

## Documentation

| Document | Contents |
|---|---|
| [`README.md`](README.md) | This file: user manual. |
| [`DEVELOP.md`](DEVELOP.md) | Development, SITL bench, flashing and test commands. |
| [`docs/PROJECT.md`](docs/PROJECT.md) | Consolidated project reference: security audit, validation results, testing strategy, roadmap, detailed findings. |
| [`AGENTS.md`](AGENTS.md) | Conventions and working rules for coding agents. |
| [`SECURITY.md`](SECURITY.md) | Vulnerability reporting policy. |

---

## License

GNU GPL v3.0. See [`LICENSE`](LICENSE).
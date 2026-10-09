# FlyWithMe — Project Documentation

Consolidated reference for the project: security audit, validation results, testing strategy and
roadmap. It replaces the previous `docs/AUDITORIA_SEGURIDAD.md`, `docs/VALIDACION_PRE_RELEASE.md`,
`docs/PLAN_PRUEBAS_SISTEMA.md` and `docs/ROADMAP.md`.

For user-facing instructions see [README.md](../README.md). For development setup see
[DEVELOP.md](../DEVELOP.md). For agent conventions see [AGENTS.md](../AGENTS.md).

---

## 1. Project overview

FlyWithMe is formation flight for ArduPlane using a LoRa radio link between two aircraft:

- A **leader** broadcasts its position over LoRa (SX1276, 866 MHz).
- A **follower** computes a formation position relative to the leader and feeds it to its own
  autopilot over MAVLink, using `GUIDED` mode.

Hardware target is the TTGO LoRa32 V1 (ESP32 + SX1276). One single firmware image is used for all
aircraft; the leader/follower role is a runtime parameter stored in NVS, not a build flag.

**FlyWithMe is under development.** It does not replace the pilot, the autopilot failsafes, or a
certified collision-avoidance system.

### 1.1 Current status

| Item | State |
|---|---|
| Release | `v1.0.0` published with firmware bundle and web flasher |
| Build | Both profiles compile with pinned versions (PlatformIO `espressif32@55.03.37`, Arduino core 3.3.7) |
| CI | `ci.yml` (build, native tests, Python tools) and `codeql.yml` (C/C++ and Python) green on `main` and `develop` |
| Bench | Full HIL bench passed in SITL: 18 PASS / 0 FAIL, plus `netid` and `head_on` in dedicated runs |
| Hardware validation | Self-test, roles, production image and WiFi password change verified on both boards |
| Pending | MAVLink password change (V6) and write-blocking (V7) require a real autopilot on the bench |
| Known risks | LoRa link is not authenticated; factory WiFi password is public (see §3) |

---

## 2. Repository and release layout

```
src/          firmware (Arduino C++17)
  config.h    all configuration, constants, structs, Logger
  protocol.h  LoRa wire format v2 (pure, host-testable)
  selftest.h  on-boot self-test (pure, host-testable)
  wifi_identity.h  SSID derivation and WPA2 passphrase validation (pure)
test/         Unity tests for the pure modules (`pio test -e native`)
tools/        bench, flashing, SITL harness, simulators, release packaging
web/          web flasher source (ESP Web Tools)
.github/      CI, CodeQL, release, web flasher, Dependabot
docs/         this document
```

### 2.1 Build profiles

| Profile | Purpose | Key flags |
|---|---|---|
| `ttgo-lora32-v1-flight` | **Production.** The only image published in releases. | `FC_EMULATION=0`, `FC_LINK_USB=0`, `FWM_ALLOW_RUNTIME_SITL=0`, `HEAD_ON_GUARD=0` |
| `ttgo-lora32-v1-sitl` | Development / HIL only. Never fly this image. | `FWM_ALLOW_RUNTIME_SITL=1`, `HEAD_ON_GUARD=1` |
| `native` | Host unit tests of the pure modules. | `native@1.2.1`, no firmware build |

`platformio.ini` sets `default_envs = ttgo-lora32-v1-flight`, so a bare `pio run` never builds the
emulation or bench profiles by accident.

### 2.2 Platform pin

The build is pinned to the pioarduino fork release `55.03.37` by URL:

```
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.37/platform-espressif32.zip
```

This fork is required because the project needs Arduino core 3.x / ESP-IDF 5. The official
PlatformIO registry only publishes `espressif32` 1.x–7.x, which resolve to Arduino core 2.x.
Pinning `espressif32@55.3.37` does **not** work: that version does not exist in the registry.

---

## 3. Security audit

Full audit with severities, decisions and per-finding locations: **§7** below. Summary of the
findings that shape day-to-day work:

### 3.1 Accepted risks

These are deliberate and documented, not oversights:

| Risk | Why accepted | Mitigation |
|---|---|---|
| **LoRa link is not authenticated or encrypted.** The checksum is a byte sum and the `netid` is a shared public filter. Anyone with a compatible radio and the same `netid` can inject false leader positions and modes. | Implementing a MAC-authenticated protocol (v3) is a breaking change requiring HIL validation and a new key-distribution process. | Documented in README, `SECURITY.md` and here. Do not fly where third parties can transmit on your network. |
| **Factory WiFi password `12345678`** is public and identical on every board. | Some boards have no OLED and cannot display a generated password. | Changeable from the WebUI or over MAVLink. Not forced. |
| **WebUI and REST API have no authentication.** | The system is under test. | To be hardened in a later phase. The access point only exists on the ground. |

### 3.2 Fixed findings

- **Configuration writes are fail-closed** (`canWriteConfig()`): losing the FC link after it was
  established blocks parameter edits, instead of assuming "ground".
- **`pio run` cannot build the wrong profile**: `default_envs` points at the production image.
- **Factory password is no longer printed** in the serial log.
- **Legacy unauthenticated code path removed** (`POST /save`).
- **Coordinates, COM ports and SITL paths are not in the repository**: they live in
  `tools/bench.local.json` (git-ignored), read by `tools/bench_config.py` and `tools/bench_config.ps1`.
- **Dependencies pinned**: platform, framework, all libraries, PlatformIO, Python dev tools.
- **CI hardened**: per-job permissions, actions pinned by commit SHA, Dependabot, CodeQL.
- **Real unit tests**: `pio test -e native` links against the actual `src/` headers.
- **History rewritten**: personal email, field coordinates and COM ports removed from all commits.

### 3.3 Repository history

The repository was force-rewritten with `git filter-repo` twice (email, then coordinates and COM
ports), and finally deleted and recreated so the old SHAs became unreachable. Verified: zero
occurrences across all objects. Local backup bundles were deleted.

**Do not commit `tools/bench.local.json`** (git-ignored) or re-introduce field coordinates.

### 3.4 Open findings

| ID | Severity | Finding |
|---|---|---|
| F-02 / F-07 | Medium | WebUI and `/api/logs` unauthenticated. Accepted for now; harden later. |
| F-05 | Medium | `isOnGround()` still fails open, but is now only used for reporting (`/api/stats`), not for writes. |
| F-08 | Medium | Operational: both bench boards must carry the production image. Verified 2026-10-09. |
| F-10 | Low | Hardcoded COM ports and paths remain in `platformio.ini`? No: removed. See §7. |
| F-12 | Low | CI improvements shipped; remaining item is SHA-pinning the two `github/codeql-action` steps on Dependabot PRs. |
| F-13 | Low | `Comm`/`Telem` logic (frame validation, prediction, distance) has no host tests because it depends on Arduino. |

---

## 4. Validation

### 4.1 Automated (CI, every push and PR)

| Check | Result |
|---|---|
| Build `ttgo-lora32-v1-flight` | PASS |
| Build `ttgo-lora32-v1-sitl` | PASS |
| Clean-core build (fresh PlatformIO, pinned platform URL) | PASS |
| `pio test -e native` (15 tests against `src/`) | PASS |
| Python tools compile (16 files) | PASS |
| `tools/tests` unit tests | PASS |
| `tools/proto_sim.py` protocol matrix | PASS 17/17 |
| `tools/follow_sim_test.py` predictor and offsets | PASS 8/8 |
| CodeQL C/C++ and Python | PASS |
| Release packaging and web flasher generation | PASS |

### 4.2 On-board validation (2026-10-09)

Both boards: `COM13` leader (`FWM C5B569`, SYSID 1), `COM21` follower (`FWM C5C559`, SYSID 2).

| ID | Test | Result |
|---|---|---|
| V1 | Self-test at boot | PASS · `protocol: 66 checks, 0 fails` |
| V2 | Board identity (`FWM ID`) | PASS · correct SSID, role, SYSID on both |
| V3 | Production rejects simulation | PASS · `SIMCFG ERR` on both boards |
| V4 | Role provisioning over USB | PASS · `ROLECFG OK leader` / `follower` |
| V5 | WiFi password change (WebUI) | PASS · saved to NVS, board reboots, WebUI requires the new password |
| V6 | WiFi password change (MAVLink `WIFI_CONFIG_AP`) | NOT RUN · requires a real autopilot |
| V7 | Write-blocking after FC link loss | NOT RUN · requires a real autopilot in flight-like conditions |
| V8 | Reflash both boards with production image | PASS · self-test and roles preserved |
| V9 | Web flasher on a real board | NOT RUN · requires a Chrome/Edge session |

V6 and V7 are documented as pending: V6 needs an active MAVLink link to a real autopilot; V7 needs
the link to drop while armed. Neither is meaningful in SITL, because the link is TCP to a simulator.

### 4.3 Bug found and fixed during V5

The WebUI "Change password" button never worked in the published `v1.0.0` firmware.

ESPAsyncWebServer registers routes without a wildcard using the `BackwardCompatible` matcher, which
compares `(url == route) || url.startsWith(route + "/")` (`WebServer.cpp:336`). `/api/ap` therefore
also matched `/api/ap/pass`, and since `AsyncWebServer::_attachHandler` serves the first matching
handler, the `/api/ap` handler answered `falta parámetro mode` and `setApPassphrase()` was never
called.

Fix: register `/api/ap/pass` before `/api/ap`, with a comment in `src/Web.cpp` explaining why the order
matters. Verified over HTTP after reflashing: all three rejection cases return the correct message,
and the real change persists in NVS.

### 4.4 Bench (HIL in SITL)

Bench complete, board roles leader/follower, ArduPlane SITL, `--stop-bench-after`. Omitted scenarios
were run in dedicated passes.

**Main pass** — `tools/reports/20261009-094523` — **PASS=18 FAIL=0 SKIP=1** (`head_on` skipped):

| Scenario | Result | Key data |
|---|---|---|
| `role_setup` | PASS | leader role 2, follower role 1, FWM SYSID 1/2 |
| `preflight` | PASS | both FCs reachable, both disarmed |
| `link` | PASS | parameter readback on both |
| `setup` | PASS | `netid` 4660 on both |
| `session` | PASS | JOIN/REPLY session established |
| `leader_osd` | PASS | leader sees the follower |
| `params` | PASS | read/write/readback without WiFi |
| `formations` | PASS | TRAIL, LEFT, RIGHT, ABOVE, BELOW |
| `takeoff` | PASS | 200 m, settling to 111 m commanded separation |
| `straight` | PASS | separation mean 19.4 m, min 16.0 m, max 23.8 m |
| `turn` | PASS | separation mean 23.9 m, min 14.2 m, roll max 44.6° |
| `mode_gate` | PASS | holds when leader in ACRO, resumes when stable |
| `safety` | PASS | separation min 52.0 m while recovering |

**`netid`** — `tools/reports/20261009-095511` — PASS: same ID links, different ID cuts the link,
netid restored to 4660.

**`takeoff` + `head_on`** — `tools/reports/20261009-095859` — PASS=3 FAIL=0:

| Scenario | Result | Key data |
|---|---|---|
| `takeoff` | PASS | leader 200.5 m, follower 186.3 m |
| `head_on` | PASS | **4 guard activations**, face angle max 177°, **minimum separation 20.1 m** |

The `head_on` guard is only compiled into the development profile (`HEAD_ON_GUARD=1`); production
keeps it disabled because it is experimental and is not a certified collision-avoidance system.

> The first `head_on` attempt was skipped because the `--only` invocation skipped `takeoff`, so the
> SITL aircraft were on the ground. Rerunning with `takeoff` first produced the PASS above.

---

## 5. Testing strategy

Levels, in order. A software-simulation PASS does not replace HIL, and a HIL PASS does not authorize
real flight.

| Level | Area | PASS criterion |
|---|---|---|
| L0 | Build and static | Both profiles compile, Python compiles, `git diff --check` clean |
| L1 | Pure code | Self-test, protocol and guidance simulators, native unit tests |
| L2 | CLI tools | Bridges, provisioning, packaging, web flasher generation |
| L3 | SITL without boards | Two ArduPlane instances, MAVProxy, UDP 14550, clean shutdown |
| L4 | HIL with boards | `SIMCFG OK` on both, FWM heartbeats (`COMPID=158`) on both TAPs, roles and SYSID correct, scenarios produce a report |
| L5 | Distribution / real FC | Release contains only the `flight` profile; production firmware rejects `FWM SIM ON` |

Safety rules for the bench:

- `netid` and `head_on` are opt-in and only ever in SITL, never with a real FC.
- `head_on` is a deliberate head-on encounter. Read the guard metrics, not just PASS/FAIL.
- HIL requires aircraft on the ground and disarmed, antennas connected, correct COM ports.
- Do not start a destructive or moving stage without explicit user confirmation.

---

## 6. Roadmap

| Item | State |
|---|---|
| Role as runtime parameter, single flight image | Done |
| Runtime LoRa band selection (433/868/915) | Done in code, pending board validation |
| Multi-board flash targets (V2.1, T-Beam) | Not started. Blocked on V2.1 pin ambiguity (core variant says `LORA_RST 12`, its own comment says GPIO14). |
| `canWriteConfig()` fail-closed writes | Done |
| WiFi password change (WebUI + MAVLink) | Done (WebUI verified on board; MAVLink pending V6) |
| `pio test -e native` with real `src/` headers | Done |
| CI + CodeQL + Dependabot + pinned actions | Done |
| Web flasher on GitHub Pages | Done |
| Bench parameterization (no coordinates in repo) | Done |
| **Authenticate the LoRa protocol (v3, MAC)** | Not started. Largest remaining risk. |
| **WebUI/API authentication** | Not started. |
| Host tests for `Comm`/`Telem` pure logic | Not started. Requires extracting pure logic into headers. |
| V6 / V7 with a real autopilot | Pending bench session. |

---

## 7. Detailed audit findings

Severity: **High** can affect flight safety or allow third-party control of a board. **Medium**
exposes data or weakens a stated guarantee. **Low** is hygiene or reproducibility.

| ID | Severity | Status | Finding |
|---|---|---|---|
| F-01 | High | Accepted | Factory WiFi password `12345678` is public and identical on all boards. Changeable via `POST /api/ap/pass` and MAVLink `WIFI_CONFIG_AP`. Optional lock via `FWM_FORCE_AP_PASS_CHANGE` (disabled). |
| F-02 | High | Accepted | WebUI and REST API have no authentication. System under test. |
| F-03 | High | Documented | LoRa protocol has no authentication or encryption; `netid` is a public filter. Injected beacons can steer the follower. |
| F-04 | High | Fixed | `pio run` built every environment, including `ttgo-lora32-v1` (`FC_EMULATION=1`). Fixed with `default_envs`. |
| F-05 | Medium | Fixed | `isOnGround()` failed open, allowing parameter edits when the FC link dropped. Replaced by fail-closed `canWriteConfig()` for `groundOnly`, `/api/config` and `PARAM_SET`. |
| F-06 | Medium | Fixed | AP password was written to the serial log at every boot. Removed. |
| F-07 | Medium | Accepted | `/api/logs` serves flight log (contains positions) without authentication. |
| F-08 | Medium | Fixed 2026-10-09 | Bench boards carried the development image. Both reflashed with `ttgo-lora32-v1-flight`, `SIMCFG ERR` verified. |
| F-09 | Low | Fixed | Legacy `#if !USE_WEB_SERVER` block with unauthenticated `POST /save`, `getPostParam`/`urlDecode`, old TCP server. Removed. |
| F-10 | Low | Fixed | Field coordinates, COM ports and SITL paths in tools and `platformio.ini`. Moved to `tools/bench.local.json`; `monitor_port`/`upload_port` removed. |
| F-11 | Low | Fixed | Unpinned dependencies. Platform (by URL), libraries, `requirements-dev.txt` and `platformio` are now pinned. |
| F-12 | Low | Fixed | CI: per-job permissions, SHA-pinned actions, `persist-credentials: false`, `ci.yml`, `dependabot.yml`. |
| F-13 | Low | Fixed (partial) | `pio test` failed and duplicated helpers. Replaced with 15 tests against real headers. `Comm`/`Telem` remain uncovered (Arduino dependency). |
| F-14 | Info | Fixed | Author email in every commit. History rewritten to noreply; repository recreated. |
| F-15 | Info | Fixed | README described a WiFi password change that did not exist. Corrected. |
| F-16 | Info | Fixed | No `SECURITY.md`. Added, with private vulnerability reporting. |
| F-17 | Info | Fixed | Release and flasher outputs could be committed. `site/`, `release-assets/` git-ignored. |

### 7.1 GitHub settings to verify manually

- Pages source: **GitHub Actions**.
- Secret scanning with push protection; Dependabot alerts and security updates.
- Private vulnerability reporting enabled (referenced by `SECURITY.md`).
- Workflow permissions: **Read repository contents**.
- Rulesets: `main` (no force-push, no delete, required CI checks) and `refs/tags/v*`.
- `github-pages` environment allows `main` and tags `v*`.
- Immutable releases enabled.
# Security Policy

FlyWithMe controls the following of real aircraft. Treat any security issue as high priority,
including those affecting the LoRa link, the WebUI, or board configuration.

## Supported versions

Only the latest published release (`v*`) and the `main` branch are maintained. Development branches
may change without notice.

## Reporting a vulnerability

**Do not open a public issue** with exploitation details.

1. Use **Security → Report a vulnerability** on GitHub (private vulnerability reporting).
2. Include: version or commit, firmware profile (`ttgo-lora32-v1-flight` or `-sitl`), steps to
   reproduce, expected impact (flight, configuration, data), and whether access to the WiFi AP or the
   radio is required.

We will reply to confirm receipt and agree on a fix and disclosure timeline.

## Scope

- Firmware in `src/`, tools in `tools/` and workflows in `.github/`.
- The web flasher published on GitHub Pages.

Out of scope: ArduPilot/PX4 bugs, third-party hardware faults, and documented insecure
configurations (for example, using the default WiFi password in a shared environment).

## Known risks

See [`docs/PROJECT.md` §3](docs/PROJECT.md#3-security-audit) for the full audit. In particular:

- The LoRa link is **neither authenticated nor encrypted**; `netid` only separates networks.
- The factory WiFi password is public and identical on every board. It can be used as-is, but
  changing it is not forced.
- The WebUI and REST API have no authentication; they are ground-only.
- FlyWithMe does not replace the pilot or the autopilot failsafes.
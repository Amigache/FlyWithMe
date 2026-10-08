#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Carga la imagen común y provisiona el rol de una placa por USB serie."""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: falta pyserial. Usa el Python de PlatformIO o instala pyserial.", file=sys.stderr)
    raise SystemExit(2)


ROOT = Path(__file__).resolve().parent.parent
ROLES = ("off", "leader", "follower")


def upload(port, environment):
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    command = [
        "pio", "run", "-e", environment, "-t", "upload", "--upload-port", port,
    ]
    return subprocess.run(command, cwd=ROOT, env=env).returncode == 0


def provision(port, role, timeout):
    command = f"FWM ROLE {role}\n".encode("ascii")
    deadline = time.monotonic() + timeout
    next_send = 0.0
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 57600
        ser.timeout = 0.25
        # Evita accionar DTR/RTS al abrir el puerto y resetear innecesariamente la placa.
        ser.dtr = False
        ser.rts = False
        ser.open()
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: no pude abrir {port}: {exc}", file=sys.stderr)
        return False

    print(f"Esperando firmware en {port}; asignando rol {role}...", flush=True)
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                ser.write(command)
                ser.flush()
                next_send = now + 0.5
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if line:
                print(line)
            if "ROLECFG OK" in line:
                if "rebooting" in line:
                    print(f"Rol {role} guardado en NVS; la placa reiniciará para aplicarlo.")
                else:
                    print(f"La placa ya tenía el rol {role} guardado.")
                return True
            if "ROLECFG ERR" in line:
                print("ERROR: el firmware rechazó la configuración del rol.", file=sys.stderr)
                return False
        print(
            "ERROR: no llegó confirmación ROLECFG. Comprueba COM, firmware, placa en tierra "
            "y monitor serie cerrado.",
            file=sys.stderr,
        )
        return False
    finally:
        ser.close()


def main():
    # Las líneas del firmware pueden incluir Unicode; fuerza UTF-8 también para esta consola
    # (el PYTHONIOENCODING definido en upload() solo afecta al proceso PlatformIO hijo).
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(
        description="Carga el firmware FlyWithMe y configura el rol sin compilar una variante por rol."
    )
    parser.add_argument("--port", required=True, help="COM de la placa, por ejemplo COMx")
    parser.add_argument("--role", required=True, choices=ROLES, help="off, leader o follower")
    parser.add_argument(
        "--environment",
        choices=("ttgo-lora32-v1-flight", "ttgo-lora32-v1-sitl", "ttgo-lora32-v1"),
        default="ttgo-lora32-v1-flight",
        help="flight para producción; sitl es imagen universal dev/HIL con FWM SIM ON; ttgo-lora32-v1 es emulación",
    )
    parser.add_argument(
        "--provision-only",
        action="store_true",
        help="no volver a cargar firmware; solo guardar el rol en una placa ya flasheada",
    )
    parser.add_argument("--timeout", type=float, default=30.0, help="timeout de provisión en segundos")
    args = parser.parse_args()

    if not args.provision_only:
        print(f"Cargando imagen común {args.environment} en {args.port}...")
        if not upload(args.port, args.environment):
            return 1

    return 0 if provision(args.port, args.role, args.timeout) else 1


if __name__ == "__main__":
    raise SystemExit(main())

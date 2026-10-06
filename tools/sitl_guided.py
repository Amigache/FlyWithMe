#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pone un vehiculo SITL en modo GUIDED via MAVLink (con pymavlink).

Se conecta a un puerto MAVLink de SITL (SERIAL2: TCP 5763 lider / 5773 seguidor) para no
ocupar el puerto del puente (SERIAL0) ni el de Mission Planner (SERIAL1).

Uso:
  python tools/sitl_guided.py --conn tcp:127.0.0.1:5773        # seguidor -> GUIDED (copter=4)
  python tools/sitl_guided.py --conn tcp:127.0.0.1:5763 --mode 15   # plane GUIDED=15
"""
import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: falta pymavlink (pip install pymavlink).")
    sys.exit(2)


def main():
    ap = argparse.ArgumentParser(description="Pone un vehiculo SITL en GUIDED")
    ap.add_argument("--conn", default="tcp:127.0.0.1:5773", help="endpoint MAVLink de SITL")
    ap.add_argument("--mode", type=int, default=4, help="custom_mode GUIDED (copter=4, plane=15)")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.conn, source_system=255)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("ERROR: sin heartbeat de SITL")
        sys.exit(1)
    print(f"SITL sysid={m.target_system} mode={m.flightmode}")

    # SET_MODE
    m.mav.set_mode_send(m.target_system,
                        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                        args.mode)
    # y tambien COMMAND_LONG DO_SET_MODE (mas compatible)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, args.mode,
        0, 0, 0, 0, 0)

    # Confirmar
    deadline = time.time() + 6
    while time.time() < deadline:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(f"modo ahora: {mode}")
            if mode.startswith("GUIDED"):
                break
    m.close()


if __name__ == "__main__":
    main()

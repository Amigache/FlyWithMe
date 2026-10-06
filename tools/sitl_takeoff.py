#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pone un avion SITL en GUIDED, arma y despega (MAVLink, pymavlink).

Uso:
  python tools/sitl_takeoff.py --conn tcp:127.0.0.1:5763 --alt 120   # lider
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
    ap = argparse.ArgumentParser(description="GUIDED + arm + takeoff de un avion SITL")
    ap.add_argument("--conn", default="tcp:127.0.0.1:5763")
    ap.add_argument("--mode", type=int, default=13, help="custom_mode: TAKEOFF plane=13, GUIDED=15")
    ap.add_argument("--alt", type=float, default=120.0, help="altitud de despegue (m)")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.conn, source_system=255)
    if m.wait_heartbeat(timeout=10) is None:
        print("ERROR: sin heartbeat")
        sys.exit(1)
    print(f"SITL sysid={m.target_system} mode={m.flightmode}")
    tgt, comp = m.target_system, m.target_component

    # SITL no trae calibracion 3D de acelerometros -> desactivar checks de armado
    m.mav.param_set_send(tgt, comp, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(1)

    m.mav.set_mode_send(tgt, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, args.mode)
    time.sleep(1)

    # Armar
    m.mav.command_long_send(tgt, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                            1, 0, 0, 0, 0, 0, 0)
    time.sleep(2)
    # En GUIDED hay que ordenar el despegue; en TAKEOFF (13) el avion despega solo al armar
    if args.mode == 15:
        m.mav.command_long_send(tgt, comp, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                0, 0, 0, 0, 0, 0, args.alt)

    # Esperar y mostrar altitud relativa
    deadline = time.time() + 20
    alt = 0.0
    while time.time() < deadline:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            alt = msg.relative_alt / 1000.0
            if alt > 30:
                break
    print(f"altitud relativa: {alt:.1f} m  modo: {m.flightmode}")
    m.close()


if __name__ == "__main__":
    main()

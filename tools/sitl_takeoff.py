#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pone un avion SITL en un modo y lo arma/despega (MAVLink, pymavlink), con reintentos.

Uso:
  python tools/sitl_takeoff.py --conn tcp:127.0.0.1:5763 --mode 13 --alt 120   # lider
  python tools/sitl_takeoff.py --conn tcp:127.0.0.1:5773 --mode 13 --alt 80    # seguidor
"""
import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: falta pymavlink (pip install pymavlink).")
    sys.exit(2)


def wait_ekf(m, timeout=30):
    """Espera a que el EKF/posicion sea usable (heartbeat + posicion global)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        g = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if hb and g:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description="GUIDED/TAKEOFF + arm + takeoff de un avion SITL")
    ap.add_argument("--conn", default="tcp:127.0.0.1:5763")
    ap.add_argument("--mode", type=int, default=13, help="custom_mode: TAKEOFF plane=13, GUIDED=15")
    ap.add_argument("--alt", type=float, default=120.0, help="altitud de despegue (m)")
    ap.add_argument("--target-alt", type=float, default=30.0, help="altura a la que considerar despegado")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.conn, source_system=255)
    if m.wait_heartbeat(timeout=15) is None:
        print("ERROR: sin heartbeat")
        sys.exit(1)
    print(f"SITL sysid={m.target_system} mode={m.flightmode}")
    tgt, comp = m.target_system, 1

    # Streams de telemetria
    m.mav.request_data_stream_send(tgt, comp, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    wait_ekf(m, timeout=30)

    # SITL no trae calibracion 3D de acelerometros -> desactivar checks de armado
    for _ in range(3):
        m.mav.param_set_send(tgt, comp, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.5)

    m.mav.set_mode_send(tgt, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, args.mode)
    time.sleep(1)

    # Armado con reintentos
    armed = False
    for attempt in range(6):
        m.mav.command_long_send(tgt, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                1, 0, 0, 0, 0, 0, 0)
        dl = time.time() + 3
        while time.time() < dl:
            hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                break
            ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.1)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                print(f"  arm ack result={ack.result}")
        if armed:
            break
        time.sleep(1)
    print("armado:", armed)

    # En GUIDED hay que ordenar el despegue; en TAKEOFF (13) el avion despega solo al armar
    if args.mode == 15:
        m.mav.command_long_send(tgt, comp, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                                0, 0, 0, 0, 0, 0, args.alt)

    deadline = time.time() + 45
    alt = 0.0
    while time.time() < deadline:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if msg:
            alt = msg.relative_alt / 1000.0
            if alt > args.target_alt:
                break
    print(f"altitud relativa: {alt:.1f} m  modo: {m.flightmode}")
    m.close()


if __name__ == "__main__":
    main()

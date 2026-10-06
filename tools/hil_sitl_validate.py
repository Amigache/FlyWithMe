#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validacion autonoma HIL con SITL: comprueba que el seguidor persigue al lider.

- Conecta a las dos FC SITL (SERIAL2: lider 5763 / seguidor 5773).
- Comprueba que ambos estan en el aire.
- Manda al LIDER (GUIDED) a uno o varios puntos.
- Monitoriza la distancia lider-seguidor y valida que se mantiene cerca de la
  separacion de formacion (TRAIL ~100 m) mientras el lider se mueve.

Uso:
  python tools/hil_sitl_validate.py [--target lat,lon,alt] [--seconds 90]
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def latest(conn, mtype):
    msg = None
    while True:
        m = conn.recv_match(type=mtype, blocking=False)
        if m is None:
            break
        msg = m
    return msg


def pos(conn):
    m = latest(conn, 'GLOBAL_POSITION_INT')
    if m is None:
        return None
    return (m.lat / 1e7, m.lon / 1e7, m.relative_alt / 1000.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--leader', default='tcp:127.0.0.1:5763')
    ap.add_argument('--follower', default='tcp:127.0.0.1:5773')
    ap.add_argument('--seconds', type=int, default=90)
    ap.add_argument('--target', default=None, help='lat,lon,alt para mover al lider')
    ap.add_argument('--follower-guided', action='store_true',
                    help='poner el seguidor en GUIDED justo despues de mandar el lider lejos')
    args = ap.parse_args()

    lead = mavutil.mavlink_connection(args.leader, source_system=255)
    fol = mavutil.mavlink_connection(args.follower, source_system=253)
    if lead.wait_heartbeat(timeout=10) is None or fol.wait_heartbeat(timeout=10) is None:
        print("ERROR: sin heartbeat de algun SITL")
        sys.exit(1)
    print(f"Lider sysid={lead.target_system} mode={lead.flightmode} | "
          f"Seguidor sysid={fol.target_system} mode={fol.flightmode}")

    lp, fp = pos(lead), pos(fol)
    if not lp or not fp:
        print("ERROR: sin posicion")
        sys.exit(1)
    print(f"Lider   : {lp[0]:.6f},{lp[1]:.6f} alt={lp[2]:.1f} m")
    print(f"Seguidor: {fp[0]:.6f},{fp[1]:.6f} alt={fp[2]:.1f} m")
    if lp[2] < 5 or fp[2] < 5:
        print("AVISO: algun avion parece estar en tierra (<5 m)")

    # Mover el lider a un punto (armado forzado + GUIDED + NAV_WAYPOINT por COMMAND_INT)
    if args.target:
        lat, lon, alt = [float(x) for x in args.target.split(',')]
        # armado forzado (param2=21196) por si se desarmo
        lead.mav.command_long_send(lead.target_system, lead.target_component,
                                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   1, 21196, 0, 0, 0, 0, 0)
        time.sleep(2)
        lead.mav.set_mode_send(lead.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
        time.sleep(1)
        hb = lead.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        led_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else False
        lead.mav.command_int_send(lead.target_system, lead.target_component,
                                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                  mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                                  -1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), float(alt))
        print(f"-> Lider GUIDED (DO_REPOSITION) a {lat:.6f},{lon:.6f} alt={alt} (armed={led_armed})")

    if args.follower_guided:
        fol.mav.set_mode_send(fol.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
        print("-> Seguidor -> GUIDED")

    dists = []
    t0 = time.time()
    print("\n t(s)   dist(m)   lider_alt  seg_alt")
    while time.time() - t0 < args.seconds:
        lp, fp = pos(lead), pos(fol)
        if lp and fp:
            d = haversine(lp[0], lp[1], fp[0], fp[1])
            dists.append(d)
            print(f"{time.time()-t0:5.0f}   {d:7.1f}   {lp[2]:7.1f}   {fp[2]:7.1f}")
        time.sleep(2)

    print("\n=== RESULTADO ===")
    if dists:
        print(f"muestras={len(dists)} dist min={min(dists):.0f} max={max(dists):.0f} "
              f"media={sum(dists)/len(dists):.0f} m")
        # criterio: el seguidor se mantiene a <300 m del lider (formacion ~100 m) la mayor parte del tiempo
        cerca = sum(1 for d in dists if d < 300) / len(dists)
        print(f"fraccion a <300 m: {cerca*100:.0f}%")
        ok = cerca > 0.7 and min(dists) < 250
        print("RESULTADO:", "PASS" if ok else "FAIL")
        sys.exit(0 if ok else 1)
    else:
        print("sin datos")
        sys.exit(1)


if __name__ == "__main__":
    main()

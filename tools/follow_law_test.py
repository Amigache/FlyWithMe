#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Banco de pruebas de la LEY DE SEGUIMIENTO cross-track (sin reflashear).

Conecta a dos SITL (lider 5763 / seguidor 5773), despega ambos, mueve al lider
por una ruta (recta + giro opcional) y ejecuta en el PC la misma ley que el
firmware (cross-track + velocidad longitudinal), enviando
GUIDED_CHANGE_HEADING/SPEED + DO_REPOSITION periodico para la altitud.

Permite elegir la formacion y los parametros de la ley para afinar sin reflashear.

Uso:
  python tools/follow_law_test.py --formation trail --seconds 90
  python tools/follow_law_test.py --formation left --along-gain 12 --deadband 2 --quant 50
"""
import argparse
import math
import time

from pymavlink import mavutil

DIST_OFFSET = 100.0     # m de retraso (TRAIL)
LATERAL = 50.0          # m (LEFT/RIGHT)
VERTICAL = 20.0         # m (ABOVE/BELOW)
ALT_REFRESH = 2.0
RATE_HZ = 5.0
GUIDED_CHANGE_HEADING = 43002
GUIDED_CHANGE_SPEED = 43000
MAX_HEADING_CORR = 25.0
MAX_SPEED_BOOST = 600
MAX_SPEED_SLOW = 400
AIRSPD_MIN, AIRSPD_MAX = 10.0, 30.0
FRAME = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


def connect(conn, sysid):
    m = mavutil.mavlink_connection(conn, source_system=sysid)
    m.wait_heartbeat(timeout=15)
    m.mav.request_data_stream_send(m.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    return m


def drain(conn):
    d = {}
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        d[msg.get_type()] = msg
    return d


def takeoff(conn):
    tgt = conn.target_system
    for _ in range(3):
        conn.mav.param_set_send(tgt, 1, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)
    conn.mav.set_mode_send(tgt, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 13)
    time.sleep(1)
    for _ in range(8):
        conn.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(1)
        d = drain(conn)
        hb = d.get('HEARTBEAT')
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
    dl = time.time() + 45
    while time.time() < dl:
        d = drain(conn)
        g = d.get('GLOBAL_POSITION_INT')
        if g and g.relative_alt / 1000.0 > 30:
            return True
        time.sleep(0.5)
    return False


def geo_offset(lat, lon, dist_m, brg_deg):
    br = math.radians(brg_deg)
    dlat = dist_m * math.cos(br) / 111320.0
    dlon = dist_m * math.sin(br) / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def formation_point(llat, llon, lalt, hdg_deg, formation):
    """Replica de calculateFormationPosition (punto objetivo + altitud)."""
    if formation == 'trail':
        plat, plon = geo_offset(llat, llon, DIST_OFFSET, hdg_deg + 180.0)
        palt = lalt
    elif formation == 'left':
        plat, plon = geo_offset(llat, llon, LATERAL, hdg_deg - 90.0)
        palt = lalt
    elif formation == 'right':
        plat, plon = geo_offset(llat, llon, LATERAL, hdg_deg + 90.0)
        palt = lalt
    elif formation == 'above':
        plat, plon = llat, llon
        palt = lalt + VERTICAL
    elif formation == 'below':
        plat, plon = llat, llon
        palt = lalt - VERTICAL
    return plat, plon, palt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--leader', default='tcp:127.0.0.1:5763')
    ap.add_argument('--follower', default='tcp:127.0.0.1:5773')
    ap.add_argument('--seconds', type=int, default=90)
    ap.add_argument('--formation', default='trail',
                    choices=['trail', 'left', 'right', 'above', 'below'])
    ap.add_argument('--turn', action='store_true')
    ap.add_argument('--alt', type=float, default=80)
    ap.add_argument('--cross-gain', type=float, default=0.5)
    ap.add_argument('--cross-max', type=float, default=25.0)
    ap.add_argument('--along-gain', type=float, default=12.0)
    ap.add_argument('--along-i', type=float, default=0.0, help='cm/s por (m*s) - termino integral')
    ap.add_argument('--deadband', type=float, default=2.0)
    ap.add_argument('--quant', type=float, default=50.0)
    ap.add_argument('--no-takeoff', action='store_true')
    args = ap.parse_args()

    lead = connect(args.leader, 255)
    fol = connect(args.follower, 253)
    if not args.no_takeoff:
        print("despegando...")
        takeoff(lead)
        takeoff(fol)
    print("en el aire")

    tgt_l, tgt_f = lead.target_system, fol.target_system
    for c in (lead, fol):
        c.mav.set_mode_send(c.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)
    time.sleep(2)

    g = drain(lead).get('GLOBAL_POSITION_INT')
    llat, llon = g.lat / 1e7, g.lon / 1e7
    if args.turn:
        route = geo_offset(llat, llon, 4000, 180.0)
        route2 = geo_offset(llat, llon, 8000, 90.0)
    else:
        route = geo_offset(llat, llon, 6000, 180.0)
        route2 = None
    lead.mav.command_int_send(tgt_l, 1, FRAME, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                              -1, 0, 0, 0, int(route[0] * 1e7), int(route[1] * 1e7), float(args.alt))
    route_t0 = time.time()

    alongs, crosses, dists = [], [], []
    t0 = time.time()
    last_alt = 0.0
    switched = False
    rows = []
    integral = 0.0
    dt = 1.0 / RATE_HZ
    while time.time() - t0 < args.seconds:
        dl, df = drain(lead), drain(fol)
        gl, gf = dl.get('GLOBAL_POSITION_INT'), df.get('GLOBAL_POSITION_INT')
        att = df.get('ATTITUDE')
        if not gl or not gf:
            time.sleep(1.0 / RATE_HZ)
            continue

        llat, llon, lalt = gl.lat / 1e7, gl.lon / 1e7, gl.relative_alt / 1000.0
        flat, flon, falt = gf.lat / 1e7, gf.lon / 1e7, gf.relative_alt / 1000.0
        vn, ve = gl.vx / 100.0, gl.vy / 100.0
        vmag = math.hypot(vn, ve) or 1.0
        un, ue = vn / vmag, ve / vmag
        theta = math.degrees(math.atan2(ue, un))
        hdg = dl.get('VFR_HUD').heading if dl.get('VFR_HUD') else theta

        plat, plon, palt = formation_point(llat, llon, lalt, hdg, args.formation)
        dn = (flat - plat) * 111320.0
        de = (flon - plon) * 111320.0 * math.cos(math.radians(flat))
        along = dn * un + de * ue
        cross = -dn * ue + de * un

        corr = max(-args.cross_max, min(args.cross_max, -args.cross_gain * cross))
        hcmd = (theta + corr) % 360.0

        along_dead = 0.0 if abs(along) < args.deadband else along
        integral += along * dt
        integral = max(-1000.0, min(1000.0, integral))
        boost = max(-MAX_SPEED_SLOW, min(MAX_SPEED_BOOST, -along_dead * args.along_gain - args.along_i * integral))
        base = (dl.get('VFR_HUD').groundspeed if dl.get('VFR_HUD') else 22.0)
        vcmd = base + boost / 100.0
        vcmd = max(AIRSPD_MIN, min(AIRSPD_MAX, vcmd))
        vcmd = round(vcmd * 100.0 / args.quant) * args.quant / 100.0

        fol.mav.command_int_send(tgt_f, 1, FRAME, GUIDED_CHANGE_HEADING, 0, 0, 0.0, hcmd, 20.0, 0.0, 0, 0, 0.0)
        fol.mav.command_int_send(tgt_f, 1, FRAME, GUIDED_CHANGE_SPEED, 0, 0, 0.0, vcmd, 1.0, 0.0, 0, 0, 0.0)
        if time.time() - last_alt >= ALT_REFRESH:
            fol.mav.command_int_send(tgt_f, 1, FRAME, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                                     -1, 0, 0, 0, int(plat * 1e7), int(plon * 1e7), palt)
            last_alt = time.time()

        # solo metricas en regimen (tras 30 s)
        if time.time() - t0 > 30:
            alongs.append(along)
            crosses.append(cross)
        dists.append(math.hypot((flat - llat) * 111320.0,
                                (flon - llon) * 111320.0 * math.cos(math.radians(flat))))
        rows.append((time.time() - t0, along, cross, flat, flon))

        if args.turn and not switched and time.time() - route_t0 > args.seconds * 0.5:
            lead.mav.command_int_send(tgt_l, 1, FRAME, mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                                      -1, 0, 0, 0, int(route2[0] * 1e7), int(route2[1] * 1e7), float(args.alt))
            switched = True
            print("-> giro del lider")

        time.sleep(1.0 / RATE_HZ)

    print("\n=== RESULTADO ===")
    if alongs:
        m = lambda v: sum(v) / len(v)
        std = lambda v, mm: math.sqrt(sum((x - mm) ** 2 for x in v) / len(v))
        print(f"formacion={args.formation}  cross-gain={args.cross_gain} along-gain={args.along_gain} "
              f"deadband={args.deadband} quant={args.quant}")
        print(f"along  medio={m(alongs):6.1f} m  std={std(alongs, m(alongs)):.1f}")
        print(f"cross  medio={m(crosses):6.1f} m  std={std(crosses, m(crosses)):.1f}  |max|={max(abs(c) for c in crosses):.1f}")
        print(f"dist lider-seguidor: media={m(dists):.0f} m  min={min(dists):.0f} max={max(dists):.0f}")


if __name__ == '__main__':
    main()

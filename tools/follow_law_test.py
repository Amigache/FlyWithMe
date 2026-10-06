#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Banco de pruebas de la LEY DE SEGUIMIENTO (sin reflashear).

Conecta a dos SITL (lider 5763 / seguidor 5773), despega ambos, mueve al lider
por una ruta (recta + giro) y ejecuta en el PC la ley de guiado del seguidor
enviando GUIDED_CHANGE_HEADING/SPEED (+ DO_REPOSITION periodico para la altitud).

Registra la pista del seguidor y metricas de zigzag (error de cruce / distancia).

Uso:
  python tools/follow_law_test.py --seconds 120 --law cross
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

R = 6371000.0
DIST_OFFSET = 100.0     # m de retraso (TRAIL)
ALT_REFRESH = 2.0       # s entre DO_REPOSITION (altitud)
LAW_RATE_HZ = 5.0
GUIDED_CHANGE_HEADING = 43002
GUIDED_CHANGE_SPEED = 43000
MAV_FRAME_REL = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


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


def takeoff(conn, alt):
    tgt = conn.target_system
    for _ in range(3):
        conn.mav.param_set_send(tgt, 1, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)
    conn.mav.set_mode_send(tgt, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 13)
    time.sleep(1)
    for _ in range(8):
        conn.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                   1, 0, 0, 0, 0, 0, 0)
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
    """Desplaza (lat,lon) dist_m en la marcacion brg_deg (grados)."""
    br = math.radians(brg_deg)
    dlat = dist_m * math.cos(br) / 111320.0
    dlon = dist_m * math.sin(br) / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--leader', default='tcp:127.0.0.1:5763')
    ap.add_argument('--follower', default='tcp:127.0.0.1:5773')
    ap.add_argument('--seconds', type=int, default=120)
    ap.add_argument('--law', default='cross', choices=['point', 'cross'])
    ap.add_argument('--alt', type=float, default=80)
    ap.add_argument('--turn', action='store_true', help='ruta con un giro de 90 grados')
    args = ap.parse_args()

    lead = connect(args.leader, 255)
    fol = connect(args.follower, 253)
    print("despegando...")
    if not takeoff(lead, args.alt):
        print("AVISO: lider no despego")
    if not takeoff(fol, args.alt):
        print("AVISO: seguidor no despego")
    print("en el aire")

    tgt_l = lead.target_system
    tgt_f = fol.target_system

    def set_guided(conn):
        conn.mav.set_mode_send(conn.target_system,
                               mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)

    set_guided(lead)
    set_guided(fol)
    time.sleep(2)

    dl = drain(lead)
    g = dl.get('GLOBAL_POSITION_INT')
    llat, llon = g.lat / 1e7, g.lon / 1e7
    # Ruta del lider: 2 puntos (recta) o un giro de 90
    if args.turn:
        route = [geo_offset(llat, llon, 4000, 180.0), geo_offset(llat, llon, 4000, 180.0)]
        route2 = geo_offset(llat, llon, 8000, 90.0)
    else:
        route = [geo_offset(llat, llon, 6000, 180.0)]
        route2 = None

    # Lanzar al lider al primer waypoint
    lead.mav.command_int_send(tgt_l, 1, MAV_FRAME_REL,
                              mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
                              -1, 0, 0, 0, int(route[0][0] * 1e7), int(route[0][1] * 1e7),
                              float(args.alt))
    route_t0 = time.time()

    cross_errs = []
    dists = []
    track = []
    t0 = time.time()
    last_alt = 0.0
    switched = False
    print(f"\n t  dist  cross  north-raw  head  roll  gsl  gsf")
    while time.time() - t0 < args.seconds:
        dl = drain(lead)
        df = drain(fol)
        gl = dl.get('GLOBAL_POSITION_INT')
        gf = df.get('GLOBAL_POSITION_INT')
        att = df.get('ATTITUDE')
        huds = df.get('VFR_HUD')
        hudl = dl.get('VFR_HUD')
        if not gl or not gf:
            time.sleep(1.0 / LAW_RATE_HZ)
            continue

        llat, llon, lalt = gl.lat / 1e7, gl.lon / 1e7, gl.relative_alt / 1000.0
        flat, flon, falt = gf.lat / 1e7, gf.lon / 1e7, gf.relative_alt / 1000.0
        vn, ve = gl.vx / 100.0, gl.vy / 100.0  # m/s
        vmag = math.hypot(vn, ve)
        if vmag < 1.0:
            vmag = 1.0
        un, ue = vn / vmag, ve / vmag

        # Punto de formacion (TRAIL): 100 m detras del lider
        theta = math.degrees(math.atan2(ue, un))  # rumbo de la traza
        plat, plon = geo_offset(llat, llon, DIST_OFFSET, theta + 180.0)
        palt = lalt

        # Errores relativos al punto de formacion en ejes de la traza
        dn = (flat - plat) * 111320.0
        de = (flon - plon) * 111320.0 * math.cos(math.radians(flat))
        along = dn * un + de * ue
        cross = dn * (-ue) + de * un  # positivo = a la izquierda de la traza

        # Ley de rumbo
        if args.law == 'point':
            # apuntar directo al punto (lo que hace el firmware actual)
            brg = math.degrees(math.atan2(de, dn))
            hcmd = brg
        else:
            # cross-track: traza + correccion proporcional al error lateral (signo que converge)
            k = 0.5  # deg/m
            corr = max(-25.0, min(25.0, -k * cross))
            hcmd = theta + corr
        hcmd = (hcmd + 360.0) % 360.0

        # Velocidad: traza del lider + correccion de distancia al punto
        dist_point = math.hypot(dn, de)
        along_err = along - 0.0  # queremos along=0 en el punto
        base = hudl.groundspeed if hudl else 22.0
        vcmd = base + max(-4.0, min(6.0, 0.12 * -along_err))
        vcmd = max(12.0, min(28.0, vcmd))

        fol.mav.command_int_send(tgt_f, 1, MAV_FRAME_REL, GUIDED_CHANGE_HEADING, 0, 0,
                                 0.0, hcmd, 20.0, 0.0, 0, 0, 0.0)
        fol.mav.command_int_send(tgt_f, 1, MAV_FRAME_REL, GUIDED_CHANGE_SPEED, 0, 0,
                                 0.0, vcmd, 1.0, 0.0, 0, 0, 0.0)
        if time.time() - last_alt >= ALT_REFRESH:
            fol.mav.command_int_send(tgt_f, 1, MAV_FRAME_REL, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                     0, 0, -1, 0, 0, 0, int(plat * 1e7), int(plon * 1e7), palt)
            last_alt = time.time()

        cross_errs.append(cross)
        dists.append(dist_point)
        track.append((flat, flon))
        if int(time.time() - t0) % 2 == 0 and len(track) % 10 == 1:
            yaw = math.degrees(att.yaw) if att else 0
            roll = math.degrees(att.roll) if att else 0
            gsf = huds.groundspeed if huds else 0
            gsl = hudl.groundspeed if hudl else 0
            print(f"{time.time()-t0:5.0f} {dist_point:6.0f} {cross:6.0f} {vn:6.0f} {yaw:6.0f} {roll:6.0f} {gsl:5.1f} {gsf:5.1f}")

        # cambiar de rumbo del lider a mitad (giro)
        if args.turn and not switched and time.time() - route_t0 > args.seconds * 0.5:
            lead.mav.command_int_send(tgt_l, 1, MAV_FRAME_REL, mavutil.mavlink.MAV_CMD_DO_REPOSITION,
                                      0, 0, -1, 0, 0, 0, int(route2[0] * 1e7), int(route2[1] * 1e7),
                                      float(args.alt))
            switched = True
            print("-> giro del lider")

        time.sleep(1.0 / LAW_RATE_HZ)

    print("\n=== RESULTADO ===")
    if dists:
        abs_cross = [abs(c) for c in cross_errs]
        print(f"dist media={sum(dists)/len(dists):.0f} m  min={min(dists):.0f} max={max(dists):.0f}")
        print(f"|cross| media={sum(abs_cross)/len(abs_cross):.1f} m  max={max(abs_cross):.1f} m")
        # pista
        with open('tools/_follow_track.csv', 'w') as f:
            for la, lo in track:
                f.write(f"{la:.7f},{lo:.7f}\n")
        print("pista -> tools/_follow_track.csv")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FlyWithMe -- Banco de pruebas COMPLETO (escenarios + reporte).

Ejecuta una bateria de escenarios contra el banco (2 SITL + firmware seguidor) y genera
un reporte (Markdown + JSON) en tools/reports/<fecha>/ para su analisis posterior.

Escenarios:
  link      : heartbeats + modos de ambos vehiculos.
  params    : round-trip de parametros FWM por MAVLink (tap directo a la placa, sin WiFi).
  takeoff   : despegue de ambos y comprobacion de altitud.
  straight  : seguimiento en recta (separacion / alabeo / altitud) + serie temporal CSV.
  turn      : giro de 90 del lider (recuperacion del seguidor) + CSV.
  head_on   : (opt-in --head-on-test) el lider da media vuelta y va DE CARA al seguidor; puede provocar colision SITL.
  safety    : lider muy lejos -> el seguidor debe mantenerse acotado y sin emergencia.
  formations: EN TIERRA, recorrer TRAIL/LEFT/RIGHT/ABOVE/BELOW cambiando la formacion por el tap.

La config del periferico se hace por el TAP del bridge (MAVLink directo a la placa, mismo cable
USB): NO requiere conectar el PC a la WiFi del ESP32.

Uso:
  python tools/bench_suite.py --start-bench --firmware
  python tools/bench_suite.py        # asume banco ya levantado
  python tools/bench_suite.py --head-on-test  # incluir caso potencialmente colisivo (solo SITL)
"""
import argparse
import json
import math
import re
import socket
import statistics
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "tools" / "reports"
LEADER = "tcp:127.0.0.1:5763"
FOLLOWER = "tcp:127.0.0.1:5773"
# Enlace MAVLink DIRECTO a cada placa (tap del bridge serie) -> config del periferico SIN WiFi.
LEADER_TAP = "tcp:127.0.0.1:5790"
FOLLOWER_TAP = "tcp:127.0.0.1:5791"
FWM_COMPID = 158
FORMATIONS = ["TRAIL", "LEFT", "RIGHT", "ABOVE", "BELOW"]
BENCH_ALT = 200.0      # m - altitud de las pruebas (200 m para librar el relieve al norte del home)
R = 6371000.0


# ---------------------------------------------------------------- utilidades
def haversine(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def geo_offset(lat, lon, dist_m, brg_deg):
    br = math.radians(brg_deg)
    return (lat + dist_m * math.cos(br) / 111320.0,
            lon + dist_m * math.sin(br) / (111320.0 * math.cos(math.radians(lat))))


def ground_course(global_pos, fallback=180.0):
    """Curso sobre el suelo desde velocidad N/E (no confundir con rumbo de nariz en un giro)."""
    if global_pos and math.hypot(global_pos.vx, global_pos.vy) > 100:
        return math.degrees(math.atan2(global_pos.vy, global_pos.vx)) % 360.0
    return fallback


def drain(m):
    d = {}
    while True:
        msg = m.recv_match(blocking=False)
        if msg is None:
            break
        d[msg.get_type()] = msg
    return d


def connect(ep, src):
    m = mavutil.mavlink_connection(ep, source_system=src)
    m.wait_heartbeat(timeout=15)
    m.mav.request_data_stream_send(m.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    return m


def _pid(msg):
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode("latin-1")
    return pid.split("\0")[0]


def read_tap_text(port, seconds=4.0):
    """Lee el log serie de la placa (texto) que el bridge reenvia por el tap."""
    try:
        s = socket.create_connection(("127.0.0.1", port), 3)
    except Exception:  # noqa: BLE001
        return ""
    s.settimeout(0.4)
    buf = b""
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            d = s.recv(4096)
            if d:
                buf += d
        except Exception:  # noqa: BLE001
            pass
    try:
        s.close()
    except Exception:  # noqa: BLE001
        pass
    return "".join(chr(b) if (32 <= b < 127 or b in (10, 13)) else "." for b in buf)


class TapReader:
    """Lector no bloqueante del log serie de la placa (para contar eventos como la guarda)."""

    def __init__(self, port):
        self.s = socket.create_connection(("127.0.0.1", port), 3)
        self.s.settimeout(0.0)
        self.buf = ""

    def poll(self):
        try:
            d = self.s.recv(4096)
        except Exception:  # noqa: BLE001
            d = b""
        if d:
            self.buf += "".join(chr(b) if (32 <= b < 127 or b in (10, 13)) else "." for b in d)
        return self.buf

    def close(self):
        try:
            self.s.close()
        except Exception:  # noqa: BLE001
            pass


class FwmLink:
    """Enlace MAVLink directo a la placa FWM (tap del bridge), sin WiFi.

    Permite leer/escribir los parametros del periferico (componente FWM_COMPID) y comprobar
    el servidor de parametros (Fase 3) a traves del mismo cable USB que usa el bench.
    """

    def __init__(self, ep, sysid):
        self.m = mavutil.mavlink_connection(ep, source_system=254)
        self.sysid = sysid
        hb = self.m.wait_heartbeat(timeout=10)
        self.ok = hb is not None

    def read_params(self, timeout=6):
        self.m.mav.param_request_list_send(self.sysid, FWM_COMPID)
        got = {}
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                r = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            except Exception:  # noqa: BLE001  (socket abortado)
                break
            if r and r.get_srcComponent() == FWM_COMPID:
                got[_pid(r)] = r.param_value
        return got

    def set_param(self, name, value, timeout=2, retries=3):
        # PARAM_SET no tiene ACK separado: PARAM_VALUE es el eco. Repetir idempotentemente y
        # limitar cada espera evita falsos fallos por logs/binarios intercalados en el tap.
        for _ in range(retries):
            self.m.mav.param_set_send(self.sysid, FWM_COMPID, name.encode(),
                                      float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t0 = time.time()
            while time.time() - t0 < timeout:
                try:
                    r = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                except Exception:  # noqa: BLE001
                    return None
                if r and r.get_srcComponent() == FWM_COMPID and _pid(r) == name:
                    return r.param_value
        return None

    def close(self):
        try:
            self.m.close()
        except Exception:  # noqa: BLE001
            pass


def takeoff(m, mode=13, alt=BENCH_ALT):
    tgt = m.target_system
    print(f"[takeoff] sysid={tgt}: armando y modo {mode}...", flush=True)
    for _ in range(3):
        m.mav.param_set_send(tgt, 1, b'ARMING_CHECK', 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.2)
    m.mav.set_mode_send(tgt, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)
    time.sleep(1)
    for _ in range(8):
        m.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        time.sleep(0.8)
        hb = drain(m).get('HEARTBEAT')
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            break
    dl = time.time() + 40
    while time.time() < dl:
        g = drain(m).get('GLOBAL_POSITION_INT')
        if g and g.relative_alt / 1000.0 > 25:
            print(f"[takeoff] sysid={tgt}: en el aire ({g.relative_alt/1000:.0f} m)", flush=True)
            return True
        time.sleep(0.4)
    print(f"[takeoff] sysid={tgt}: NO alcanzo altitud", flush=True)
    return False


def set_guided(m):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)


def set_mode(m, mode):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode)


def do_reposition(m, lat, lon, alt):
    m.mav.command_int_send(m.target_system, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                           mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 0, 0, 0,
                           int(lat * 1e7), int(lon * 1e7), float(alt))


def sample_once(dl, df):
    gl, gf = dl.get('GLOBAL_POSITION_INT'), df.get('GLOBAL_POSITION_INT')
    if not gl or not gf:
        return None
    att = df.get('ATTITUDE')
    hud = df.get('VFR_HUD')
    llat, llon = gl.lat / 1e7, gl.lon / 1e7
    flat, flon = gf.lat / 1e7, gf.lon / 1e7
    return {
        "dist": haversine(llat, llon, flat, flon),
        "roll": abs(math.degrees(att.roll)) if att else None,
        "lalt": gl.relative_alt / 1000.0,
        "falt": gf.relative_alt / 1000.0,
        "fgs": hud.groundspeed if hud else None,
    }


def _stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return {}
    return {"n": len(vals), "mean": round(statistics.mean(vals), 1),
            "min": round(min(vals), 1), "max": round(max(vals), 1),
            "std": round(statistics.pstdev(vals), 1) if len(vals) > 1 else 0.0}


# ---------------------------------------------------------------- suite
class Suite:
    def __init__(self, args):
        self.args = args
        self.lead = self.fol = None
        self.results = []
        self.stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.outdir = REPORTS / self.stamp
        self.outdir.mkdir(parents=True, exist_ok=True)

    def add(self, name, status, metrics=None, notes=""):
        self.results.append({"test": name, "status": status, "metrics": metrics or {}, "notes": notes})
        self.log(f"[{status:4}] {name} {metrics if metrics else ''} {notes}")

    def log(self, msg):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)

    # ---- escenarios ----
    def test_link(self):
        lp = drain(self.lead).get('HEARTBEAT')
        fp = drain(self.fol).get('HEARTBEAT')
        m = {"leader_mode": self.lead.flightmode, "follower_mode": self.fol.flightmode}
        self.add("link", "PASS" if (lp and fp) else "FAIL", m)

    def test_takeoff(self):
        ok_l = takeoff(self.lead, 13, BENCH_ALT)
        ok_f = takeoff(self.fol, 13, BENCH_ALT)
        # Poner ambos en GUIDED antes de iniciar la ruta. El líder sube y vuela al SUR mientras el
        # seguidor ya puede seguirlo; evita dejarlo en TAKEOFF mientras el líder se aleja.
        set_guided(self.lead)
        set_guided(self.fol)
        gl = drain(self.lead).get('GLOBAL_POSITION_INT')
        if gl:
            do_reposition(self.lead, *geo_offset(gl.lat / 1e7, gl.lon / 1e7, 1500, 180.0), BENCH_ALT)
        leader_ok = False
        follower_ok = False
        t0 = time.time()
        last_log = -10.0
        settle_distance = max(40.0, (self.args.dist_offset or 96.0) + 15.0)
        while time.time() - t0 < 180:
            gl = drain(self.lead).get('GLOBAL_POSITION_INT')
            gf = drain(self.fol).get('GLOBAL_POSITION_INT')
            sep = (haversine(gl.lat / 1e7, gl.lon / 1e7, gf.lat / 1e7, gf.lon / 1e7)
                   if gl and gf else None)
            leader_ok = bool(gl and gl.relative_alt / 1000.0 >= BENCH_ALT - 10)
            if (gl and gf and leader_ok
                    and gf.relative_alt / 1000.0 >= BENCH_ALT - 15
                    and sep is not None and sep <= settle_distance):
                follower_ok = True
                break
            elapsed = time.time() - t0
            if elapsed - last_log >= 10:
                last_log = elapsed
                lalt = gl.relative_alt / 1000.0 if gl else -1
                falt = gf.relative_alt / 1000.0 if gf else -1
                sep_text = f"sep={sep:.0f}/{settle_distance:.0f}m" if sep is not None else "sep=n/d"
                self.log(f"    takeoff: lider={lalt:.0f}m seguidor={falt:.0f}m {sep_text}")
            time.sleep(0.5)
        m = {"leader_alt": round(gl.relative_alt / 1000, 1) if gl else None,
             "follower_alt": round(gf.relative_alt / 1000, 1) if gf else None,
             "separation": round(sep, 1) if sep is not None else None,
             "settle_distance": settle_distance,
             "target_alt": BENCH_ALT}
        self.add("takeoff", "PASS" if (ok_l and ok_f and leader_ok and follower_ok) else "FAIL", m)

    def _follow_segment(self, seconds, label, turn=None, warmup=20.0):
        set_guided(self.lead)
        set_guided(self.fol)
        time.sleep(2)
        g = drain(self.lead).get('GLOBAL_POSITION_INT')
        llat, llon = g.lat / 1e7, g.lon / 1e7
        do_reposition(self.lead, *geo_offset(llat, llon, 6000, 180.0), BENCH_ALT)
        switched = False
        dists, rolls, alts = [], [], []
        rows = []
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < seconds:
            dl, df = drain(self.lead), drain(self.fol)
            s = sample_once(dl, df)
            if s:
                t = round(time.time() - t0, 2)
                rows.append({"t": t, "dist": round(s["dist"], 1),
                             "roll": round(s["roll"], 1) if s["roll"] is not None else "",
                             "lalt": round(s["lalt"], 1), "falt": round(s["falt"], 1),
                             "fgs": round(s["fgs"], 1) if s["fgs"] is not None else ""})
                if t > warmup:      # regimen (descartar el transitorio inicial)
                    dists.append(s["dist"]); rolls.append(s["roll"])
                    alts.append(abs(s["lalt"] - s["falt"]))
            if turn and not switched and (time.time() - t0) > seconds * 0.5:
                gg = dl.get('GLOBAL_POSITION_INT')
                if gg:
                    do_reposition(self.lead, *geo_offset(gg.lat / 1e7, gg.lon / 1e7, 6000, 90.0), BENCH_ALT)
                switched = True
            el = time.time() - t0
            if el - last_log >= 10:
                last_log = el
                d = f"dist={s['dist']:.0f}m roll={s['roll']:.0f}" if s else "sin datos"
                self.log(f"    {label}: {int(el)}/{int(seconds)}s  {d}")
            time.sleep(0.4)
        self._write_csv(label, rows)
        m = {"dist": _stats(dists), "roll_abs": _stats(rolls), "alt_diff": _stats(alts),
             "lead_mode": self.lead.flightmode, "fol_mode": self.fol.flightmode}
        return m

    def _write_csv(self, label, rows):
        if not rows:
            return
        path = self.outdir / f"{label}.csv"
        cols = ["t", "dist", "roll", "lalt", "falt", "fgs"]
        with open(path, "w", encoding="utf-8") as f:
            f.write(",".join(cols) + "\n")
            for r in rows:
                f.write(",".join(str(r.get(c, "")) for c in cols) + "\n")

    def test_straight(self):
        m = self._follow_segment(80, "straight")
        d = m["dist"]
        # Umbral de seguridad: no declarar PASS si la separación horizontal cae a escala de
        # envergadura/colisión. En pruebas de 20m exigimos al menos 10m en toda la serie.
        min_safe = max(5.0, min(10.0, (self.args.dist_offset or 96.0) * 0.5))
        ok = bool(d) and d["mean"] < 300 and d["max"] < 600 and d["min"] >= min_safe
        m["min_safe_required"] = min_safe
        self.add("straight", "PASS" if ok else "FAIL", m)

    def test_turn(self):
        m = self._follow_segment(110, "turn", turn=True)
        d = m["dist"]
        min_safe = max(5.0, min(10.0, (self.args.dist_offset or 96.0) * 0.5))
        ok = bool(d) and d["mean"] < 400 and d["min"] >= min_safe
        m["min_safe_required"] = min_safe
        self.add("turn", "PASS" if ok else "FAIL", m)

    def test_mode_gate(self):
        """Gate de modo: con el lider en modo inestable y cerca, el seguidor NO guia (hold); al volver
        a un modo estable, reanuda. Se detecta por el log del seguidor (tap)."""
        set_guided(self.lead); set_guided(self.fol)
        time.sleep(2)
        g = drain(self.lead).get('GLOBAL_POSITION_INT')
        if not g:
            self.add("mode_gate", "SKIP", {}, "sin posicion del lider")
            return
        vh = drain(self.lead).get('VFR_HUD')
        course = ground_course(g, vh.heading if vh else 180.0)
        do_reposition(self.lead, *geo_offset(g.lat / 1e7, g.lon / 1e7, 3000, course), BENCH_ALT)
        self.log("    mode_gate: estableciendo seguimiento (25s)...")
        t0 = time.time()
        while time.time() - t0 < 25:
            drain(self.lead); drain(self.fol)
            time.sleep(0.5)
        try:
            tap = TapReader(self.args.follower_tap)
        except Exception:  # noqa: BLE001
            tap = None
        self.log("    mode_gate: lider a ACRO (inestable) y cerca...")
        set_mode(self.lead, 4)  # ACRO = inestable
        txt = ""
        t0 = time.time()
        while time.time() - t0 < 12:
            drain(self.lead); drain(self.fol)
            if tap:
                txt = tap.poll()
            time.sleep(0.5)
        held = "mode unstable" in txt.lower()
        self.log("    mode_gate: lider de vuelta a GUIDED (estable)...")
        # Marcar el cursor ANTES de cambiar el modo: la reanudación puede registrarse inmediatamente.
        if tap:
            txt = tap.poll()
        n = len(txt)
        set_guided(self.lead)
        t0 = time.time()
        while time.time() - t0 < 10:
            drain(self.lead); drain(self.fol)
            if tap:
                txt = tap.poll()
            time.sleep(0.5)
        resumed = "mode gate released" in txt[n:].lower()
        if tap:
            tap.close()
        self.add("mode_gate", "PASS" if (held and resumed) else "FAIL",
                 {"hold_con_acrobatico": held, "reanuda_con_estable": resumed,
                  "dist_offset": self.args.dist_offset})

    def test_head_on(self):
        """El lider da media vuelta y va DE CARA al seguidor: mide la separacion minima (guarda)."""
        set_guided(self.lead); set_guided(self.fol)
        time.sleep(2)
        gl = drain(self.lead).get('GLOBAL_POSITION_INT')
        gf = drain(self.fol).get('GLOBAL_POSITION_INT')
        if not gl or not gf:
            self.add("head_on", "SKIP", {}, "sin posicion de ambos vehiculos")
            return
        lalt, falt = gl.relative_alt / 1000.0, gf.relative_alt / 1000.0
        if min(lalt, falt) < BENCH_ALT - 15:
            self.log("    head_on: llevando ambos a 200m al SUR antes de separar...")
            do_reposition(self.lead, *geo_offset(gl.lat / 1e7, gl.lon / 1e7, 1000, 180.0), BENCH_ALT)
            t0 = time.time()
            while time.time() - t0 < 120:
                gl = drain(self.lead).get('GLOBAL_POSITION_INT') or gl
                gf = drain(self.fol).get('GLOBAL_POSITION_INT') or gf
                if (gl.relative_alt / 1000.0 >= BENCH_ALT - 10
                        and gf.relative_alt / 1000.0 >= BENCH_ALT - 15):
                    break
                time.sleep(0.5)
            if (gl.relative_alt / 1000.0 < BENCH_ALT - 10
                    or gf.relative_alt / 1000.0 < BENCH_ALT - 15):
                self.add("head_on", "SKIP", {"leader_alt": gl.relative_alt / 1000.0,
                                               "follower_alt": gf.relative_alt / 1000.0},
                         "no se alcanzó altitud SITL segura de 200m")
                return

        set_mode(self.fol, 12)  # LOITER: mantener al seguidor mientras el líder gana separación al SUR
        time.sleep(2)
        gl = drain(self.lead).get('GLOBAL_POSITION_INT') or gl
        gf = drain(self.fol).get('GLOBAL_POSITION_INT') or gf

        # Separar primero los aviones (seguidor en LOITER), siempre hacia el SUR del home.
        do_reposition(self.lead, *geo_offset(gl.lat / 1e7, gl.lon / 1e7, 900, 180.0), BENCH_ALT)
        self.log("    head_on: separando líder al SUR (objetivo 500m de gap)...")
        t0 = time.time()
        last_gap_log = -10.0
        gap = haversine(gl.lat / 1e7, gl.lon / 1e7, gf.lat / 1e7, gf.lon / 1e7)
        while time.time() - t0 < 60 and gap < 500.0:
            gl = drain(self.lead).get('GLOBAL_POSITION_INT') or gl
            gf = drain(self.fol).get('GLOBAL_POSITION_INT') or gf
            gap = haversine(gl.lat / 1e7, gl.lon / 1e7, gf.lat / 1e7, gf.lon / 1e7)
            elapsed = time.time() - t0
            if elapsed - last_gap_log >= 10:
                last_gap_log = elapsed
                self.log(f"    head_on: gap={gap:.0f}/500m")
            time.sleep(0.5)
        if gap < 400.0:
            self.add("head_on", "SKIP", {"initial_gap": round(gap, 1)},
                     "no se obtuvo margen de separación >=400m; no iniciar aproximación frontal")
            return

        # Activar seguimiento solo con margen, y luego ordenar al líder que vuelva de frente.
        set_guided(self.fol)
        time.sleep(1.0)
        gl = drain(self.lead).get('GLOBAL_POSITION_INT') or gl
        self.log(f"    head_on: vuelta del líder desde gap={gap:.0f}m (ambos a 200m)...")
        do_reposition(self.lead, *geo_offset(gl.lat / 1e7, gl.lon / 1e7, 3000, 0.0), BENCH_ALT)
        try:
            tap = TapReader(self.args.follower_tap)
        except Exception:  # noqa: BLE001
            tap = None
        dists, rolls = [], []
        t0 = time.time()
        last_log = 0.0
        taptext = ""
        while time.time() - t0 < 75:
            s = sample_once(drain(self.lead), drain(self.fol))
            if s:
                dists.append(s["dist"])
                if s["roll"] is not None:
                    rolls.append(s["roll"])
            if tap:
                taptext = tap.poll()
            el = time.time() - t0
            if el - last_log >= 10:
                last_log = el
                last = f"dist={dists[-1]:.0f}m (min {min(dists):.0f})" if dists else "sin datos"
                self.log(f"    head_on: {int(el)}/75s  {last}")
            time.sleep(0.5)
        guard_hits = taptext.count("HEAD_ON guard")
        faces = [int(x) for x in re.findall(r"face=(\d+)", taptext)]
        rngs = [int(x) for x in re.findall(r"rng=(\d+)m", taptext)]
        if tap:
            tap.close()
        d = _stats(dists)
        ok = bool(d) and d["min"] >= 20.0 and guard_hits > 0
        self.add("head_on", "PASS" if ok else "FAIL",
                 {"dist": d, "roll_abs": _stats(rolls), "guard_hits": guard_hits,
                  "face_max": max(faces) if faces else None,
                  "rng_min": min(rngs) if rngs else None,
                  "dbg_n": len(faces)})

    def test_safety(self):
        # Lider muy lejos: el seguidor debe mantenerse acotado (no diverger) y sin emergencia.
        set_guided(self.lead); set_guided(self.fol)
        g = drain(self.lead).get('GLOBAL_POSITION_INT')
        vh = drain(self.lead).get('VFR_HUD')
        course = ground_course(g, vh.heading if vh else 180.0)
        # Mantener curso actual para que el escenario de distancia no provoque una inversión/giro brusco.
        do_reposition(self.lead, *geo_offset(g.lat / 1e7, g.lon / 1e7, 25000, course), BENCH_ALT)
        dists = []
        rows = []
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < 60:
            s = sample_once(drain(self.lead), drain(self.fol))
            if s:
                dists.append(s["dist"])
                rows.append({"t": round(time.time() - t0, 2), "dist": round(s["dist"], 1),
                             "lalt": round(s["lalt"], 1), "falt": round(s["falt"], 1)})
            el = time.time() - t0
            if el - last_log >= 10:
                last_log = el
                self.log(f"    safety: {int(el)}/60s  dist={dists[-1]:.0f}m" if dists else f"    safety: {int(el)}/60s")
            time.sleep(0.5)
        self._write_csv("safety", rows)
        d = _stats(dists)
        ok = bool(d) and d["max"] < 6000 and d["min"] >= 10.0
        self.add("safety", "PASS" if ok else "FAIL", {"dist": d, "min_safe_required": 10.0})

    def test_preflight(self):
        """Puerta previa: comprobar por el tap que el lider TIENE FC (si no, no emitira beacons)."""
        lt = read_tap_text(self.args.leader_tap, 3.0)
        ft = read_tap_text(self.args.follower_tap, 3.0)
        leader_fc = "No FC connection" not in lt
        link_line = ""
        for line in ft.splitlines():
            if "Link:" in line:
                link_line = line.strip()
        notes = []
        if not leader_fc:
            notes.append("lider: 'No FC connection' (no emite beacons)")
        # en tierra el seguidor puede estar SEARCHING (el lider aun no se mueve): solo informativo
        ok = leader_fc
        self.add("preflight", "PASS" if ok else "FAIL",
                 {"leader_fc": leader_fc, "follower_link": link_line or "n/d"},
                 "; ".join(notes))

    def test_setup(self):
        """EN TIERRA: alinea el netid (red) en AMBOS y fija dist_offset si se pidio."""
        try:
            fl = FwmLink(f"tcp:127.0.0.1:{self.args.follower_tap}", 2)
            ll = FwmLink(f"tcp:127.0.0.1:{self.args.leader_tap}", 1)
        except Exception as e:  # noqa: BLE001
            self.add("setup", "SKIP", {}, f"tap no disponible: {e}")
            return
        info = {}
        nl = ll.read_params().get("netid")
        nf = fl.read_params().get("netid")
        want = self.args.netid
        if want is not None:
            if nl != want:
                ll.set_param("netid", want)
            if nf != want:
                fl.set_param("netid", want)
        # Verificar ambos valores después del set; no marcar PASS por haberlo intentado.
        time.sleep(0.5)
        nl = ll.read_params().get("netid")
        nf = fl.read_params().get("netid")
        info["netid_lead"] = nl
        info["netid_foll"] = nf
        info["netid_expected"] = want
        aligned = nl is not None and nl == nf and (want is None or nl == want)
        if self.args.dist_offset is not None:
            fl.set_param("dist_offset", self.args.dist_offset)
            got_offset = fl.read_params().get("dist_offset")
            info["dist_offset_readback"] = got_offset
            info["dist_offset_expected"] = self.args.dist_offset
            aligned = aligned and got_offset is not None and abs(got_offset - self.args.dist_offset) < 0.5
        fl.close(); ll.close()
        self.add("setup", "PASS" if aligned else "FAIL", info)

    def _follower_rx(self):
        """Ultimo contador rx=N del log del seguidor (paquetes VALIDOS recibidos)."""
        txt = read_tap_text(self.args.follower_tap, 4.0)
        m = re.findall(r"rx=(\d+)", txt)
        return int(m[-1]) if m else None

    @staticmethod
    def _link_counters(text):
        return [(int(rx), int(tx)) for rx, tx in
                re.findall(r"Link: [^\r\n]*?rx=(\d+) tx=(\d+)", text)]

    def test_session(self):
        """Valida JOIN/REPLY, tasa activa, timeout de sesión y recuperación (solo EN TIERRA).

        Requiere firmware FOLLOWER_REPLY=1. Cambia temporalmente el netid del seguidor para simular
        pérdida total de retorno y SIEMPRE lo restaura en finally.
        """
        hbl = drain(self.lead).get("HEARTBEAT")
        hbf = drain(self.fol).get("HEARTBEAT")
        armed = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        if ((hbl and (hbl.base_mode & armed)) or (hbf and (hbf.base_mode & armed))):
            self.add("session", "SKIP", {}, "prueba modifica netid; requiere ambos SITL desarmados/en tierra")
            return
        try:
            lead_tap = TapReader(self.args.leader_tap)
            fol_tap = TapReader(self.args.follower_tap)
            fl = FwmLink(f"tcp:127.0.0.1:{self.args.follower_tap}", 2)
        except Exception as e:  # noqa: BLE001
            self.add("session", "SKIP", {}, f"tap no disponible: {e}")
            return
        original = fl.read_params().get("netid")
        if original is None:
            fl.close(); lead_tap.close(); fol_tap.close()
            self.add("session", "FAIL", {}, "no pude leer netid del seguidor")
            return

        ltxt = ftxt = ""
        leader_osd_events = []
        active_reply = slow_after_timeout = recovered = False
        active_metrics = timeout_metrics = recovery_metrics = []
        try:
            self.log("    session: esperando JOIN/REPLY (20s)...")
            t0 = time.time(); last_log = 0.0
            while time.time() - t0 < 20:
                ltxt = lead_tap.poll(); ftxt = fol_tap.poll()
                while True:
                    stat = self.lead.recv_match(type="STATUSTEXT", blocking=False)
                    if stat is None:
                        break
                    text = stat.text.decode("latin-1", "ignore") if isinstance(stat.text, bytes) else str(stat.text)
                    leader_osd_events.append((text.rstrip("\0"), int(stat.severity),
                                              stat.get_srcSystem(), stat.get_srcComponent()))
                elapsed = time.time() - t0
                if elapsed - last_log >= 10:
                    last_log = elapsed
                    self.log(f"    session: {int(elapsed)}/20s")
                time.sleep(0.1)
            active_metrics = self._link_counters(ltxt)
            fol_active = self._link_counters(ftxt)
            active_reply = (max((x[0] for x in active_metrics), default=0) > 0
                            and max((x[1] for x in fol_active), default=0) > 0)
            active_deltas = [b[1] - a[1] for a, b in zip(active_metrics, active_metrics[1:])
                             if b[1] >= a[1]]
            active_rate = max(active_deltas, default=0)

            # Cortar solo la dirección seguidor->líder (mismatch) para comprobar timeout y fallback.
            silence_id = (int(original) + 1) & 0xFFFF
            mark = len(ltxt)
            self.log("    session: netid temporal distinto en seguidor; espero expiracion (18s)...")
            fl.set_param("netid", silence_id)
            set_back = fl.read_params().get("netid") == silence_id
            t0 = time.time()
            while time.time() - t0 < 18:
                ltxt = lead_tap.poll(); ftxt = fol_tap.poll()
                time.sleep(0.1)
            timeout_metrics = self._link_counters(ltxt[mark:])
            slow_deltas = [b[1] - a[1] for a, b in zip(timeout_metrics, timeout_metrics[1:])
                           if b[1] >= a[1]]
            slow_after_timeout = set_back and bool(slow_deltas) and min(slow_deltas) <= 5

            mark = len(ltxt)
            self.log("    session: restaurando netid; compruebo re-JOIN (12s)...")
            fl.set_param("netid", original)
            restore_ok = fl.read_params().get("netid") == original
            t0 = time.time()
            while time.time() - t0 < 12:
                ltxt = lead_tap.poll(); ftxt = fol_tap.poll()
                time.sleep(0.1)
            recovery_metrics = self._link_counters(ltxt[mark:])
            recovery_rx = [x[0] for x in recovery_metrics]
            recovered = restore_ok and len(recovery_rx) >= 2 and recovery_rx[-1] > recovery_rx[0]
        finally:
            # Pase lo que pase, no dejar las placas en redes diferentes.
            fl.set_param("netid", original)
            fl.close(); lead_tap.close(); fol_tap.close()

        active_deltas = [b[1] - a[1] for a, b in zip(active_metrics, active_metrics[1:])
                         if b[1] >= a[1]]
        active_rate = max(active_deltas, default=0)
        ok = active_reply and active_rate >= 5 and slow_after_timeout and recovered
        self.add("session", "PASS" if ok else "FAIL",
                 {"reply_bidireccional": active_reply, "max_beacon_delta_active": active_rate,
                  "discovery_after_timeout": slow_after_timeout, "rejoin_after_restore": recovered,
                  "restored_netid": original})
        osd_events = [e for e in leader_osd_events if "Follower " in e[0]]
        duplicate_pairs = sum(1 for a, b in zip(osd_events, osd_events[1:])
                              if a[0] == b[0] and a[1] == b[1])
        osd_warning = any(e[1] == mavutil.mavlink.MAV_SEVERITY_WARNING for e in osd_events)
        osd_status = "PASS" if (osd_events and osd_warning and duplicate_pairs == 0) else (
            "FAIL" if osd_events else "SKIP")
        self.add("leader_osd", osd_status,
                 {"statustext_count": len(leader_osd_events), "follower_distance_messages": len(osd_events),
                  "warning_severity_for_mp_hud": osd_warning, "duplicate_pairs": duplicate_pairs,
                  "sources": sorted(set((e[2], e[3]) for e in osd_events))},
                 "STATUSTEXT puede aparecer en Messages aunque MP filtre subcomponentes en el HUD" if osd_events else
                 "el FC/GCS no reenviò STATUSTEXT del componente FWM")

    def test_netid(self):
        """Filtro de red (netid): mismo id -> recibe beacons; distinto -> deja de recibirlos.

        Se hace EN TIERRA (netid es ground-only). Mide el contador rx del seguidor por el tap.
        """
        try:
            fl = FwmLink(f"tcp:127.0.0.1:{self.args.follower_tap}", 2)
            ll = FwmLink(f"tcp:127.0.0.1:{self.args.leader_tap}", 1)
        except Exception as e:  # noqa: BLE001
            self.add("netid", "SKIP", {}, f"tap no disponible: {e}")
            return
        orig_l = ll.read_params().get("netid")
        orig_f = fl.read_params().get("netid")
        if orig_l is None or orig_f is None:
            fl.close(); ll.close()
            self.add("netid", "FAIL", {}, "no se pudieron leer ambos netid; no modifico NVS")
            return
        same = orig_l == orig_f
        new = 0x2B2B
        rx_a = rx_b = rx_c = rx_d = None
        linked = dropped = False
        restore_ok = False
        try:
            ll.set_param("netid", new)
            fl.set_param("netid", new)
            time.sleep(1)
            both_new = (ll.read_params().get("netid") == new
                        and fl.read_params().get("netid") == new)
            if both_new:
                time.sleep(6)
                rx_a = self._follower_rx()
                time.sleep(6)
                rx_b = self._follower_rx()
                linked = rx_a is not None and rx_b is not None and rx_b > rx_a
                fl.set_param("netid", 0x2B2C)  # probar rechazo de red distinta
                time.sleep(1)
                if fl.read_params().get("netid") == 0x2B2C:
                    time.sleep(6)
                    rx_c = self._follower_rx()
                    time.sleep(6)
                    rx_d = self._follower_rx()
                    dropped = rx_c is not None and rx_d is not None and rx_d <= rx_c + 1
        finally:
            # Restauración incondicional: nunca dejar el banco en redes diferentes.
            ll.set_param("netid", orig_l)
            fl.set_param("netid", orig_f)
            time.sleep(0.5)
            restore_ok = (ll.read_params().get("netid") == orig_l
                          and fl.read_params().get("netid") == orig_f)
            fl.close(); ll.close()
        ok = same and linked and dropped and restore_ok
        self.add("netid", "PASS" if ok else "FAIL",
                 {"mismo_id": same, "id_nuevo": new, "enlaza": linked, "corta_al_cambiar": dropped,
                  "restaurado": restore_ok, "rx": [rx_a, rx_b, rx_c, rx_d]})

    def test_params(self):
        """Round-trip de parametros FWM por MAVLink (tap): leer, escribir, releer y restaurar."""
        try:
            fl = FwmLink(FOLLOWER_TAP, 2)
        except Exception as e:  # noqa: BLE001
            self.add("params", "SKIP", {}, f"tap del seguidor no disponible: {e}")
            return
        if not fl.ok:
            fl.close()
            self.add("params", "SKIP", {}, "sin heartbeat del periferico por el tap (¿placa conectada?)")
            return
        before = fl.read_params()
        probe = 111.0
        original = before.get("dist_offset", 96.0)
        try:
            fl.set_param("dist_offset", probe)
            after = fl.read_params()
            wet = after.get("dist_offset")
            ok = wet is not None and abs(wet - probe) < 0.5
        finally:
            # Restaurar incluso si falla el eco o el readback.
            fl.set_param("dist_offset", original)
            restored = fl.read_params().get("dist_offset")
        fl.close()
        ok = bool(before) and ok and restored is not None and abs(restored - original) < 0.5
        self.add("params", "PASS" if ok else "FAIL",
                 {"count": len(before), "dist_offset_set": probe,
                  "dist_offset_read": round(wet, 1) if wet is not None else None,
                  "restored": round(restored, 1) if restored is not None else None})

    def test_formations(self):
        """Valida las formaciones EN TIERRA (son ground-only por diseno) cambiandolas por el tap."""
        try:
            fl = FwmLink(FOLLOWER_TAP, 2)
        except Exception as e:  # noqa: BLE001
            self.add("formations", "SKIP", {}, f"tap del seguidor no disponible: {e}")
            return
        if not fl.ok:
            fl.close()
            self.add("formations", "SKIP", {}, "sin heartbeat del periferico por el tap")
            return
        ok_all = True
        for i, name in enumerate(FORMATIONS):
            fl.set_param("formation", i)
            back = None
            for _ in range(3):
                back = fl.read_params().get("formation")
                if back is not None and abs(back - i) < 0.5:
                    break
                fl.set_param("formation", i)
            ok = back is not None and abs(back - i) < 0.5
            self.add(f"formation_{name}", "PASS" if ok else "FAIL",
                     {"set": i, "readback": back})
            ok_all = ok_all and ok
        fl.set_param("formation", 0)   # restaurar TRAIL
        restored = fl.read_params().get("formation")
        fl.close()
        ok_all = ok_all and restored is not None and abs(restored) < 0.5
        self.add("formations", "PASS" if ok_all else "FAIL", {"count": len(FORMATIONS)})

    def _safe(self, name, fn):
        """Ejecuta un escenario aislando excepciones (un fallo no debe abortar el reporte)."""
        t0 = time.time()
        try:
            fn()
        except Exception as e:  # noqa: BLE001
            self.add(name, "FAIL", {}, f"excepcion: {e}")
        self.log(f"    ({name} en {time.time() - t0:.0f}s)")

    def run(self):
        self.log(f"Reporte -> {self.outdir}")
        self.log("conectando al banco (SITL)...")
        self.lead = connect(LEADER, 255)
        self.fol = connect(FOLLOWER, 253)
        only = None
        if getattr(self.args, "only", None):
            only = {x.strip() for x in self.args.only.split(",") if x.strip()}
        # (nombre, funcion, duracion estimada en s) para mostrar progreso/ETA
        scenarios = [
            ("preflight", self.test_preflight, 8),
            ("link", self.test_link, 2),
            ("setup", self.test_setup, 10),
            ("session", self.test_session, 35),
            ("netid", self.test_netid, 30),
            ("params", self.test_params, 25),
            ("formations", self.test_formations, 45),
            ("takeoff", self.test_takeoff, 240),
            ("straight", self.test_straight, 95),
            ("turn", self.test_turn, 125),
            ("mode_gate", self.test_mode_gate, 75),
            ("head_on", self.test_head_on, 115),
            ("safety", self.test_safety, 65),
        ]
        todo = [(n, f) for n, f, _ in scenarios if not only or n in only]
        if not getattr(self.args, "netid_test", False):
            todo = [(n, f) for n, f in todo if n != "netid"]   # disruptivo: solo con --netid-test
        if getattr(self.args, "no_session_test", False):
            todo = [(n, f) for n, f in todo if n != "session"]
        if not getattr(self.args, "head_on_test", False):
            if any(n == "head_on" for n, _ in todo):
                self.add("head_on", "SKIP", {}, "caso potencialmente colisivo; usar explícitamente --head-on-test")
            todo = [(n, f) for n, f in todo if n != "head_on"]
        eta = sum(d for n, _, d in scenarios if not only or n in only)
        self.log(f"== Escenarios: {', '.join(n for n, _ in todo)} ==")
        self.log(f"== Duracion estimada: ~{eta // 60}m {eta % 60}s ==")
        t_all = time.time()
        for i, (name, fn) in enumerate(todo, 1):
            self.log(f"=== [{i}/{len(todo)}] {name} ===")
            self._safe(name, fn)
        self.log(f"== Todos los escenarios terminados en {time.time() - t_all:.0f}s ==")
        self.write_report()

    def write_report(self):
        passed = sum(1 for r in self.results if r["status"] == "PASS")
        failed = sum(1 for r in self.results if r["status"] == "FAIL")
        skipped = sum(1 for r in self.results if r["status"] == "SKIP")
        data = {"when": self.stamp, "passed": passed, "failed": failed, "skipped": skipped,
                "results": self.results}
        (self.outdir / "report.json").write_text(json.dumps(data, indent=2), encoding="utf-8")
        md = [f"# FlyWithMe bench {self.stamp}", "",
              f"PASS={passed}  FAIL={failed}  SKIP={skipped}", ""]
        for r in self.results:
            md.append(f"## {r['test']} -- {r['status']}")
            if r["metrics"]:
                md.append("```json")
                md.append(json.dumps(r["metrics"], indent=2))
                md.append("```")
            if r["notes"]:
                md.append(r["notes"])
            md.append("")
        csvs = sorted(p.name for p in self.outdir.glob("*.csv"))
        if csvs:
            md.append("## Series temporales (CSV)")
            md.append("")
            for c in csvs:
                md.append(f"- `{c}`")
            md.append("")
        (self.outdir / "report.md").write_text("\n".join(md), encoding="utf-8")
        print(f"\n# RESULTADO: PASS={passed} FAIL={failed} SKIP={skipped}")
        print(f"reporte: {self.outdir / 'report.md'}")


def main():
    ap = argparse.ArgumentParser(description="FlyWithMe bench completo")
    ap.add_argument("--start-bench", action="store_true", help="levantar el banco antes")
    ap.add_argument("--firmware", action="store_true", help="conectar tambien las placas (puentes)")
    ap.add_argument("--dist-offset", type=float, default=None,
                    help="fijar dist_offset EN TIERRA (m) para probar vuelo cercano (p. ej. 10)")
    ap.add_argument("--netid", type=int, default=4660,
                    help="netid ('frase') a alinear en AMBAS placas antes de probar (default 4660)")
    ap.add_argument("--netid-test", action="store_true",
                    help="incluir el escenario 'netid' (cambia el netid y lo restaura; disruptivo)")
    ap.add_argument("--no-session-test", action="store_true",
                    help="omitir JOIN/REPLY por slot (por defecto se prueba en tierra)")
    ap.add_argument("--head-on-test", action="store_true",
                    help="incluir el escenario frente-a-frente (solo SITL; potencialmente colisivo)")
    ap.add_argument("--leader-tap", type=int, default=5790, help="puerto del tap del lider")
    ap.add_argument("--follower-tap", type=int, default=5791, help="puerto del tap del seguidor")
    ap.add_argument("--only", default=None,
                    help="ejecutar solo estos escenarios (coma): link,takeoff,straight,turn,head_on,safety,...")
    args = ap.parse_args()
    if args.start_bench:
        import sys
        sys.path.insert(0, str(ROOT / "tools"))
        from lab import Lab, DEFAULTS
        cfg = dict(DEFAULTS); cfg["firmware"] = args.firmware
        Lab(cfg).start_bench()
    Suite(args).run()


if __name__ == "__main__":
    main()

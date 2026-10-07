#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FlyWithMe -- Banco de pruebas COMPLETO (escenarios + reporte).

Ejecuta una bateria de escenarios contra el banco (2 SITL + firmware seguidor) y genera
un reporte (Markdown + JSON) en tools/reports/<fecha>/ para su analisis posterior.

Escenarios:
  link      : heartbeats + modos de ambos vehiculos.
  takeoff   : despegue de ambos y comprobacion de altitud.
  straight  : seguimiento en recta (metrica separacion / alabeo / altitud).
  turn      : giro de 90 del lider (recuperacion del seguidor).
  safety    : lider muy lejos -> el seguidor debe mantenerse acotado y sin emergencia.
  formations: (opcional, si el AP del ESP32 es alcanzable) recorrer TRAIL/LEFT/RIGHT/ABOVE/BELOW.

Uso:
  python tools/bench_suite.py --start-bench --firmware
  python tools/bench_suite.py                 # asume banco ya levantado
"""
import argparse
import json
import math
import socket
import statistics
import time
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / "tools" / "reports"
LEADER = "tcp:127.0.0.1:5763"
FOLLOWER = "tcp:127.0.0.1:5773"
ESP_AP = "http://192.168.4.1"          # AP del seguidor (config de FWM)
FORMATIONS = ["TRAIL", "LEFT", "RIGHT", "ABOVE", "BELOW"]
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


def takeoff(m, mode=13, alt=80):
    tgt = m.target_system
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
            return True
        time.sleep(0.4)
    return False


def set_guided(m):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 15)


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


def ap_get(path, timeout=4):
    try:
        with urllib.request.urlopen(ESP_AP + path, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:  # noqa: BLE001
        return None


def ap_post(path, data, timeout=4):
    try:
        body = urllib.parse.urlencode(data).encode()
        req = urllib.request.Request(ESP_AP + path, data=body, method="POST")
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:  # noqa: BLE001
        return None


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
        print(f"  [{status:4}] {name} {metrics if metrics else ''} {notes}")

    # ---- escenarios ----
    def test_link(self):
        lp = drain(self.lead).get('HEARTBEAT')
        fp = drain(self.fol).get('HEARTBEAT')
        m = {"leader_mode": self.lead.flightmode, "follower_mode": self.fol.flightmode}
        self.add("link", "PASS" if (lp and fp) else "FAIL", m)

    def test_takeoff(self):
        ok_l = takeoff(self.lead, 13, 80)
        ok_f = takeoff(self.fol, 13, 80)
        time.sleep(3)
        gl = drain(self.lead).get('GLOBAL_POSITION_INT')
        gf = drain(self.fol).get('GLOBAL_POSITION_INT')
        m = {"leader_alt": round(gl.relative_alt / 1000, 1) if gl else None,
             "follower_alt": round(gf.relative_alt / 1000, 1) if gf else None}
        self.add("takeoff", "PASS" if (ok_l and ok_f) else "FAIL", m)

    def _follow_segment(self, seconds, label, turn=None):
        set_guided(self.lead)
        set_guided(self.fol)
        time.sleep(2)
        g = drain(self.lead).get('GLOBAL_POSITION_INT')
        llat, llon = g.lat / 1e7, g.lon / 1e7
        do_reposition(self.lead, *geo_offset(llat, llon, 6000, 180.0), 80)
        switched = False
        dists, rolls, alts, fdists = [], [], [], []
        t0 = time.time()
        while time.time() - t0 < seconds:
            dl, df = drain(self.lead), drain(self.fol)
            s = sample_once(dl, df)
            if s and time.time() - t0 > 20:      # regimen
                dists.append(s["dist"]); rolls.append(s["roll"])
                if s["roll"] is not None:
                    pass
                alts.append(abs(s["lalt"] - s["falt"]))
            if turn and not switched and (time.time() - t0) > seconds * 0.5:
                gg = dl.get('GLOBAL_POSITION_INT')
                if gg:
                    do_reposition(self.lead, *geo_offset(gg.lat / 1e7, gg.lon / 1e7, 6000, 90.0), 80)
                switched = True
            time.sleep(0.4)
        m = {"dist": _stats(dists), "roll_abs": _stats(rolls), "alt_diff": _stats(alts)}
        return m

    def test_straight(self):
        m = self._follow_segment(80, "straight")
        d = m["dist"]
        ok = bool(d) and d["mean"] < 300 and d["max"] < 600
        self.add("straight", "PASS" if ok else "FAIL", m)

    def test_turn(self):
        m = self._follow_segment(110, "turn", turn=True)
        d = m["dist"]
        ok = bool(d) and d["mean"] < 400
        self.add("turn", "PASS" if ok else "FAIL", m)

    def test_safety(self):
        # Lider muy lejos: el seguidor debe mantenerse acotado (no diverger) y sin emergencia.
        set_guided(self.lead); set_guided(self.fol)
        g = drain(self.lead).get('GLOBAL_POSITION_INT')
        do_reposition(self.lead, *geo_offset(g.lat / 1e7, g.lon / 1e7, 25000, 180.0), 80)
        dists = []
        t0 = time.time()
        while time.time() - t0 < 60:
            s = sample_once(drain(self.lead), drain(self.fol))
            if s:
                dists.append(s["dist"])
            time.sleep(0.5)
        d = _stats(dists)
        ok = bool(d) and d["max"] < 6000   # limite de seguridad (MAX_FOLLOW_DISTANCE ~5000)
        self.add("safety", "PASS" if ok else "FAIL", {"dist": d})

    def test_formations(self):
        st = ap_get("/api/stats")
        if st is None:
            self.add("formations", "SKIP", {}, "AP del ESP32 no alcanzable (conecta el WiFi FWM AP 2)")
            return
        for i, name in enumerate(FORMATIONS):
            r = ap_post("/api/params", {"formation": str(i)})
            if not r or not r.get("success"):
                self.add(f"formation_{name}", "FAIL", {}, "no se pudo fijar")
                continue
            m = self._follow_segment(45, f"form_{name}")
            self.add(f"formation_{name}", "PASS" if m["dist"] else "FAIL", m)

    def run(self):
        print(f"Reporte -> {self.outdir}")
        self.lead = connect(LEADER, 255)
        self.fol = connect(FOLLOWER, 253)
        print("== Escenarios ==")
        self.test_link()
        self.test_takeoff()
        self.test_straight()
        self.test_turn()
        self.test_safety()
        self.test_formations()
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
        (self.outdir / "report.md").write_text("\n".join(md), encoding="utf-8")
        print(f"\n# RESULTADO: PASS={passed} FAIL={failed} SKIP={skipped}")
        print(f"reporte: {self.outdir / 'report.md'}")


def main():
    ap = argparse.ArgumentParser(description="FlyWithMe bench completo")
    ap.add_argument("--start-bench", action="store_true", help="levantar el banco antes")
    ap.add_argument("--firmware", action="store_true", help="conectar tambien las placas (puentes)")
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

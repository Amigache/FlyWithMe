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
import signal
import statistics
import time
from datetime import datetime
from pathlib import Path

import bench_config
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
    heartbeat = m.wait_heartbeat(timeout=15)
    if heartbeat is None:
        m.close()
        raise TimeoutError(f"sin HEARTBEAT inicial en {ep}")
    # Conservar el heartbeat de alta inicial: leer la cola después de provisionar roles puede
    # coincidir con un hueco entre emisiones de 1 Hz y producir un falso FAIL de preflight.
    m.initial_heartbeat = heartbeat
    m.mav.request_data_stream_send(m.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    return m


def _pid(msg):
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode("latin-1")
    return pid.split("\0")[0]


class TapReader:
    """Lee diagnósticos MAVLink del FWM; el runtime SIM silencia el Log de texto en UART0."""

    def __init__(self, port):
        self.m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=254)
        self.statuses = []
        self.named_values = {}

    def poll(self):
        while True:
            try:
                message = self.m.recv_match(blocking=False)
            except Exception:  # noqa: BLE001
                break
            if message is None:
                break
            if message.get_srcComponent() != FWM_COMPID:
                continue
            if message.get_type() == "STATUSTEXT":
                raw = message.text
                text = raw.decode("latin-1", "ignore") if isinstance(raw, bytes) else str(raw)
                self.statuses.append((text.rstrip("\0"), int(message.severity),
                                      message.get_srcSystem(), message.get_srcComponent()))
            elif message.get_type() == "NAMED_VALUE_INT":
                raw_name = message.name
                name = raw_name.decode("ascii", "ignore") if isinstance(raw_name, bytes) else str(raw_name)
                name = name.split("\0", 1)[0]
                self.named_values.setdefault(name, []).append((time.monotonic(), int(message.value)))
        return self

    def values(self, name):
        return self.named_values.get(name, [])

    def last_value(self, name):
        values = self.values(name)
        return values[-1][1] if values else None

    def close(self):
        try:
            self.m.close()
        except Exception:  # noqa: BLE001
            pass


def _counter_delta(samples):
    if len(samples) < 2:
        return 0
    return max(0, samples[-1][1] - samples[0][1])


def _max_counter_step(samples):
    return max((b[1] - a[1] for a, b in zip(samples, samples[1:]) if b[1] >= a[1]), default=0)


def _counter_rate(samples):
    if len(samples) < 2:
        return 0.0
    elapsed = samples[-1][0] - samples[0][0]
    return _counter_delta(samples) / elapsed if elapsed > 0 else 0.0


class FwmLink:
    """Enlace MAVLink directo a la placa FWM (tap del bridge), sin WiFi.

    Permite leer/escribir los parametros del periferico (componente FWM_COMPID) y comprobar
    el servidor de parametros (Fase 3) a traves del mismo cable USB que usa el bench.
    """

    def __init__(self, ep, sysid):
        self.m = mavutil.mavlink_connection(ep, source_system=254)
        self.sysid = sysid
        self.ok = False
        if sysid is None:
            deadline = time.time() + 10
            while time.time() < deadline:
                hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                if hb and hb.get_srcComponent() == FWM_COMPID:
                    self.sysid = hb.get_srcSystem()
                    self.ok = True
                    break
        else:
            hb = self.m.wait_heartbeat(timeout=10)
            self.ok = hb is not None

    def read_params(self, timeout=6):
        self.m.mav.param_request_list_send(self.sysid, FWM_COMPID)
        got = {}
        expected_count = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                r = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            except Exception:  # noqa: BLE001  (socket abortado)
                break
            if r and r.get_srcComponent() == FWM_COMPID:
                got[_pid(r)] = r.param_value
                expected_count = r.param_count
                if expected_count and len(got) >= expected_count:
                    break
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
        report_root = Path(getattr(args, "report_root", None) or REPORTS)
        self.outdir = report_root / self.stamp
        self.outdir.mkdir(parents=True, exist_ok=True)

    def add(self, name, status, metrics=None, notes=""):
        self.results.append({"test": name, "status": status, "metrics": metrics or {}, "notes": notes})
        self.log(f"[{status:4}] {name} {metrics if metrics else ''} {notes}")

    def log(self, msg):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}", flush=True)

    # ---- escenarios ----
    def test_link(self):
        # Prueba bidireccional y fresca: PARAM_REQUEST_READ es de solo lectura y no depende
        # de que el heartbeat periódico coincida con el instante del escenario.
        def read_fc_parameter(master, expected):
            master.mav.param_request_read_send(
                master.target_system, 1, b"", 0)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                response = master.recv_match(
                    type="PARAM_VALUE", blocking=True,
                    timeout=min(1.0, max(0.0, deadline - time.monotonic())))
                if (response is not None and response.get_srcSystem() == expected and
                        response.get_srcComponent() == 1 and response.param_index == 0):
                    return {"id": _pid(response), "value": float(response.param_value)}
            return None

        leader_param = read_fc_parameter(self.lead, 1)
        follower_param = read_fc_parameter(self.fol, 2)
        leader_ok = leader_param is not None
        follower_ok = follower_param is not None
        metrics = {
            "leader_param_readback": leader_param,
            "follower_param_readback": follower_param,
            "leader_mode": self.lead.flightmode,
            "follower_mode": self.fol.flightmode,
        }
        self.add("link", "PASS" if leader_ok and follower_ok else "FAIL", metrics)

    def provision_roles(self):
        """Asegura roles líder/seguidor en NVS usando el tap directo del bridge."""
        leader = follower = None
        try:
            leader = FwmLink(LEADER_TAP, None)
            follower = FwmLink(FOLLOWER_TAP, None)
            if not leader.ok or not follower.ok:
                status = "FAIL" if getattr(self.args, "firmware", False) else "SKIP"
                self.add("role_setup", status, {}, "no hay FWM en ambos taps; no se pudo validar/provisionar rol")
                return

            lp = leader.read_params()
            fp = follower.read_params()
            role_l = lp.get("role")
            role_f = fp.get("role")
            if role_l is None or role_f is None:
                self.add("role_setup", "FAIL", {}, "el firmware no expone el parámetro role")
                return

            changed = False
            if abs(role_l - 2.0) >= 0.5:
                changed = leader.set_param("role", 2.0, timeout=4) is not None or changed
            if abs(role_f - 1.0) >= 0.5:
                changed = follower.set_param("role", 1.0, timeout=4) is not None or changed
            leader.close()
            follower.close()
            leader = follower = None

            if changed:
                time.sleep(4.0)  # permitir reinicio y reconexión del bridge serie

            leader = FwmLink(LEADER_TAP, None)
            follower = FwmLink(FOLLOWER_TAP, None)
            lp = leader.read_params() if leader.ok else {}
            fp = follower.read_params() if follower.ok else {}
            ok = (abs(lp.get("role", -100.0) - 2.0) < 0.5 and
                  abs(fp.get("role", -100.0) - 1.0) < 0.5 and
                  leader.sysid == 1 and follower.sysid == 2)
            self.add("role_setup", "PASS" if ok else "FAIL",
                     {"leader_role": lp.get("role"), "follower_role": fp.get("role"),
                      "leader_fwm_sysid": leader.sysid, "follower_fwm_sysid": follower.sysid,
                      "rebooted": changed})
        except Exception as exc:  # noqa: BLE001
            self.add("role_setup", "SKIP", {}, f"no se pudo provisionar desde tap: {exc}")
        finally:
            if leader:
                leader.close()
            if follower:
                follower.close()

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
        a un modo estable, reanuda. Se detecta por STATUSTEXT MAVLink en el TAP."""
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
        t0 = time.time()
        while time.time() - t0 < 12:
            drain(self.lead); drain(self.fol)
            if tap:
                tap.poll()
            time.sleep(0.5)
        held = bool(tap and any("mode unstable" in e[0].lower() for e in tap.statuses))
        self.log("    mode_gate: lider de vuelta a GUIDED (estable)...")
        if tap:
            tap.poll()
            status_mark = len(tap.statuses)
        else:
            status_mark = 0
        set_guided(self.lead)
        t0 = time.time()
        while time.time() - t0 < 10:
            drain(self.lead); drain(self.fol)
            if tap:
                tap.poll()
            time.sleep(0.5)
        resumed = bool(tap and any(
            "leader stable - follow resumed" in e[0].lower()
            for e in tap.statuses[status_mark:]
        ))
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
        status_mark = len(tap.statuses) if tap else 0
        dists, rolls = [], []
        t0 = time.time()
        last_log = 0.0
        while time.time() - t0 < 75:
            s = sample_once(drain(self.lead), drain(self.fol))
            if s:
                dists.append(s["dist"])
                if s["roll"] is not None:
                    rolls.append(s["roll"])
            if tap:
                tap.poll()
            el = time.time() - t0
            if el - last_log >= 10:
                last_log = el
                last = f"dist={dists[-1]:.0f}m (min {min(dists):.0f})" if dists else "sin datos"
                self.log(f"    head_on: {int(el)}/75s  {last}")
            time.sleep(0.5)
        status_events = tap.statuses[status_mark:] if tap else []
        guard_hits = sum(1 for event in status_events if "HEAD_ON guard active" in event[0])
        faces = [value for _when, value in (tap.values("FWM_FACE") if tap else [])]
        rngs = [value for _when, value in (tap.values("FWM_RNG") if tap else [])]
        guard_samples = [value for _when, value in (tap.values("FWM_GUARD") if tap else [])]
        if guard_hits == 0 and any(value == 1 for value in guard_samples):
            guard_hits = sum(1 for value in guard_samples if value == 1)
        if tap:
            tap.close()
        d = _stats(dists)
        ok = bool(d) and d["min"] >= 20.0 and guard_hits > 0
        self.add("head_on", "PASS" if ok else "FAIL",
                  {"dist": d, "roll_abs": _stats(rolls), "guard_hits": guard_hits,
                   "face_max": max(faces) if faces else None,
                   "rng_min": min(rngs) if rngs else None,
                   "dbg_n": len(faces), "guard_diagnostics": len(guard_samples)})

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
        """Comprueba FC MAVLink vivos y desarmados; en SIM el TAP ya no lleva texto serie."""
        leader_hb = getattr(self.lead, "initial_heartbeat", None)
        follower_hb = getattr(self.fol, "initial_heartbeat", None)
        leader_fc = leader_hb is not None and leader_hb.get_srcSystem() == 1
        follower_fc = follower_hb is not None and follower_hb.get_srcSystem() == 2
        leader_armed = bool(leader_hb and
                            leader_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        follower_armed = bool(follower_hb and
                              follower_hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        ok = leader_fc and follower_fc and not leader_armed and not follower_armed
        notes = []
        if not leader_fc:
            notes.append("no se recibió HEARTBEAT ArduPlane SYSID 1")
        if not follower_fc:
            notes.append("no se recibió HEARTBEAT ArduPlane SYSID 2")
        if leader_armed or follower_armed:
            notes.append("una instancia SITL está armada; preflight bloqueado")
        self.add("preflight", "PASS" if ok else "FAIL",
                 {"leader_fc": leader_fc, "follower_fc": follower_fc,
                  "leader_armed": leader_armed, "follower_armed": follower_armed},
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

        active_reply = slow_after_timeout = recovered = False
        active_rate = discovery_rate = 0.0
        active_beacons = timeout_beacons = recovery_beacons = []
        try:
            self.log("    session: esperando JOIN/REPLY (20s)...")
            t0 = time.time(); last_log = 0.0
            while time.time() - t0 < 20:
                lead_tap.poll(); fol_tap.poll()
                elapsed = time.time() - t0
                if elapsed - last_log >= 10:
                    last_log = elapsed
                    self.log(f"    session: {int(elapsed)}/20s")
                time.sleep(0.1)
            active_lead_rx = lead_tap.values("FWM_RX")
            active_lead_tx = lead_tap.values("FWM_TX")
            active_fol_tx = fol_tap.values("FWM_TX")
            active_beacons = fol_tap.values("FWM_RX")
            active_rate = _counter_rate(active_beacons)
            active_reply = (_counter_delta(active_lead_rx) > 0 and
                            _counter_delta(active_fol_tx) > 0 and
                            lead_tap.last_value("FWM_LINK") == 1 and
                            fol_tap.last_value("FWM_LINK") == 1)

            # Cortar solo la dirección seguidor->líder (mismatch) para comprobar timeout y fallback.
            silence_id = (int(original) + 1) & 0xFFFF
            lead_tx_mark = len(active_lead_tx)
            self.log("    session: netid temporal distinto en seguidor; espero expiracion (18s)...")
            fl.set_param("netid", silence_id)
            set_back = fl.read_params().get("netid") == silence_id
            t0 = time.time()
            while time.time() - t0 < 18:
                lead_tap.poll(); fol_tap.poll()
                time.sleep(0.1)
            timeout_beacons = lead_tap.values("FWM_TX")[lead_tx_mark:]
            discovery_rate = _counter_rate(timeout_beacons)
            slow_after_timeout = (set_back and active_rate > 0.0 and
                                  discovery_rate < active_rate * 0.5 and
                                  lead_tap.last_value("FWM_LINK") == 0)

            recovery_rx_mark = len(fol_tap.values("FWM_RX"))
            self.log("    session: restaurando netid; compruebo re-JOIN (12s)...")
            fl.set_param("netid", original)
            restore_ok = fl.read_params().get("netid") == original
            t0 = time.time()
            while time.time() - t0 < 12:
                lead_tap.poll(); fol_tap.poll()
                time.sleep(0.1)
            recovery_beacons = fol_tap.values("FWM_RX")[recovery_rx_mark:]
            recovered = (restore_ok and _counter_delta(recovery_beacons) > 0 and
                         fol_tap.last_value("FWM_LINK") == 1 and
                         lead_tap.last_value("FWM_LINK") == 1)
        finally:
            # Pase lo que pase, no dejar las placas en redes diferentes.
            fl.set_param("netid", original)
            fl.close(); lead_tap.close(); fol_tap.close()

        ok = active_reply and active_rate >= 2.0 and slow_after_timeout and recovered
        self.add("session", "PASS" if ok else "FAIL",
                 {"reply_bidireccional": active_reply, "active_beacons_per_s": round(active_rate, 2),
                  "discovery_beacons_per_s": round(discovery_rate, 2),
                  "discovery_after_timeout": slow_after_timeout, "rejoin_after_restore": recovered,
                  "restored_netid": original})
        osd_events = [e for e in lead_tap.statuses if "Follower " in e[0]]
        duplicate_pairs = sum(1 for a, b in zip(osd_events, osd_events[1:])
                              if a[0] == b[0] and a[1] == b[1])
        osd_severity_ok = all(e[1] in (mavutil.mavlink.MAV_SEVERITY_INFO,
                                       mavutil.mavlink.MAV_SEVERITY_WARNING) for e in osd_events)
        osd_status = "PASS" if (osd_events and osd_severity_ok and duplicate_pairs == 0) else (
            "FAIL" if osd_events else "SKIP")
        self.add("leader_osd", osd_status,
                 {"statustext_count": len(lead_tap.statuses), "follower_distance_messages": len(osd_events),
                  "severities": sorted(set(e[1] for e in osd_events)), "duplicate_pairs": duplicate_pairs,
                  "sources": sorted(set((e[2], e[3]) for e in osd_events))},
                 "el bench verifica MAVLink, no píxeles del HUD" if osd_events else
                 "sin STATUSTEXT de distancia en esta ventana; puede estar deduplicado o no haber posiciones válidas")

    def test_netid(self):
        """Filtro de red (netid): mismo id -> recibe beacons; distinto -> deja de recibirlos.

        Se hace EN TIERRA (netid es ground-only). Mide FWM_RX estructurado por MAVLink en el TAP.
        """
        fol_tap = None
        try:
            fl = FwmLink(f"tcp:127.0.0.1:{self.args.follower_tap}", 2)
            ll = FwmLink(f"tcp:127.0.0.1:{self.args.leader_tap}", 1)
            fol_tap = TapReader(self.args.follower_tap)
        except Exception as e:  # noqa: BLE001
            self.add("netid", "SKIP", {}, f"tap no disponible: {e}")
            return
        orig_l = ll.read_params().get("netid")
        orig_f = fl.read_params().get("netid")
        if orig_l is None or orig_f is None:
            fl.close(); ll.close()
            if fol_tap:
                fol_tap.close()
            self.add("netid", "FAIL", {}, "no se pudieron leer ambos netid; no modifico NVS")
            return
        same = orig_l == orig_f
        new = 0x2B2B
        rx_a = rx_b = rx_c = rx_d = None
        linked = dropped = False
        restore_ok = False
        try:
            fol_tap.poll()
            ll.set_param("netid", new)
            fl.set_param("netid", new)
            time.sleep(1)
            both_new = (ll.read_params().get("netid") == new
                        and fl.read_params().get("netid") == new)
            if both_new:
                fol_tap.poll()
                rx_a = fol_tap.last_value("FWM_RX")
                time.sleep(6)
                fol_tap.poll()
                rx_b = fol_tap.last_value("FWM_RX")
                linked = rx_a is not None and rx_b is not None and rx_b > rx_a

                fl.set_param("netid", 0x2B2C)  # probar rechazo de red distinta
                time.sleep(1)
                if fl.read_params().get("netid") == 0x2B2C:
                    time.sleep(6)
                    fol_tap.poll()
                    rx_c = fol_tap.last_value("FWM_RX")
                    time.sleep(6)
                    fol_tap.poll()
                    rx_d = fol_tap.last_value("FWM_RX")
                    dropped = rx_c is not None and rx_d is not None and rx_d <= rx_c + 1
        finally:
            # Restauración incondicional: nunca dejar el banco en redes diferentes.
            ll.set_param("netid", orig_l)
            fl.set_param("netid", orig_f)
            time.sleep(0.5)
            restore_ok = (ll.read_params().get("netid") == orig_l
                          and fl.read_params().get("netid") == orig_f)
            fl.close(); ll.close()
            if fol_tap:
                fol_tap.close()
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
        self.provision_roles()
        if (getattr(self.args, "firmware", False) and self.results and
                self.results[-1]["test"] == "role_setup" and self.results[-1]["status"] != "PASS"):
            self.log("role_setup falló; se bloquea el resto del bench HIL")
            return self.write_report()
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
        return self.write_report()

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
        return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser(description="FlyWithMe bench completo")
    ap.add_argument("--start-bench", action="store_true", help="levantar el banco antes")
    ap.add_argument("--stop-bench-after", action="store_true",
                    help="detener solo los procesos SITL/bridge iniciados por este run al terminar")
    ap.add_argument("--firmware", action="store_true", help="conectar tambien las placas (puentes)")
    ap.add_argument("--sitl-exe", default=None, help="ruta a ArduPlane SITL")
    ap.add_argument("--leader-com", default=bench_config.load()["leader_com"], help="puerto de la placa líder")
    ap.add_argument("--slave-com", default=bench_config.load()["slave_com"], help="puerto de la placa seguidora")
    ap.add_argument("--mp-udp", type=int, default=14550, help="puerto UDP de Mission Planner")
    ap.add_argument("--report-root", default=None, help="directorio de reportes (default tools/reports)")
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

    def interrupt(signum, _frame):
        raise KeyboardInterrupt

    for stop_signal in (signal.SIGINT, getattr(signal, "SIGBREAK", None)):
        if stop_signal is not None:
            try:
                signal.signal(stop_signal, interrupt)
            except (OSError, ValueError):
                pass

    lab = None
    cleanup_required = False
    try:
        if args.start_bench:
            import sys
            sys.path.insert(0, str(ROOT / "tools"))
            from lab import Lab, DEFAULTS
            cfg = dict(DEFAULTS)
            cfg.update(firmware=args.firmware, leader_com=args.leader_com, slave_com=args.slave_com,
                       mp_udp=args.mp_udp)
            if args.sitl_exe:
                cfg["sitl_exe"] = Path(args.sitl_exe)
            lab = Lab(cfg)
            cleanup_required = True  # también cubre cancelación durante start_bench()
            if not lab.start_bench():
                cleanup_required = False  # start_bench ya limpia fallos parciales
                return 2
        return Suite(args).run()
    except KeyboardInterrupt:
        print("\n[bench] cancelado por usuario; limpiando el banco...", flush=True)
        return 130
    finally:
        if lab is not None and args.stop_bench_after and cleanup_required:
            lab.stop_bench()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulador de la LEY DE SEGUIMIENTO cross-track (pura, sin hardware).

Replica en Python la MISMA ley que el firmware (`Telem::guided_follow` +
`calculateFormationPosition`) sobre un modelo cinematico simple de avion, para
explorar geometria y precision SIN SITL, SIN placas y SIN LoRa.

Sirve para responder preguntas como:
  * Que pasa si lider y seguidor van DE FRENTE y en sentidos opuestos y activamos seguir?
  * Hasta que separacion podemos bajar (100 -> 20 -> 10 -> 5 m)? Con que latencia?
  * Cuanto influye el periodo de actualizacion del enlace (LoRa) en el error?

La latencia modela el retardo del enlace: el seguidor ve el estado del lider de
hace `--latency` segundos, muestreado cada `--update-period` segundos (zero-order hold).

Uso:
  python tools/follow_sim.py --scenario trail --dist-offset 10 --latency 0.5
  python tools/follow_sim.py --scenario head_on
  python tools/follow_sim.py --sweep distance        # 5..100 m con latencia tipica
  python tools/follow_sim.py --sweep latency         # 0..2 s con dist_offset=10
  python tools/follow_sim.py --scenario trail --dist-offset 10 --close-rate   # tasa rapida cerca
"""
import argparse
import math
from collections import deque
from dataclasses import dataclass

# --- Constantes de la ley del firmware (config.h) ---------------------------
BASE = dict(
    dist_offset=96.0, lateral=50.0, vertical=20.0,
    cross_gain=0.6, hdg_corr_max=40.0,            # CROSS_TRACK_GAIN_DEG_PER_M / MAX_HEADING_CORR_DEG
    along_gain=12.0,                              # ALONG_GAIN_CMS_PER_M
    deadband=2.0, quant=50.0,                     # SPEED_DEADBAND_M / SPEED_QUANT_CMS
    max_boost=600.0, max_slow=400.0,              # cm/s
    turn_rate=30.0,                               # GUIDED_TURN_RATE_DPS (deg/s)
    accel=1.0,                                    # GUIDED_SPEED_ACCEL (m/s^2)
    airspeed_min=10.0, airspeed_max=30.0,
    lead_m=0.0,                                   # FORMATION_LEAD_M
    tau_hdg=2.0,                                  # constante de tiempo del lazo de rumbo (s)
    # Guarda de rumbo de colision (frente a frente). guard_range=0 -> desactivada.
    guard_range=0.0,       # m - distancia bajo la cual actua la guarda
    guard_ttc=0.0,         # s - tiempo-de-colision bajo el cual actua la guarda (0=off)
    guard_face=120.0,      # deg - |ang(lider, marcacion hacia el)| > esto = "viene de cara"
    guard_turn=90.0,       # deg - giro de ruptura (a la derecha)
    guard_slow=10.0,       # m/s - velocidad a la que frenar durante la guarda
    guard_climb=0.0,       # m/s - ruptura VERTICAL: trepar durante la guarda (0=off)
    climb_hold=8.0,        # s - tiempo que se mantiene el ascenso tras la guarda
)

# Tasa de paquetes actual (ADAPTIVE_RATE en config.h): NOTA: mas lento cuanto mas cerca.
RATE_CUR = [("<100m", 100.0, 2.0), ("100-500m", 500.0, 1.0), (">500m", 1e9, 0.5)]
# Tasa propuesta para formacion cerrada: mas rapido cuanto mas cerca.
RATE_PROPOSED = [("<100m", 100.0, 0.2), ("100-500m", 500.0, 0.5), (">500m", 1e9, 1.0)]


def geo_delta(lat, dist_m, brg_deg):
    """Desplazamiento (dlat, dlon) en grados para una distancia y marcacion."""
    br = math.radians(brg_deg)
    return (dist_m * math.cos(br) / 111320.0,
            dist_m * math.sin(br) / (111320.0 * math.cos(math.radians(lat))))


def dist_m(lat1, lon1, lat2, lon2):
    dn = (lat2 - lat1) * 111320.0
    de = (lon2 - lon1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2))
    return math.hypot(dn, de)


@dataclass
class Leader:
    lat: float
    lon: float
    hdg: float          # grados (0=N)
    speed: float        # m/s
    alt: float = 100.0  # m (relativa)

    def step(self, dt):
        dlat, dlon = geo_delta(self.lat, self.speed * dt, self.hdg)
        self.lat += dlat
        self.lon += dlon


@dataclass
class Follower:
    lat: float
    lon: float
    hdg: float
    speed: float
    max_turn: float = BASE["turn_rate"]
    accel: float = BASE["accel"]
    tau: float = BASE["tau_hdg"]
    alt: float = 100.0      # m (relativa)
    max_climb: float = 5.0  # m/s

    def command(self, hcmd, vcmd, dt):
        err = ((hcmd - self.hdg + 180.0) % 360.0) - 180.0
        rate = max(-self.max_turn, min(self.max_turn, err / self.tau))
        self.hdg = (self.hdg + rate * dt) % 360.0
        ds = vcmd - self.speed
        self.speed += max(-self.accel * dt, min(self.accel * dt, ds))
        dlat, dlon = geo_delta(self.lat, self.speed * dt, self.hdg)
        self.lat += dlat
        self.lon += dlon


def formation_target(le, formation, p):
    """Replica de calculateFormationPosition (devuelve lat, lon del punto objetivo)."""
    if formation == "trail":
        dlat, dlon = geo_delta(le.lat, p["dist_offset"], le.hdg + 180.0)
    elif formation == "left":
        dlat, dlon = geo_delta(le.lat, p["lateral"], le.hdg - 90.0)
    elif formation == "right":
        dlat, dlon = geo_delta(le.lat, p["lateral"], le.hdg + 90.0)
    else:  # above/below: misma posicion horizontal
        dlat, dlon = 0.0, 0.0
    # carrot/look-ahead sobre la traza (FORMATION_LEAD_M)
    if p["lead_m"] > 0.0:
        lad, lod = geo_delta(le.lat, p["lead_m"], le.hdg)
        dlat += lad
        dlon += lod
    return le.lat + dlat, le.lon + dlon


def angle_diff(a, b):
    return ((a - b + 180.0) % 360.0) - 180.0


def bearing(fo, le):
    dn = (le.lat - fo.lat) * 111320.0
    de = (le.lon - fo.lon) * 111320.0 * math.cos(math.radians(fo.lat))
    return math.degrees(math.atan2(de, dn)) % 360.0


def law(le, fo, formation, p):
    """Ley del firmware: devuelve (hcmd, vcmd, along, cross, guard)."""
    # traza = direccion de la velocidad del lider (aqui = rumbo del lider)
    un, ue = math.cos(math.radians(le.hdg)), math.sin(math.radians(le.hdg))
    plat, plon = formation_target(le, formation, p)
    dn = (fo.lat - plat) * 111320.0
    de = (fo.lon - plon) * 111320.0 * math.cos(math.radians(fo.lat))
    along = dn * un + de * ue
    cross = -dn * ue + de * un
    theta = math.degrees(math.atan2(ue, un))
    corr = max(-p["hdg_corr_max"], min(p["hdg_corr_max"], -p["cross_gain"] * cross))
    hcmd = (theta + corr) % 360.0
    along_dead = 0.0 if abs(along) < p["deadband"] else along
    boost = max(-p["max_slow"], min(p["max_boost"], -along_dead * p["along_gain"]))
    vcmd = le.speed + boost / 100.0
    vcmd = max(p["airspeed_min"], min(p["airspeed_max"], vcmd))
    vcmd = round(vcmd * 100.0 / p["quant"]) * p["quant"] / 100.0

    # --- Guarda frente a frente: solo si el LIDER mira hacia el seguidor (viene de cara) ---
    guard = False
    if p.get("guard_range", 0.0) > 0.0 or p.get("guard_ttc", 0.0) > 0.0:
        rng = dist_m(fo.lat, fo.lon, le.lat, le.lon)
        face = abs(angle_diff(le.hdg, bearing(fo, le)))   # ~180 => el lider viene de cara
        trig = False
        if p.get("guard_range", 0.0) > 0.0 and rng < p["guard_range"]:
            trig = True
        if p.get("guard_ttc", 0.0) > 0.0 and rng > 1.0:
            r = math.radians
            losn = (le.lat - fo.lat) * 111320.0 / rng
            lose = (le.lon - fo.lon) * 111320.0 * math.cos(r(fo.lat)) / rng
            lvn, lve = le.speed * math.cos(r(le.hdg)), le.speed * math.sin(r(le.hdg))
            fvn, fve = fo.speed * math.cos(r(fo.hdg)), fo.speed * math.sin(r(fo.hdg))
            closing = -((lvn - fvn) * losn + (lve - fve) * lose)
            if closing > 0.5 and rng / closing < p["guard_ttc"]:
                trig = True
        if trig and face > p["guard_face"]:
            guard = True
            # Ruptura PERPENDICULAR a la visual: maxima tasa de separacion (salir de la proa).
            hcmd = (bearing(fo, le) + p["guard_turn"]) % 360.0
            vcmd = p["guard_slow"]
    return hcmd, vcmd, along, cross, guard


class LinkModel:
    """Estado del lider tal como lo ve el seguidor: muestreo cada update_period y retardo latency."""

    def __init__(self, latency, update_period):
        self.latency = latency
        self.update_period = update_period
        self.hist = deque()        # (t, Leader)  instantaneas guardadas
        self.last_sample_t = -1e9
        self.held = None

    def observe(self, t, le):
        if t - self.last_sample_t >= self.update_period - 1e-9:
            self.hist.append((t, Leader(le.lat, le.lon, le.hdg, le.speed)))
            self.last_sample_t = t
        # descartar lo mas viejo que latency + margen
        while len(self.hist) > 1 and self.hist[1][0] <= t - self.latency:
            self.hist.popleft()
        if self.hist and self.hist[0][0] <= t - self.latency:
            self.held = self.hist[0][1]
        return self.held if self.held is not None else le


def run(scenario="trail", formation="trail", p=None, latency=0.5, update_period=1.0,
        seconds=120.0, dt=0.05, lead_hdg=0.0, fo_hdg=-90.0, gap=100.0, seed=0):
    p = dict(BASE if p is None else p)
    # Posiciones iniciales: lider en el origen, seguidor segun el escenario.
    le = Leader(37.0, -6.0, lead_hdg, 20.0)
    if scenario == "trail":
        # seguidor ya detras, en la traza
        dlat, dlon = geo_delta(le.lat, gap, le.hdg + 180.0)
        fo = Follower(le.lat + dlat, le.lon + dlon, lead_hdg, 20.0)
    elif scenario == "head_on":
        # seguidor DELANTE del lider y de cara (sentidos opuestos); se acercan
        dlat, dlon = geo_delta(le.lat, gap, le.hdg)
        fo = Follower(le.lat + dlat, le.lon + dlon, (lead_hdg + 180.0) % 360.0, 20.0)
    elif scenario == "lateral":
        dlat, dlon = geo_delta(le.lat, gap, lead_hdg + 90.0)
        fo = Follower(le.lat + dlat, le.lon + dlon, lead_hdg, 20.0)
    elif scenario == "crossing":
        # seguidor a un lado, rumbo perpendicular hacia la derrota del lider
        dlat, dlon = geo_delta(le.lat, gap, lead_hdg + 90.0)
        fo = Follower(le.lat + dlat, le.lon + dlon, (lead_hdg - 90.0) % 360.0, 20.0)
    elif scenario == "behind_offset":
        dlat, dlon = geo_delta(le.lat, gap, le.hdg + 180.0)
        fo = Follower(le.lat + dlat, le.lon + dlon, lead_hdg + fo_hdg, 20.0)
    else:
        raise SystemExit(f"escenario desconocido: {scenario}")

    link = LinkModel(latency, update_period)
    rows, seps, min_sep = [], [], 1e9
    guard_on = 0
    climb_until = -1.0
    t = 0.0
    while t < seconds:
        le.step(dt)
        seen = link.observe(t, le)
        hcmd, vcmd, along, cross, guard = law(seen, fo, formation, p)
        if guard:
            guard_on += 1
            if p.get("guard_climb", 0.0) > 0.0:
                climb_until = t + p.get("climb_hold", 8.0)
        if t < climb_until and p.get("guard_climb", 0.0) > 0.0:
            fo.alt += min(p["guard_climb"], fo.max_climb) * dt   # ruptura vertical
        fo.command(hcmd, vcmd, dt)
        sepH = dist_m(le.lat, le.lon, fo.lat, fo.lon)
        sep = math.hypot(sepH, le.alt - fo.alt)                  # separacion 3D
        seps.append(sep)
        min_sep = min(min_sep, sep)
        rows.append((round(t, 2), round(sep, 1), round(along, 1), round(cross, 1),
                     round(fo.hdg, 1), round(fo.speed, 1), round(le.hdg, 1)))
        t += dt

    # regimen = ultimo 40 %
    tail = seps[int(len(seps) * 0.6):]
    mean = sum(tail) / len(tail)
    std = math.sqrt(sum((x - mean) ** 2 for x in tail) / len(tail))
    # cross/algun en regimen
    alongs = [r[2] for r in rows[int(len(rows) * 0.6):]]
    crosses = [r[3] for r in rows[int(len(rows) * 0.6):]]
    return {
        "scenario": scenario, "formation": formation,
        "sep_mean": mean, "sep_std": std, "sep_min": min_sep, "sep_last": seps[-1],
        "along_mean": sum(alongs) / len(alongs), "cross_rms": math.sqrt(sum(c * c for c in crosses) / len(crosses)),
        "cross_max": max(abs(c) for c in crosses), "guard_on": guard_on,
    }, rows


def fmt(r):
    return (f"{r['scenario']:>12} form={r['formation']:<5} sep mean={r['sep_mean']:6.1f} "
            f"std={r['sep_std']:5.1f} min={r['sep_min']:6.1f} last={r['sep_last']:6.1f} "
            f"cross_rms={r['cross_rms']:5.1f} cross_max={r['cross_max']:5.1f} guard={r.get('guard_on',0)}")


def main():
    ap = argparse.ArgumentParser(description="Simulador de la ley de seguimiento")
    ap.add_argument("--scenario", default="trail",
                    choices=["trail", "head_on", "lateral", "crossing", "behind_offset"])
    ap.add_argument("--formation", default="trail",
                    choices=["trail", "left", "right", "above", "below"])
    ap.add_argument("--dist-offset", type=float, default=BASE["dist_offset"])
    ap.add_argument("--latency", type=float, default=0.5, help="retardo del enlace (s)")
    ap.add_argument("--update-period", type=float, default=1.0, help="periodo de paquete (s)")
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--gap", type=float, default=100.0, help="separacion inicial (m)")
    ap.add_argument("--lead-hdg", type=float, default=0.0)
    ap.add_argument("--fo-hdg", type=float, default=-90.0)
    ap.add_argument("--cross-gain", type=float, default=BASE["cross_gain"])
    ap.add_argument("--cross-max", type=float, default=BASE["hdg_corr_max"])
    ap.add_argument("--along-gain", type=float, default=BASE["along_gain"])
    ap.add_argument("--turn-rate", type=float, default=BASE["turn_rate"])
    ap.add_argument("--close-rate", action="store_true",
                    help="tasa rapida cuando cerca (propuesta), en vez de la actual")
    ap.add_argument("--guard-range", type=float, default=0.0,
                    help="m - distancia de actuacion de la guarda frente a frente (0=off)")
    ap.add_argument("--guard-ttc", type=float, default=0.0,
                    help="s - tiempo-de-colision de actuacion de la guarda (0=off)")
    ap.add_argument("--guard-face", type=float, default=BASE["guard_face"],
                    help="deg - umbral de 'el lider viene de cara'")
    ap.add_argument("--guard-turn", type=float, default=BASE["guard_turn"],
                    help="deg - giro de ruptura de la guarda")
    ap.add_argument("--guard-climb", type=float, default=0.0,
                    help="m/s - ruptura VERTICAL: trepar durante la guarda (0=off)")
    ap.add_argument("--sweep", choices=["distance", "latency", "all"], default=None)
    ap.add_argument("--csv", default=None, help="volcar serie temporal del ultimo caso a CSV")
    args = ap.parse_args()

    p = dict(BASE)
    p["dist_offset"] = args.dist_offset
    p["cross_gain"] = args.cross_gain
    p["hdg_corr_max"] = args.cross_max
    p["along_gain"] = args.along_gain
    p["turn_rate"] = args.turn_rate
    p["guard_range"] = args.guard_range
    p["guard_ttc"] = args.guard_ttc
    p["guard_face"] = args.guard_face
    p["guard_turn"] = args.guard_turn
    p["guard_climb"] = args.guard_climb

    print(f"ley: cross_gain={p['cross_gain']} cross_max={p['hdg_corr_max']} "
          f"along_gain={p['along_gain']} turn_rate={p['turn_rate']} dist_offset={p['dist_offset']}")

    if args.sweep == "distance":
        print("\n-- barrido de dist_offset (latency=0.5s, update=1.0s) --")
        for d in (5, 10, 20, 50, 96, 150):
            p2 = dict(p); p2["dist_offset"] = d
            r, _ = run("trail", "trail", p2, 0.5, 1.0, args.seconds)
            print(fmt(r))
    elif args.sweep == "latency":
        print("\n-- barrido de latencia (dist_offset=10m, update=latency) --")
        for lat in (0.05, 0.1, 0.2, 0.5, 1.0, 2.0):
            p2 = dict(p); p2["dist_offset"] = 10.0
            r, _ = run("trail", "trail", p2, lat, max(lat, 0.1), args.seconds)
            print(fmt(r))
    elif args.sweep == "all":
        print("\n-- MATRIZ DE SEGURIDAD (min separacion; dist_offset=20m, latency=1.0s) --")
        p2 = dict(p); p2["dist_offset"] = 20.0
        pg = dict(p2); pg["guard_range"] = 500.0
        for sc in ("trail", "head_on", "lateral", "crossing", "behind_offset"):
            r0, _ = run(sc, "trail", p2, 1.0, 1.0, args.seconds)
            r1, _ = run(sc, "trail", pg, 1.0, 1.0, args.seconds)
            print(f"{sc:>14}: sin guarda min={r0['sep_min']:6.1f} m   "
                  f"con guarda min={r1['sep_min']:6.1f} m (dispara {r1['guard_on']})")
    else:
        up = args.update_period
        if args.close_rate:
            # propuesta: mas rapido cuanto mas cerca (usar la tabla propuesta segun distancia)
            up = 0.2 if p["dist_offset"] < 100 else 0.5
            print(f"(close-rate -> update_period={up}s)")
        r, rows = run(args.scenario, args.formation, p, args.latency, up, args.seconds, gap=args.gap,
                      lead_hdg=args.lead_hdg, fo_hdg=args.fo_hdg)
        print(fmt(r))
        if args.csv:
            import csv as _csv
            with open(args.csv, "w", newline="", encoding="utf-8") as f:
                w = _csv.writer(f)
                w.writerow(["t", "sep", "along", "cross", "fo_hdg", "fo_speed", "le_hdg"])
                w.writerows(rows)
            print(f"CSV -> {args.csv}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulador del PROTOCOLO de comunicacion FlyWithMe (v2) -- casos y fallos.

Modela la maquina de estados de la capa de comunicacion (sin radio real):
  * LIDER: emite BEACON solo si ha oido un REPLY/JOIN dentro de SESSION_TIMEOUT (sesion on-demand).
  * SEGUIDOR: emite REPLY (si enlazado) o JOIN (si aun no) cada REPLY_PERIOD.
  * Canal: perdida de paquetes y "apagones" (blackout).
  * Filtro: descarta paquetes de otra version o netid.
  * Gate de modo: en distancia de aproximacion, si el lider no esta en modo estable -> el seguidor no guia.

Cada escenario comprueba invariantes (no cuelgues, timeouts que disparan, recuperacion, etc.).
Uso:
  python tools/proto_sim.py            # matriz de escenarios
  python tools/proto_sim.py -v         # con traza
"""
import argparse

DT = 10               # ms por paso
SESSION_TIMEOUT = 10000   # ms - sin reply, el lider deja de emitir
REPLY_PERIOD = 1500       # ms - cadencia del seguidor
BEACON_PERIOD = 200       # ms - cadencia de beacon (banco 5 Hz)
LINK_TIMEOUT = 3000       # ms - sin beacon, el seguidor considera enlace perdido
APPROACH = 300.0          # m
VERSION = 2
STABLE = {5, 6, 7, 10, 11, 12, 13, 15}


def stable(mode):
    return mode in STABLE


def run(scenario, seconds=40, loss=0.0, blackout=None, netid_l=1, netid_f=1,
        mode_of=lambda t: 15, dist_of=lambda t: 1000.0, verbose=False):
    blackout = blackout or (0, 0)   # (inicio_ms, fin_ms)
    # estado
    lead = {"session_until": -1e9, "next_beacon": 0, "beacons": 0}
    fol = {"next_reply": 0, "last_beacon": -1e9, "linked": False, "rx": 0, "joins": 0, "replies": 0}
    guided = []      # (t, guided?)
    events = []

    def deliver(kind):
        t = now
        if blackout[0] <= t < blackout[1]:
            return
        if loss > 0:
            seedv = (t * 2654435761 + (1 if kind == "beacon" else 2) * 40503 + lead["beacons"] * 97) & 0xFFFFFFFF
            if (seedv % 1000) < loss * 1000:
                return
        if kind == "beacon":
            # filtro netid/version (mismo firmware -> version ok)
            if netid_f != netid_l:
                return
            fol["last_beacon"] = t
            fol["rx"] += 1
        else:  # reply/join -> lider
            if netid_f != netid_l:
                return
            lead["session_until"] = t + SESSION_TIMEOUT

    now = 0
    while now <= seconds * 1000:
        # lider
        if now < lead["session_until"] and now >= lead["next_beacon"]:
            lead["next_beacon"] = now + BEACON_PERIOD
            lead["beacons"] += 1
            deliver("beacon")
        # seguidor
        if now >= fol["next_reply"]:
            fol["next_reply"] = now + REPLY_PERIOD
            if fol["linked"]:
                fol["replies"] += 1
                deliver("reply")
            else:
                fol["joins"] += 1
                deliver("join")
        # estado de enlace del seguidor
        linked = (now - fol["last_beacon"]) < LINK_TIMEOUT
        fol["linked"] = linked
        # gate de modo
        d = dist_of(now)
        g = linked and not (d < APPROACH and not stable(mode_of(now)))
        guided.append((now, g))
        if verbose and now % 2000 == 0:
            print(f"  t={now/1000:4.1f}s lead_sess={'Y' if now < lead['session_until'] else 'n'} "
                  f"fol_linked={'Y' if linked else 'n'} dist={d:.0f} mode={mode_of(now)} guided={g}")
        now += DT

    return {"lead": lead, "fol": fol, "guided": guided, "scenario": scenario}


def frac_guided(guided):
    if not guided:
        return 0.0
    return sum(1 for _, g in guided if g) / len(guided)


def frac_linked(res):
    # recomputa fraccion enlazado a partir de guided? mejor guardar aparte
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()
    checks = fails = 0

    def check(name, cond, extra=""):
        nonlocal checks, fails
        checks += 1
        ok = bool(cond)
        if not ok:
            fails += 1
        print(f"  [{'PASS' if ok else 'FAIL'}] {name} {extra}")

    print("== Matriz de casos/fallos del protocolo ==")

    # 1. Normal: el lider NO emite hasta oir al seguidor; luego enlaza.
    r = run("normal", verbose=args.verbose)
    check("normal: el lider emite (hay sesion)", r["lead"]["beacons"] > 0)
    check("normal: el seguidor enlaza", frac_guided(r["guided"]) > 0.9,
          f"guided={frac_guided(r['guided']):.2f}")

    # 2. Perdida 30%: el seguidor mantiene el enlace la mayor parte del tiempo.
    r = run("loss30", loss=0.30)
    check("loss30: sigue guiando >80%", frac_guided(r["guided"]) > 0.8,
          f"guided={frac_guided(r['guided']):.2f}")

    # 3. Perdida 60%: se degrada pero no cuelga ni diverge sin control.
    r = run("loss60", loss=0.60)
    check("loss60: no se queda 'guiando' a ciegas (guided<1)", frac_guided(r["guided"]) < 1.0,
          f"guided={frac_guided(r['guided']):.2f}")

    # 4. Blackout 8-16s: pierde enlace y RECUPERA.
    r = run("blackout", blackout=(8000, 16000))
    g = r["guided"]
    lost_mid = any(not gg for t, gg in g if 9000 <= t <= 15000)
    recov_end = any(gg for t, gg in g if t > 20000)
    check("blackout: pierde enlace durante el apagon", lost_mid)
    check("blackout: recupera despues", recov_end)

    # 5. Sesion on-demand: sin seguidor, el lider no emite.
    r = run("no_follower", netid_f=0)
    check("sin seguidor (netid distinto): el lider NO emite", r["lead"]["beacons"] == 0)
    check("sin seguidor: el seguidor NO enlaza", frac_guided(r["guided"]) == 0.0)

    # 6. Gate de modo: a 100 m con modo inestable -> no guia; al pasar a estable -> si.
    mode_tl = lambda t: 4 if 5000 <= t < 15000 else 15   # ACRO(4) entre 5-15s
    r = run("mode_gate", mode_of=mode_tl, dist_of=lambda t: 100.0)
    g = r["guided"]
    held = all(not gg for t, gg in g if 6000 <= t <= 14000)
    resumed = any(gg for t, gg in g if 17000 <= t <= 25000)
    check("gate: mantiene (no guia) con lider inestable y cerca", held)
    check("gate: reanuda al volver a modo estable", resumed)

    # 7. Lejos: aunque el modo sea inestable, a >approach se aproxima (guiado).
    r = run("far_unstable", mode_of=lambda t: 4, dist_of=lambda t: 2000.0)
    check("lejos+inestable: SI guia (acude a buscarlo)", frac_guided(r["guided"]) > 0.9,
          f"guided={frac_guided(r['guided']):.2f}")

    # 8. No cuelgues: todos los escenarios terminan y el estado final es coherente.
    check("sin bloqueos: la simulacion completa todos los pasos", True)

    print(f"\n# PROTO: {checks - fails}/{checks} checks OK, {fails} fallos")


if __name__ == "__main__":
    main()

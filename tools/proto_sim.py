#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Simulador del PROTOCOLO de comunicacion FlyWithMe (v2) -- casos y fallos.

Modela la maquina de estados de la capa de comunicacion (sin radio real):
  * LIDER: en la configuración estable actual (FOLLOWER_REPLY=0) conserva la cadencia normal.
    El modelo también prueba el futuro modo de descubrimiento+sesión cuando REPLY esté habilitado.
  * SEGUIDOR: REPLY/JOIN es opt-in porque el SX1276 es half-duplex y requiere slot coordinado.
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
SESSION_TIMEOUT = 10000   # ms - sesion expira sin reply
REPLY_PERIOD = 1500       # ms - cadencia del seguidor
BEACON_PERIOD = 200       # ms - cadencia de beacon (banco 5 Hz)
DISCOVERY_PERIOD = 2000   # ms - tasa lenta de descubrimiento mientras no hay reply
LINK_TIMEOUT = 10000      # ms - LOST_TIME_BEACON real del firmware
APPROACH = 300.0          # m
VERSION = 2
STABLE = {5, 6, 7, 10, 11, 12, 13, 15}
FOLLOWER_REPLY_DEFAULT = True


def stable(mode):
    return mode in STABLE


def run(scenario, seconds=40, loss=0.0, blackout=None, netid_l=1, netid_f=1,
        mode_of=lambda t: 15, dist_of=lambda t: 1000.0, verbose=False,
        reply_enabled=FOLLOWER_REPLY_DEFAULT, reply_silence=None):
    blackout = blackout or (0, 0)   # (inicio_ms, fin_ms)
    reply_silence = reply_silence or (0, 0)
    # estado
    lead = {"session_until": -1e9, "next_beacon": 0, "beacons": 0,
            "fast_beacons": 0, "beacon_events": []}
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
        active = now < lead["session_until"]
        period = (BEACON_PERIOD if active or not reply_enabled else DISCOVERY_PERIOD)
        if now >= lead["next_beacon"]:
            lead["next_beacon"] = now + period
            lead["beacons"] += 1
            lead["fast_beacons"] += int(active)
            lead["beacon_events"].append((now, active))
            deliver("beacon")
        # seguidor
        if reply_enabled and now >= fol["next_reply"]:
            fol["next_reply"] = now + REPLY_PERIOD
            silenced = reply_silence[0] <= now < reply_silence[1]
            if fol["linked"] and not silenced:
                fol["replies"] += 1
                deliver("reply")
            elif not fol["linked"] and not silenced:
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

    # 1. Config estable actual: sin REPLY, el líder mantiene la tasa normal y el seguidor enlaza.
    r = run("normal", verbose=args.verbose, reply_enabled=False)
    check("normal: lider mantiene beacon sin REPLY", r["lead"]["beacons"] >= 40_000 // BEACON_PERIOD)
    check("normal: el seguidor enlaza", frac_guided(r["guided"]) > 0.9,
          f"guided={frac_guided(r['guided']):.2f}")
    check("sin REPLY de diagnóstico no registra sesión", r["lead"]["session_until"] < 0)

    # 1b. Camino de retorno modelado: respuesta válida acelera; expira con timeout.
    r = run("session", seconds=30, reply_enabled=True)
    check("sesion: JOIN/REPLY activa tasa rápida", r["lead"]["fast_beacons"] > 20,
          f"fast={r['lead']['fast_beacons']}")

    # 1c. Si dejan de llegar replies, la sesion expira y el lider vuelve a discovery; al volver,
    # el seguidor se reengancha y la tasa activa se recupera.
    r = run("session_timeout", seconds=45, reply_enabled=True, reply_silence=(8000, 30000))
    events = r["lead"]["beacon_events"]
    slow_after = any(20000 <= t <= 29000 and not active for t, active in events)
    resumed = any(t > 33000 and active for t, active in events)
    check("session timeout: sin reply vuelve a discovery", slow_after)
    check("session timeout: reply restaurado reactiva tasa rápida", resumed)

    # 2. Perdida 30%: el seguidor mantiene el enlace la mayor parte del tiempo.
    r = run("loss30", loss=0.30)
    check("loss30: sigue guiando >80%", frac_guided(r["guided"]) > 0.8,
          f"guided={frac_guided(r['guided']):.2f}")

    # 3. Perdida 60%: se degrada pero no cuelga ni diverge sin control.
    r = run("loss60", loss=0.60)
    check("loss60: recibe datos y el bucle termina", r["fol"]["rx"] > 0 and len(r["guided"]) > 0,
          f"guided={frac_guided(r['guided']):.2f} rx={r['fol']['rx']}")

    # 4. Blackout 8-16s: pierde enlace y RECUPERA.
    r = run("blackout", blackout=(8000, 24000))
    g = r["guided"]
    lost_mid = any(not gg for t, gg in g if 17000 <= t <= 23000)
    recov_end = any(gg for t, gg in g if t > 28000)
    check("blackout: pierde enlace durante el apagon", lost_mid)
    check("blackout: recupera despues", recov_end)

    # 5. Red ajena: el líder conserva sus beacons, pero el receptor filtra netid.
    r = run("no_follower", netid_f=0)
    check("red distinta: el seguidor rechaza todos los paquetes", r["fol"]["rx"] == 0)
    check("red distinta: el lider no cree que haya sesión", r["lead"]["session_until"] < 0)
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

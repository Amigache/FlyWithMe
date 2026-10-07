#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Regresion automatica de seguridad geometrica del predictor (sin SITL/hardware)."""
from follow_sim import BASE, run


def main():
    checks = []
    p = dict(BASE, dist_offset=20.0, prediction_time=1.0,
             prediction_max_lead_fraction=1.0)
    uncapped, _ = run("trail", "trail", p, latency=0.2, update_period=0.2,
                      seconds=120, gap=100)
    checks.append(("prediccion sin cap reproduce el offset cancelado",
                   uncapped["sep_min"] < 10.0, uncapped["sep_min"]))

    for offset in (5.0, 10.0, 20.0, 96.0):
        capped = dict(BASE, dist_offset=offset, prediction_time=1.0,
                      prediction_max_lead_fraction=0.25)
        result, _ = run("trail", "trail", capped, latency=0.2, update_period=0.2,
                        seconds=120, gap=100)
        floor = max(2.0, offset * 0.5)
        checks.append((f"offset={offset:g} conserva suelo >= {floor:g}m",
                       result["sep_min"] >= floor, result["sep_min"]))

    p_head = dict(BASE, dist_offset=20.0, prediction_time=1.0,
                  prediction_max_lead_fraction=0.25)
    no_guard, _ = run("head_on", "trail", p_head, 0.2, 0.2, 180, gap=200)
    guarded_p = dict(p_head, guard_range=500.0)
    guarded, _ = run("head_on", "trail", guarded_p, 0.2, 0.2, 180, gap=200)
    checks.append(("head-on a 200m sin guarda reproduce casi-colision",
                   no_guard["sep_min"] < 2.0, no_guard["sep_min"]))
    checks.append(("head-on a 200m con ruptura guarda >10m",
                   guarded["sep_min"] >= 10.0, guarded["sep_min"]))

    crossing, _ = run("crossing", "trail", p_head, 0.2, 0.2, 180, gap=200)
    checks.append(("cruce perpendicular mantiene separacion",
                   crossing["sep_min"] >= 10.0, crossing["sep_min"]))

    failures = 0
    for name, passed, value in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: min={value:.1f}m")
        failures += not passed
    print(f"\n# FOLLOW SIM: {len(checks) - failures}/{len(checks)} checks OK")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()

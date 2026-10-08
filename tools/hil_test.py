#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HIL test: valida el enlace lider -> seguidor sin autopiloto (FC_EMULATION=1).

Lee simultaneamente el serial de ambas placas y comprueba, en estado estacionario:
  - El lider transmite paquetes LoRa ("Sent compressed packet").
  - El seguidor engancha el beacon y calcula la formacion ("Formation position").
  - No entra en modo EMERGENCY.

Requisitos:
  - Ambas placas con el mismo perfil `ttgo-lora32-v1` (FC_EMULATION=1).
  - Roles provisionados: leader en una placa y follower en la otra.
  - pyserial instalado en el Python que lo ejecute. Con PlatformIO:
      .platformio\\penv\\Scripts\\python.exe tools\\hil_test.py --master COMx --slave COMx

Uso:
  python tools/hil_test.py --master COMx --slave COMx [--seconds 35] [--baud 57600]
"""
import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("ERROR: falta pyserial. Instala con 'pip install pyserial' o usa el python de PlatformIO.")
    sys.exit(2)


def read_port(results, name, port, baud, seconds):
    try:
        s = serial.Serial(port, baud, timeout=0.3)
        s.dtr = False
        s.rts = False
        s.reset_input_buffer()
        buf = b""
        t0 = time.time()
        while time.time() - t0 < seconds:
            data = s.read(4096)
            if data:
                buf += data
        s.close()
        results[name] = buf.decode("utf-8", "replace")
    except Exception as e:  # noqa: BLE001
        results[name] = ""
        results[name + "_error"] = str(e)


def main():
    ap = argparse.ArgumentParser(description="HIL test FlyWithMe (lider/seguidor por LoRa)")
    ap.add_argument("--master", default="COMx", help="puerto serie del lider")
    ap.add_argument("--slave", default="COMx", help="puerto serie del seguidor")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--seconds", type=int, default=35)
    args = ap.parse_args()

    print(f"Leyendo {args.master} (lider) y {args.slave} (seguidor) durante {args.seconds}s ...")
    results = {}
    threads = [
        threading.Thread(target=read_port, args=(results, "master", args.master, args.baud, args.seconds)),
        threading.Thread(target=read_port, args=(results, "slave", args.slave, args.baud, args.seconds)),
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    master = results.get("master", "")
    slave = results.get("slave", "")
    if "master_error" in results:
        print(f"ERROR abriendo {args.master}: {results['master_error']}")
    if "slave_error" in results:
        print(f"ERROR abriendo {args.slave}: {results['slave_error']}")

    checks = [
        ("lider transmite (Send Packet)", "Send Packet:" in master),
        ("lider sin emergencia", "EMERGENCY MODE" not in master),
        ("seguidor arranca (FWM Ready)", "FWM Ready" in slave),
        ("seguidor engancha beacon (BEACON LOCK)", "BEACON LOCK" in slave),
        ("seguidor en FOLLOWING", "-> FOLLOWING" in slave or "Following leader" in slave),
        ("seguidor calcula formacion (Formation position)", "Formation position" in slave),
        ("seguidor sin emergencia", "EMERGENCY MODE" not in slave),
    ]

    ok = True
    print("\nResultado:")
    for name, passed in checks:
        print(f"  [{'OK  ' if passed else 'FAIL'}] {name}")
        ok = ok and passed

    if not ok:
        print("\n--- master (ultimas lineas) ---")
        print("\n".join(master.splitlines()[-15:]))
        print("\n--- slave (ultimas lineas) ---")
        print("\n".join(slave.splitlines()[-25:]))

    print("\nRESULTADO:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

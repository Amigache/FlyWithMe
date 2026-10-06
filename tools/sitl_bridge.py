#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Puente SITL <-> placa de pruebas.

Conecta un vehiculo SITL (ArduPilot, MAVLink por TCP) con una placa FlyWithMe cuyo firmware
tiene FC_LINK_USB=1 (MAVLink por el USB/UART0). Es un pipe de bytes en ambos sentidos: no
requiere pymavlink.

SITL (Mission Planner, Documents\\Mission Planner\\sitl):
  ArduCopter.exe --instance 0 --serial0 tcp:5760 -M+ -s1 --home -35.363261,149.165230,584,353
  ArduPlane.exe  --instance 0 --serial0 tcp:5760 -M+ -s1 --home -35.363261,149.165230,584,353

Uso (pyserial necesario; el python de PlatformIO lo tiene):
  python tools/sitl_bridge.py --tcp 127.0.0.1:5760 --port COMx --baud 38400
  .\\.platformio\\penv\\Scripts\\python.exe tools/sitl_bridge.py --tcp 127.0.0.1:5760 --port COMx

Multi-vehiculo: instancia 0 -> TCP 5760, instancia 1 -> TCP 5770, etc.
"""
import argparse
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("ERROR: falta pyserial. Usa el python de PlatformIO o 'pip install pyserial'.")
    sys.exit(2)


def main():
    ap = argparse.ArgumentParser(description="Puente SITL (TCP) <-> placa (serie USB)")
    ap.add_argument("--tcp", default="127.0.0.1:5760", help="host:puerto del SITL")
    ap.add_argument("--port", required=True, help="puerto serie de la placa (p. ej. COMx)")
    ap.add_argument("--baud", type=int, default=38400, help="baud del USB de la placa (38400)")
    ap.add_argument("--print-serial", action="store_true", help="mostrar los datos de la placa (logs)")
    args = ap.parse_args()

    host, port = args.tcp.split(":")
    port = int(port)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        ser.dtr = False
        ser.rts = False  # evitar resetear la placa al abrir
    except Exception as e:  # noqa: BLE001
        print(f"ERROR abriendo {args.port}: {e}")
        sys.exit(2)

    try:
        sock = socket.create_connection((host, port), 5)
        sock.settimeout(0.1)
    except Exception as e:  # noqa: BLE001
        print(f"ERROR conectando a SITL {args.tcp}: {e}")
        ser.close()
        sys.exit(2)

    print(f"Puente activo: SITL {args.tcp} <-> {args.port}@{args.baud}. Ctrl+C para salir.")
    stop = threading.Event()

    def tcp_to_ser():
        while not stop.is_set():
            try:
                data = sock.recv(1024)
                if not data:
                    break
                ser.write(data)
            except socket.timeout:
                continue
            except Exception as e:  # noqa: BLE001
                print("tcp->ser:", e)
                break
        stop.set()

    def ser_to_tcp():
        while not stop.is_set():
            try:
                data = ser.read(1024)
                if data:
                    sock.sendall(data)
                    if args.print_serial:
                        txt = "".join(chr(b) if (32 <= b < 127 or b in (10, 13)) else "" for b in data)
                        if txt.strip():
                            sys.stdout.write(txt)
                            sys.stdout.flush()
            except Exception as e:  # noqa: BLE001
                print("ser->tcp:", e)
                break
        stop.set()

    threading.Thread(target=tcp_to_ser, daemon=True).start()
    threading.Thread(target=ser_to_tcp, daemon=True).start()

    try:
        while not stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()
        try:
            sock.close()
        except Exception:  # noqa: BLE001
            pass
        print("Puente cerrado.")


if __name__ == "__main__":
    main()

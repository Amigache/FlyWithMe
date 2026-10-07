#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Puente SITL <-> placa de pruebas (y "tap" MAVLink opcional a la placa).

Conecta un vehiculo SITL (ArduPilot, MAVLink por TCP) con una placa FlyWithMe cuyo firmware
tiene FC_LINK_USB=1 (MAVLink por el USB/UART0). Es un pipe de bytes en ambos sentidos: no
requiere pymavlink.

Ademas, con --tap-port abre un servidor TCP que da un enlace MAVLink DIRECTO a la placa
(sin pasar por el SITL ni por WiFi): lo que el cliente escribe va a la placa, y lo que la
placa emite se reenvia al cliente. Sirve para configurar/leer el periferico FWM desde el PC.

Uso:
  python tools/sitl_bridge.py --tcp 127.0.0.1:5760 --port COMx --baud 57600 [--tap-port 5790]
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

try:
    sys.stdout.reconfigure(line_buffering=True)  # ver logs aunque la salida se redirija a fichero
except Exception:  # noqa: BLE001
    pass


def main():
    ap = argparse.ArgumentParser(description="Puente SITL (TCP) <-> placa (serie USB) + tap MAVLink")
    ap.add_argument("--tcp", default="127.0.0.1:5760", help="host:puerto del SITL")
    ap.add_argument("--port", required=True, help="puerto serie de la placa (p. ej. COMx)")
    ap.add_argument("--baud", type=int, default=57600, help="baud del USB de la placa (57600)")
    ap.add_argument("--print-serial", action="store_true", help="mostrar los datos de la placa (logs)")
    ap.add_argument("--tap-port", type=int, default=0,
                    help="puerto TCP del tap MAVLink directo a la placa (0 = sin tap)")
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
    taps = []
    taps_lock = threading.Lock()
    ser_lock = threading.Lock()

    def write_ser(data):
        """Escribe en el serie de forma atomica (lo comparten tcp_to_ser y los taps)."""
        with ser_lock:
            ser.write(data)

    def tcp_to_ser():
        while not stop.is_set():
            try:
                data = sock.recv(1024)
                if not data:
                    break
                write_ser(data)
            except socket.timeout:
                continue
            except Exception as e:  # noqa: BLE001
                print("tcp->ser:", e)
                break
        stop.set()

    def broadcast(data):
        with taps_lock:
            for c in list(taps):
                try:
                    c.sendall(data)
                except Exception:  # noqa: BLE001
                    try:
                        taps.remove(c)
                    except ValueError:
                        pass

    def ser_to_tcp():
        while not stop.is_set():
            try:
                data = ser.read(1024)
                if data:
                    sock.sendall(data)
                    broadcast(data)  # espejo a los clientes del tap (MAVLink de la placa)
                    if args.print_serial:
                        txt = "".join(chr(b) if (32 <= b < 127 or b in (10, 13)) else "" for b in data)
                        if txt.strip():
                            sys.stdout.write(txt)
                            sys.stdout.flush()
            except Exception as e:  # noqa: BLE001
                print("ser->tcp:", e)
                break
        stop.set()

    def tap_client(conn):
        # OJO: conn tiene timeout; socket.timeout NO debe cerrar el tap (solo esperar mas datos).
        try:
            while not stop.is_set():
                try:
                    data = conn.recv(1024)
                except socket.timeout:
                    continue
                if not data:
                    break
                write_ser(data)     # lo que escribe el cliente va a la placa
        except Exception:  # noqa: BLE001
            pass
        finally:
            with taps_lock:
                try:
                    taps.remove(conn)
                except ValueError:
                    pass
            try:
                conn.close()
            except Exception:  # noqa: BLE001
                pass

    def tap_server():
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(("127.0.0.1", args.tap_port))
        except Exception as e:  # noqa: BLE001
            print(f"ERROR: no se pudo abrir el tap en {args.tap_port}: {e}")
            return
        srv.listen(2)
        srv.settimeout(0.5)
        print(f"Tap MAVLink (placa directa) en tcp:127.0.0.1:{args.tap_port}")
        while not stop.is_set():
            try:
                conn, _ = srv.accept()
                conn.settimeout(0.2)
                with taps_lock:
                    taps.append(conn)
                threading.Thread(target=tap_client, args=(conn,), daemon=True).start()
            except socket.timeout:
                continue
            except Exception:  # noqa: BLE001
                break

    threading.Thread(target=tcp_to_ser, daemon=True).start()
    threading.Thread(target=ser_to_tcp, daemon=True).start()
    if args.tap_port:
        threading.Thread(target=tap_server, daemon=True).start()

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

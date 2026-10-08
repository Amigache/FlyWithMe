#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Puente SITL <-> placa de pruebas (y "tap" MAVLink opcional a la placa).

Conecta un vehiculo SITL (ArduPilot, MAVLink por TCP) con una placa FlyWithMe en modo runtime SITL
(FWM SIM ON, MAVLink por USB/UART0). Es un pipe de bytes en ambos sentidos: no
requiere pymavlink.

Robustez: si el socket del SITL se cae (el SITL reinicia/cierra SERIAL0), el puente **reconecta**
solo en vez de morir, de modo que el enlace placa<->SITL se recupera sin reiniciar el banco.

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


class SockLink:
    """Socket TCP al SITL con reconexion automatica."""

    def __init__(self, host, port, stop):
        self.host = host
        self.port = port
        self.stop = stop
        self._s = None
        self._lock = threading.Lock()
        self._connect(5.0)

    def _connect(self, timeout):
        try:
            s = socket.create_connection((self.host, self.port), timeout)
            s.settimeout(0.1)
            with self._lock:
                self._s = s
            return True
        except Exception:  # noqa: BLE001
            return False

    def _drop(self):
        with self._lock:
            s, self._s = self._s, None
        try:
            if s:
                s.close()
        except Exception:  # noqa: BLE001
            pass

    def send(self, data):
        with self._lock:
            s = self._s
        if s is None:
            return
        try:
            s.sendall(data)
        except Exception:  # noqa: BLE001
            self._drop()

    def recv(self, n):
        if self.stop.is_set():
            return None
        with self._lock:
            s = self._s
        if s is None:
            if self._connect(1.0):
                print(f"Puente: reconectado a SITL {self.host}:{self.port}")
            else:
                time.sleep(0.5)
            return None
        try:
            data = s.recv(n)
            if not data:                      # EOF -> el SITL cerro SERIAL0
                print(f"Puente: SITL cerro la conexion; reintentando {self.host}:{self.port}")
                self._drop()
                return None
            return data
        except socket.timeout:
            return None
        except Exception:  # noqa: BLE001
            self._drop()
            return None

    def close(self):
        self._drop()


def open_serial_port(port, baud):
    """Configura DTR/RTS antes de open(): abrir el COM de un CP210x puede resetear el ESP32."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.1
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def main():
    ap = argparse.ArgumentParser(description="Puente SITL (TCP) <-> placa (serie USB) + tap MAVLink")
    ap.add_argument("--tcp", default="127.0.0.1:5760", help="host:puerto del SITL")
    ap.add_argument("--port", required=True, help="puerto serie de la placa (p. ej. COMx)")
    ap.add_argument("--baud", type=int, default=57600, help="baud del USB de la placa (57600)")
    ap.add_argument("--print-serial", action="store_true", help="mostrar los datos de la placa (logs)")
    ap.add_argument("--stats", action="store_true", help="mostrar bytes serie↔SITL y número de taps cada 5s")
    ap.add_argument("--trace-mavlink", action="store_true", help="registrar HEARTBEAT SYSID/COMPID en ambos sentidos")
    ap.add_argument("--tap-port", type=int, default=0,
                    help="puerto TCP del tap MAVLink directo a la placa (0 = sin tap)")
    args = ap.parse_args()

    trace_from_sitl = trace_from_board = None
    if args.trace_mavlink:
        from pymavlink.dialects.v20 import common as mavlink2
        trace_from_sitl = mavlink2.MAVLink(None)
        trace_from_board = mavlink2.MAVLink(None)

    def trace_heartbeats(parser, direction, data):
        if parser is None:
            return
        for value in data:
            message = parser.parse_char(bytes((value,)))
            if message and message.get_type() == "HEARTBEAT":
                print(f"[bridge] {direction} HEARTBEAT sysid={message.get_srcSystem()} "
                      f"compid={message.get_srcComponent()}", flush=True)

    host, port = args.tcp.split(":")
    port = int(port)

    try:
        ser = open_serial_port(args.port, args.baud)
    except Exception as e:  # noqa: BLE001
        print(f"ERROR abriendo {args.port}: {e}")
        sys.exit(2)

    stop = threading.Event()
    link = SockLink(host, port, stop)
    if link._s is None:
        # no abortamos: seguimos reintentando en recv()
        print(f"AVISO: no se pudo conectar a SITL {args.tcp} aun; reintentando...")

    print(f"Puente activo: SITL {args.tcp} <-> {args.port}@{args.baud}. Ctrl+C para salir.")
    taps = []
    taps_lock = threading.Lock()
    ser_lock = threading.Lock()
    stats_lock = threading.Lock()
    stats = {"sitl_to_board": 0, "board_to_sitl": 0}

    def write_ser(data):
        """Escribe en el serie de forma atomica (lo comparten tcp_to_ser y los taps)."""
        with ser_lock:
            try:
                ser.write(data)
                with stats_lock:
                    stats["sitl_to_board"] += len(data)
            except Exception as e:  # noqa: BLE001
                print("ser.write:", e)

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

    def tcp_to_ser():
        while not stop.is_set():
            data = link.recv(1024)
            if data:
                trace_heartbeats(trace_from_sitl, "SITL->placa", data)
                write_ser(data)

    def ser_to_tcp():
        while not stop.is_set():
            try:
                data = ser.read(1024)
            except Exception as e:  # noqa: BLE001
                print("ser.read:", e)
                time.sleep(0.2)
                continue
            if data:
                trace_heartbeats(trace_from_board, "placa->SITL/TAP", data)
                with stats_lock:
                    stats["board_to_sitl"] += len(data)
                link.send(data)
                broadcast(data)  # espejo a los clientes del tap (MAVLink de la placa)
                if args.print_serial:
                    txt = "".join(chr(b) if (32 <= b < 127 or b in (10, 13)) else "" for b in data)
                    if txt.strip():
                        sys.stdout.write(txt)
                        sys.stdout.flush()

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
        last_stats = time.monotonic()
        while not stop.is_set():
            if args.stats and time.monotonic() - last_stats >= 5:
                with stats_lock:
                    sent = stats["sitl_to_board"]
                    received = stats["board_to_sitl"]
                with taps_lock:
                    tap_count = len(taps)
                print(f"[bridge] hacia placa={sent} B, desde placa={received} B, taps={tap_count}", flush=True)
                last_stats = time.monotonic()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.close()
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
        print("Puente cerrado.")


if __name__ == "__main__":
    main()

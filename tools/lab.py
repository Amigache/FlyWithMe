#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""FlyWithMe LAB -- CLI interactiva del banco de pruebas SITL.

Levanta dos instancias de SITL (ArduPlane) y un MAVProxy headless que las reune en un
unico puerto UDP, de modo que Mission Planner (una sola conexion UDP) ve AMBOS aviones.
Opcionalmente conecta las placas reales (puentes serie) para HIL.

Despues ofrece un menu para lanzar acciones/benchmarks y genera un reporte por ejecucion.

Uso:
  python tools/lab.py                 # menu interactivo
  python tools/lab.py --firmware --master-com COMx --slave-com COMx
  python tools/lab.py --mp-udp 14550
"""
import argparse
import json
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TOOLS = ROOT / "tools"
REPORTS = TOOLS / "reports"
PY = sys.executable

# --- Configuracion (ajustable por args) ---
DEFAULTS = {
    "sitl_exe": Path(os.environ.get("LOCALAPPDATA", "")) / "Temp/opencode/sitl-stable/ArduPlane.exe",
    "sitl_base": Path(os.environ.get("LOCALAPPDATA", "")) / "Temp/opencode/sitl",
    "leader_parm": TOOLS / "sitl/leader.parm",
    "follower_parm": TOOLS / "sitl/follower.parm",
    "fallback_parm": TOOLS / "sitl_plane.parm",
    # SITL: instancia N -> SERIAL0 = 5760+10N (puente/HIL), SERIAL1 = +2 (MAVProxy/MP), SERIAL2 = +3 (control)
    "leader_home": "0.000000,0.000000,302,180",
    "follower_home": "0.000000,0.000000,302,180",
    "mp_udp": 14550,        # UDP donde escucha Mission Planner
    "baud": 57600,
    "leader_com": "COMx",
    "slave_com": "COMx",
    "leader_tap": 5790,     # tap MAVLink directo a la placa del lider
    "slave_tap": 5791,      # tap MAVLink directo a la placa del seguidor
}


def run_quiet(cmd):
    return subprocess.run(cmd, capture_output=True, text=True)


def _ps(script):
    return run_quiet(["powershell", "-NoProfile", "-Command", script])


def pids_of(name):
    r = _ps(f"@(Get-Process -Name '{name}' -ErrorAction SilentlyContinue).Id")
    return [int(x) for x in r.stdout.split() if x.isdigit()]


def kill_all(names):
    for n in names:
        _ps(f"Get-Process -Name '{n}' -ErrorAction SilentlyContinue | Stop-Process -Force")


def kill_python_matching(substrs):
    """Mata procesos python.exe cuyo CommandLine contenga alguno de los substrings."""
    pat = "|".join(substrs)
    _ps("Get-CimInstance Win32_Process -Filter \"name='python.exe'\" | "
        f"Where-Object {{ $_.CommandLine -match '{pat}' }} | "
        "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }")


class Lab:
    def __init__(self, cfg):
        self.cfg = cfg
        self.procs = {}          # nombre -> list[Popen]
        self.stamp = None
        self.report_dir = None

    # ---------- bench ----------
    def _reset_boards(self):
        """Reinicia las placas por DTR/RTS (las CP210x/ESP32 a veces se cuelgan entre sesiones)."""
        try:
            import serial
        except ImportError:
            print("[lab] pyserial no disponible; no reseteo las placas")
            return
        for com in (self.cfg["leader_com"], self.cfg["slave_com"]):
            try:
                s = serial.Serial(com, 115200, timeout=1)
                s.setDTR(False); s.setRTS(True); time.sleep(0.15)
                s.setRTS(False); s.setDTR(False)
                s.close()
                print(f"[lab] reset placa {com}")
            except Exception as e:  # noqa: BLE001
                print(f"[lab] no pude resetear {com}: {e}")
        time.sleep(5)   # dar tiempo al arranque del firmware

    def _sitl_args(self, inst, sysid, home, parm):
        return [str(self.cfg["sitl_exe"]),
                "--instance", str(inst),
                "--serial0", f"tcp:{5760 + 10 * inst}",
                "--sysid", str(sysid),
                "--model", "plane",
                "--home", home,
                "--defaults", str(parm)]

    def start_bench(self):
        print("[lab] parando restos previos...")
        kill_all(["ArduPlane"])
        kill_python_matching(["mp_launch", "mavproxy", "sitl_bridge"])
        time.sleep(1.5)
        exe = Path(self.cfg["sitl_exe"])
        if not exe.exists():
            print(f"[lab] ERROR: no existe SITL: {exe}")
            return False
        lparm = Path(self.cfg["leader_parm"])
        fparm = Path(self.cfg["follower_parm"])
        if not lparm.exists():
            lparm = Path(self.cfg["fallback_parm"])
        if not fparm.exists():
            fparm = Path(self.cfg["fallback_parm"])
        base = Path(self.cfg["sitl_base"])
        for d in (base / "l0", base / "l1"):
            d.mkdir(parents=True, exist_ok=True)

        if self.cfg.get("firmware"):
            self._reset_boards()

        print("[lab] lanzando 2 SITL ArduPlane...")
        p0 = subprocess.Popen(self._sitl_args(0, 1, self.cfg["leader_home"], lparm),
                              cwd=str(base / "l0"),
                              stdout=open(base / "l0/out.log", "w"), stderr=subprocess.STDOUT)
        p1 = subprocess.Popen(self._sitl_args(1, 2, self.cfg["follower_home"], fparm),
                              cwd=str(base / "l1"),
                              stdout=open(base / "l1/out.log", "w"), stderr=subprocess.STDOUT)
        self.procs["sitl"] = [p0, p1]
        time.sleep(11)

        # MAVProxy headless: reune SERIAL1 de ambos y lo saca por UDP para Mission Planner
        masters = ["--master=tcp:127.0.0.1:5762", "--master=tcp:127.0.0.1:5772"]
        out = f"--out=udp:127.0.0.1:{self.cfg['mp_udp']}"
        print(f"[lab] MAVProxy headless -> {out} (Mission Planner conecta UDP {self.cfg['mp_udp']})")
        mplog = open(base / "mavproxy.log", "w", encoding="utf-8", errors="ignore")
        mp = subprocess.Popen([PY, str(TOOLS / "mp_launch.py"), *masters, out, "--daemon"],
                              cwd=str(base), stdout=mplog, stderr=subprocess.STDOUT)
        self.procs["mavproxy"] = [mp]

        if self.cfg.get("firmware"):
            print("[lab] puentes serie a las placas (HIL)...")
            b0 = subprocess.Popen([PY, str(TOOLS / "sitl_bridge.py"),
                                   "--tcp", "127.0.0.1:5760", "--port", self.cfg["leader_com"],
                                   "--baud", str(self.cfg["baud"]),
                                   "--tap-port", str(self.cfg["leader_tap"])], cwd=str(ROOT))
            b1 = subprocess.Popen([PY, str(TOOLS / "sitl_bridge.py"),
                                   "--tcp", "127.0.0.1:5770", "--port", self.cfg["slave_com"],
                                   "--baud", str(self.cfg["baud"]),
                                   "--tap-port", str(self.cfg["slave_tap"])], cwd=str(ROOT))
            self.procs["bridges"] = [b0, b1]
            time.sleep(12)

        print("[lab] bench listo.")
        return True

    def stop_bench(self):
        print("[lab] parando bench...")
        kill_all(["ArduPlane"])
        kill_python_matching(["mp_launch", "mavproxy", "sitl_bridge"])
        for name in ("bridges", "mavproxy"):
            for p in self.procs.get(name, []):
                try:
                    p.terminate()
                except Exception:  # noqa: BLE001
                    pass
        self.procs.clear()
        print("[lab] bench parado.")

    # ---------- acciones (delegan en los scripts existentes) ----------
    def _call(self, args, title):
        print(f"[lab] {title}: {' '.join(str(a) for a in args)}")
        r = subprocess.run([PY, *[str(a) for a in args]], cwd=str(ROOT), text=True,
                           capture_output=True)
        sys.stdout.write(r.stdout)
        if r.returncode != 0 and r.stderr:
            sys.stderr.write(r.stderr)
        return r.returncode, r.stdout

    def takeoff(self, who="both"):
        if who in ("both", "leader"):
            self._call([TOOLS / "sitl_takeoff.py", "--conn", "tcp:127.0.0.1:5763", "--mode", "13", "--alt", "80"],
                       "despegue lider")
        if who in ("both", "follower"):
            self._call([TOOLS / "sitl_takeoff.py", "--conn", "tcp:127.0.0.1:5773", "--mode", "13", "--alt", "80"],
                       "despegue seguidor")

    def follower_guided(self):
        self._call([TOOLS / "sitl_guided.py", "--conn", "tcp:127.0.0.1:5773", "--mode", "15"], "seguidor GUIDED")

    def benchmark(self, kind):
        tgt = {"straight": ("0.000000,0.000000,80", None),
               "turn": ("0.000000,0.000000,80", "0.000000,0.000000,80")}[kind]
        args = [TOOLS / "hil_sitl_validate.py", "--target", tgt[0], "--seconds", "130", "--follower-guided"]
        if tgt[1]:
            args += ["--target2", tgt[1]]
        return self._call(args, f"benchmark {kind}")

    # ---------- reporte ----------
    def new_report(self):
        self.stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.report_dir = REPORTS / self.stamp
        self.report_dir.mkdir(parents=True, exist_ok=True)
        return self.report_dir

    def save_report(self, name, rc, out):
        if self.report_dir is None:
            self.new_report()
        (self.report_dir / f"{name}.txt").write_text(out or "", encoding="utf-8")
        meta = {"when": self.stamp, "name": name, "rc": rc}
        (self.report_dir / f"{name}.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(f"[lab] reporte: {self.report_dir / (name + '.txt')}")


MENU = """
================= FlyWithMe LAB =================
 1) Levantar / reiniciar bench (SITL + MAVProxy)
 2) Estado (procesos / puertos)
 3) Despegar ambos
 4) Despegar solo lider
 5) Seguidor -> GUIDED
 6) Benchmark: recta
 7) Benchmark: giro 90
 8) Parar bench
 9) BENCH COMPLETO (todos los tests + reporte)
 0) Salir
=================================================
"""


def status():
    print("ArduPlane:", pids_of("ArduPlane"))
    r = _ps("Get-CimInstance Win32_Process -Filter \"name='python.exe'\" | "
            "Where-Object { $_.CommandLine -match 'mp_launch|mavproxy|sitl_bridge' } | "
            "Select-Object -ExpandProperty ProcessId")
    print("mavproxy/bridges:", [int(x) for x in r.stdout.split() if x.isdigit()])


def main():
    ap = argparse.ArgumentParser(description="FlyWithMe LAB (SITL + MAVProxy + MP por UDP)")
    ap.add_argument("--firmware", action="store_true", help="tambien conecta las placas via puentes serie")
    ap.add_argument("--leader-com", default=DEFAULTS["leader_com"])
    ap.add_argument("--slave-com", default=DEFAULTS["slave_com"])
    ap.add_argument("--mp-udp", type=int, default=DEFAULTS["mp_udp"])
    ap.add_argument("--sitl-exe", default=str(DEFAULTS["sitl_exe"]))
    args = ap.parse_args()

    cfg = dict(DEFAULTS)
    cfg.update(firmware=args.firmware, leader_com=args.leader_com, slave_com=args.slave_com,
               mp_udp=args.mp_udp, sitl_exe=Path(args.sitl_exe))
    lab = Lab(cfg)

    print(f"SITL: {cfg['sitl_exe']}")
    print(f"Mission Planner -> UDP 127.0.0.1:{cfg['mp_udp']}")
    while True:
        print(MENU)
        try:
            c = input("opcion> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if c == "1":
            lab.start_bench()
        elif c == "2":
            status()
        elif c == "3":
            lab.takeoff("both")
        elif c == "4":
            lab.takeoff("leader")
        elif c == "5":
            lab.follower_guided()
        elif c in ("6", "7"):
            kind = "straight" if c == "6" else "turn"
            rc, out = lab.benchmark(kind)
            lab.save_report(f"bench_{kind}", rc, out)
        elif c == "8":
            lab.stop_bench()
        elif c == "9":
            subprocess.run([PY, str(TOOLS / "bench_suite.py")], cwd=str(ROOT))
        elif c == "0":
            break
        else:
            print("opcion no valida")
    lab.stop_bench()
    print("adios.")


if __name__ == "__main__":
    main()

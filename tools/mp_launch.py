#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lanza MAVProxy en modo headless (sin wxPython/GUI).

En Windows MAVProxy fuerza `mp_util.has_wxpython = True` (deteccion rota), lo que hace
que importe wx aunque no haga falta. Aqui lo ponemos a False ANTES de arrancar, de modo
que MAVProxy corre sin GUI ni wxPython (consola de texto / modo daemon).

Uso (igual que mavproxy):
  python tools/mp_launch.py --master=tcp:127.0.0.1:5760 --master=tcp:127.0.0.1:5770 --out=udp:127.0.0.1:14550 --daemon
"""
import os
import runpy
import sys
import inspect
import platform
import textwrap

# Parche en memoria de pymavlink (Windows): las salidas UDP bindean el puerto DESTINO, lo que
# choca con Mission Planner en el mismo PC. No modificar site-packages ni el bundle instalado.
try:
    import pymavlink.mavutil as _mavutil

    if platform.system() == "Windows":
        _method = _mavutil.mavudp.__init__
        _source = inspect.getsource(_method)
        _old = "self.port.bind(('0.0.0.0', int(a[1])))"
        if _old in _source:
            _patched = _source.replace(_old, "self.port.bind(('0.0.0.0', 0))", 1)
            _namespace = {}
            exec(compile(textwrap.dedent(_patched), inspect.getsourcefile(_method) or "<mavudp>", "exec"),
                 _mavutil.__dict__, _namespace)
            _mavutil.mavudp.__init__ = _namespace["__init__"]
except Exception:  # noqa: BLE001
    pass

try:
    from MAVProxy.modules.lib import mp_util

    mp_util.has_wxpython = False
except Exception:  # noqa: BLE001
    pass

# En modo daemon no hay consola (y prompt_toolkit/win32 falla). Sustituimos rline por un dummy.
try:
    from MAVProxy.modules.lib import rline as _rline

    class _DummyRline:  # noqa: D401
        def __init__(self, _prompt, mpstate, *a, **k):
            if not hasattr(mpstate, "completion_functions"):
                mpstate.completion_functions = {}

        def __getattr__(self, _name):
            def _f(*a, **k):
                return ""
            return _f

    _rline.rline = _DummyRline
except Exception:  # noqa: BLE001
    pass

import MAVProxy  # noqa: E402

_script = os.path.join(os.path.dirname(MAVProxy.__file__), "mavproxy.py")
sys.argv = ["mavproxy.py"] + sys.argv[1:]
runpy.run_path(_script, run_name="__main__")

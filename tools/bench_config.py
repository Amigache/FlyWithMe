"""Configuración local del banco de pruebas: coordenadas, puertos COM y rutas de SITL.

Los valores reales viven en tools/bench.local.json, que git ignora. tools/bench.example.json es la
plantilla versionada con valores genéricos. Las claves que falten en el fichero local toman el valor
de la plantilla. Los scripts PowerShell leen los mismos ficheros con tools/bench_config.ps1.
"""
import json
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
EXAMPLE_PATH = TOOLS / "bench.example.json"
LOCAL_PATH = TOOLS / "bench.local.json"


def load():
    # utf-8-sig: PowerShell y Notepad añaden BOM al guardar en Windows.
    cfg = json.loads(EXAMPLE_PATH.read_text(encoding="utf-8-sig"))
    if LOCAL_PATH.is_file():
        cfg.update(json.loads(LOCAL_PATH.read_text(encoding="utf-8-sig")))
    return cfg


def is_configured():
    return LOCAL_PATH.is_file()

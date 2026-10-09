#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Comprueba que las hashes del sitio del flasheador coinciden con las de la release.

El sitio y el zip de la release deben publicar los MISMOS binarios. Las builds de ESP-IDF no
son reproducibles byte a byte, asi que cualquier divergencia significa que uno de los dos
canales se ha generado aparte. En ese caso el usuario que instala desde la web no puede
verificar nada contra el manifiesto de la release, y el despliegue debe abortar.
"""

import argparse
import json
import sys
from pathlib import Path


def read_sha256sums(path: Path) -> dict:
    """Lee un fichero en formato 'sha256  nombre'."""
    sums = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        digest, name = line.split(maxsplit=1)
        sums[name.strip()] = digest
    return sums


def manifest_hashes(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    return {segment["asset"]: segment["sha256"] for segment in data["segments"]}


def main() -> int:
    parser = argparse.ArgumentParser(description="Verifica las hashes del flasheador frente a la release")
    parser.add_argument("--manifest", type=Path, required=True, help="manifest.json de la release")
    parser.add_argument("--sha256sums", type=Path, required=True, help="SHA256SUMS del sitio")
    args = parser.parse_args()

    expected = manifest_hashes(args.manifest)
    actual = read_sha256sums(args.sha256sums)

    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    mismatched = sorted(name for name in set(expected) & set(actual) if expected[name] != actual[name])

    if missing or extra or mismatched:
        if missing:
            print(f"FALLO: ausentes en el sitio: {', '.join(missing)}")
        if extra:
            print(f"FALLO: en el sitio pero no en la release: {', '.join(extra)}")
        for name in mismatched:
            print(f"FALLO: {name}\n  release {expected[name]}\n  sitio    {actual[name]}")
        print("\nEl sitio y la release no publican el mismo firmware.", file=sys.stderr)
        return 1

    print(f"OK: {len(expected)} hashes coinciden entre el sitio y la release")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
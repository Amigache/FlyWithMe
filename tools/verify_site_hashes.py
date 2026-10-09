#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Comprueba que las hashes del sitio del flasheador coinciden con las de la release, por placa.

El sitio y los zips de la release deben publicar los MISMOS binarios. Las builds de ESP-IDF no son
reproducibles byte a byte, asi que cualquier divergencia significa que uno de los dos canales se ha
generado aparte. En ese caso el usuario que instala desde la web no puede verificar nada contra el
manifiesto de la release, y el despliegue debe abortar.
"""

import argparse
import hashlib
import json
import sys
import zipfile
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


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


def segment_hashes_from_bundle(bundle: Path) -> dict:
    """Lee manifest.json dentro del zip de la release y devuelve {asset: sha256}."""
    with zipfile.ZipFile(bundle) as archive:
        data = json.loads(archive.read("manifest.json"))
    return {segment["asset"]: segment["sha256"] for segment in data["segments"]}


def compare(expected: dict, actual: dict, label: str) -> list:
    problems = []
    for name in sorted(set(expected) - set(actual)):
        problems.append(f"{label}: ausente {name}")
    for name in sorted(set(actual) - set(expected)):
        problems.append(f"{label}: sobra {name}")
    for name in sorted(set(expected) & set(actual)):
        if expected[name] != actual[name]:
            problems.append(f"{label}: {name}\n        release {expected[name]}\n        {label} {actual[name]}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description="Verifica las hashes del flasheador frente a la release")
    parser.add_argument("--release-assets", type=Path, required=True,
                        help="directorio con los zips FlyWithMe-<version>-<placa>.zip")
    parser.add_argument("--site", type=Path, required=True, help="directorio del sitio generado")
    args = parser.parse_args()

    bundles = sorted(args.release_assets.glob("FlyWithMe-*.zip"))
    if not bundles:
        print(f"FALLO: no hay zips en {args.release_assets}", file=sys.stderr)
        return 1

    failed = False
    checked = 0
    for bundle in bundles:
        # FlyWithMe-<version>-<board>.zip
        board = bundle.stem.split("-", 2)[2]
        firmware_dir = args.site / "firmware" / board
        if not firmware_dir.is_dir():
            print(f"FALLO [{board}]: el sitio no tiene {firmware_dir}", file=sys.stderr)
            failed = True
            continue

        expected = segment_hashes_from_bundle(bundle)
        problems = []

        # 1) Los binarios REALES del sitio: es lo que descarga el navegador. Hashearlos es lo unico
        #    que detecta un fichero corrupto o truncado; comparar dos listas de hashes no lo haria.
        on_disk = {}
        for name in expected:
            candidate = firmware_dir / name
            if not candidate.is_file():
                problems.append(f"falta el binario {name} en el sitio")
                continue
            on_disk[name] = sha256(candidate)
        problems += compare(expected, on_disk, "sitio")

        # 2) Coherencia del SHA256SUMS publicado, que es lo que un usuario comprueba a mano.
        sums_file = firmware_dir / "SHA256SUMS"
        if not sums_file.is_file():
            problems.append("falta firmware/<placa>/SHA256SUMS")
        else:
            problems += compare(expected, read_sha256sums(sums_file), "SHA256SUMS")

        if problems:
            print(f"FALLO [{board}]:")
            for problem in problems:
                print(f"    {problem}")
            failed = True
        else:
            checked += 1
            print(f"OK [{board}]: hashes coinciden con la release")

    if failed:
        print("\nEl sitio y la release no publican el mismo firmware.", file=sys.stderr)
        return 1
    print(f"OK: {checked} placas verificadas")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
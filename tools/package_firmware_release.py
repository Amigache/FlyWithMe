#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Empaqueta los perfiles de vuelo para releases de producción.

Una entrada por placa. `beta` marca los targets que todavia NO se han validado en hardware:
se publican para que los betatesters puedan probarlos, pero el flasheador los etiqueta como beta
y el README lo advierte. Nunca poner beta=False sin haber verificado la placa de verdad.
"""

import argparse
import hashlib
import json
import os
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Fuente unica de verdad de las placas publicadas. La usan package_firmware_release.py y
# build_web_flasher.py, de modo que el zip de la release y el sitio del flasheador no puedan
# discrepar sobre que placas existen.
BOARDS = {
    "ttgo-lora32-v1": {
        "env": "ttgo-lora32-v1-flight",
        "label": "TTGO LoRa32 V1",
        "beta": False,
    },
    "ttgo-lora32-v21": {
        "env": "ttgo-lora32-v21-flight",
        "label": "TTGO LoRa32 V1.6 / V2.0 / V2.1.6 (T3)",
        "beta": True,
    },
}

SEGMENTS = (
    ("bootloader.bin", "0x1000"),
    ("partitions.bin", "0x8000"),
    ("boot_app0.bin", "0xE000"),
    ("firmware.bin", "0x10000"),
)

PROTOCOL_VERSION = 2


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_boot_app0(override: Path | None) -> Path:
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    candidates = [override] if override else []
    candidates.append(core / "packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin")
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate
    raise FileNotFoundError("No se encontró boot_app0.bin en el paquete Arduino ESP32.")


def resolve_segments(board: str, build_root: Path, boot_app0: Path | None,
                     binaries_dir: Path | None = None) -> list:
    """Devuelve [(nombre, direccion, fichero)] para una placa.

    Con `binaries_dir` se reutilizan binarios ya construidos (el zip de la release); sin el se leen
    del build de PlatformIO. Reutilizarlos es lo que garantiza que el sitio y la release publiquen
    exactamente el mismo firmware: las builds de ESP-IDF no son reproducibles byte a byte.
    """
    env = BOARDS[board]["env"]
    build_dir = build_root / env
    resolved = []
    for filename, address in SEGMENTS:
        if binaries_dir is not None:
            source = binaries_dir / filename
        elif filename == "boot_app0.bin":
            if boot_app0 is None:
                raise ValueError("sin binaries_dir hace falta boot_app0")
            source = boot_app0
        else:
            source = build_dir / filename
        if not source.is_file():
            hint = (f"faltan binarios de {board} en {binaries_dir}" if binaries_dir is not None
                    else f"falta {source}; compila primero -e {env}")
            raise FileNotFoundError(hint)
        resolved.append((filename, address, source))
    return resolved


def build_manifest(board: str, version: str, commit: str, segments: list) -> dict:
    return {
        "schema": 1,
        "version": version,
        "git_commit": commit,
        "board": board,
        "label": BOARDS[board]["label"],
        "beta": BOARDS[board]["beta"],
        "protocol": PROTOCOL_VERSION,
        "erase_nvs": False,
        "flash": {"chip": "esp32", "baud": 115200, "mode": "dio", "frequency": "40m", "size": "4MB"},
        "segments": [
            {"asset": name, "address": address, "sha256": sha256(source)}
            for name, address, source in segments
        ],
    }


def package_board(board: str, version: str, commit: str, output: Path, segments: list) -> dict:
    manifest = build_manifest(board, version, commit, segments)
    bundle_name = f"FlyWithMe-{version}-{board}.zip"
    bundle_path = output / bundle_name
    with zipfile.ZipFile(bundle_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
        bundle.writestr("manifest.json", json.dumps(manifest, indent=2))
        for filename, _address, source in segments:
            bundle.write(source, arcname=filename)
    return {
        "archive": bundle_name,
        "size": bundle_path.stat().st_size,
        "sha256": sha256(bundle_path),
        "beta": BOARDS[board]["beta"],
        "label": BOARDS[board]["label"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Crea bundles de firmware de producción para distribución")
    parser.add_argument("--version", default=os.environ.get("GITHUB_REF_NAME", "dev"))
    parser.add_argument("--commit", default=os.environ.get("GITHUB_SHA", "unknown"))
    parser.add_argument("--build-root", type=Path, default=ROOT / ".pio" / "build")
    parser.add_argument("--boot-app0", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=ROOT / "release-assets")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    boot_app0 = find_boot_app0(args.boot_app0)

    profiles = {}
    for board in BOARDS:
        segments = resolve_segments(board, args.build_root, boot_app0)
        profiles[board] = package_board(board, args.version, args.commit, args.output, segments)

    manifest = {
        "schema": 1,
        "version": args.version,
        "git_commit": args.commit,
        "protocol": PROTOCOL_VERSION,
        "boards": profiles,
    }
    (args.output / "release-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    for board, info in profiles.items():
        tag = " (beta)" if info["beta"] else ""
        print(f"  {board}: {info['archive']} ({info['size']} B){tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
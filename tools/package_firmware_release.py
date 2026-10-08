#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Empaqueta solo el perfil bloqueado de vuelo para releases de producción."""

import argparse
import hashlib
import json
import os
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROFILES = {
    "flight": "ttgo-lora32-v1-flight",
}
SEGMENTS = (
    ("bootloader.bin", "0x1000"),
    ("partitions.bin", "0x8000"),
    ("boot_app0.bin", "0xE000"),
    ("firmware.bin", "0x10000"),
)


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


def package_profile(profile: str, version: str, commit: str, build_root: Path,
                    output: Path, boot_app0: Path) -> dict:
    env = PROFILES[profile]
    build_dir = build_root / env
    segment_paths = []
    for filename, address in SEGMENTS:
        source = boot_app0 if filename == "boot_app0.bin" else build_dir / filename
        if not source.is_file():
            raise FileNotFoundError(f"Falta {source}; primero compila -e {env}.")
        segment_paths.append((filename, address, source))

    profile_manifest = {
        "schema": 1,
        "version": version,
        "git_commit": commit,
        "board": "ttgo-lora32-v1",
        "profile": profile,
        "protocol": 2,
        "erase_nvs": False,
        "flash": {"chip": "esp32", "baud": 115200, "mode": "dio", "frequency": "40m", "size": "4MB"},
        "segments": [
            {"asset": name, "address": address, "sha256": sha256(source)}
            for name, address, source in segment_paths
        ],
    }
    bundle_name = f"FlyWithMe-{version}-ttgo-lora32-v1-{profile}.zip"
    bundle_path = output / bundle_name
    with zipfile.ZipFile(bundle_path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as bundle:
        bundle.writestr("manifest.json", json.dumps(profile_manifest, indent=2))
        for filename, _address, source in segment_paths:
            bundle.write(source, arcname=filename)
    return {"archive": bundle_name, "size": bundle_path.stat().st_size,
            "sha256": sha256(bundle_path)}


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
    profiles = {
        name: package_profile(name, args.version, args.commit, args.build_root, args.output, boot_app0)
        for name in PROFILES
    }
    manifest = {
        "schema": 1,
        "version": args.version,
        "git_commit": args.commit,
        "board": "ttgo-lora32-v1",
        "protocol": 2,
        "profiles": profiles,
    }
    (args.output / "release-manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"Bundles y manifest listos en {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

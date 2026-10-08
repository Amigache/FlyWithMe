#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Genera el sitio estático del flasheador web (GitHub Pages) para el perfil de vuelo.

El manifiesto de ESP Web Tools publica las cuatro particiones por separado (bootloader, tabla,
boot_app0 y aplicación) en sus offsets reales. No se publica una imagen "merged": esa imagen
rellenaría con 0xFF la NVS (0x9000) y borraría el rol, los parámetros y el netid de cada placa.
"""

import argparse
import json
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from package_firmware_release import PROFILES, SEGMENTS, find_boot_app0, sha256  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
WEB_SRC = ROOT / "web"
PROFILE = "flight"
ESP_WEB_TOOLS_VERSION = "10.4.0"


def fetch_esp_web_tools(destination: Path) -> None:
    """Descarga esp-web-tools (versión fijada) desde npm y copia dist/web para servirlo localmente."""
    npm = shutil.which("npm")
    if npm is None:
        raise FileNotFoundError("Se necesita npm para descargar esp-web-tools.")
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [npm, "pack", f"esp-web-tools@{ESP_WEB_TOOLS_VERSION}", "--pack-destination", tmp],
            check=True, capture_output=True, text=True,
        )
        tarball = Path(tmp) / f"esp-web-tools-{ESP_WEB_TOOLS_VERSION}.tgz"
        if not tarball.is_file():
            raise FileNotFoundError(f"npm no generó {tarball.name}")
        destination.mkdir(parents=True, exist_ok=True)
        with tarfile.open(tarball, "r:gz") as archive:
            members = [m for m in archive.getmembers() if m.name.startswith("package/dist/web/") and m.isfile()]
            if not members:
                raise FileNotFoundError("El paquete esp-web-tools no contiene dist/web/")
            for member in members:
                target = destination / Path(member.name).relative_to("package/dist/web")
                target.parent.mkdir(parents=True, exist_ok=True)
                source = archive.extractfile(member)
                if source is None:
                    continue
                target.write_bytes(source.read())


def build_site(version: str, commit: str, build_root: Path, boot_app0: Path, output: Path) -> dict:
    env = PROFILES[PROFILE]
    build_dir = build_root / env
    if output.exists():
        shutil.rmtree(output)
    firmware_dir = output / "firmware"
    firmware_dir.mkdir(parents=True)

    parts = []
    checksums = []
    for filename, address in SEGMENTS:
        source = boot_app0 if filename == "boot_app0.bin" else build_dir / filename
        if not source.is_file():
            raise FileNotFoundError(f"Falta {source}; primero compila -e {env}.")
        shutil.copyfile(source, firmware_dir / filename)
        digest = sha256(firmware_dir / filename)
        checksums.append(f"{digest}  {filename}")
        parts.append({"path": f"firmware/{filename}", "offset": int(address, 16)})

    (firmware_dir / "SHA256SUMS").write_text("\n".join(checksums) + "\n", encoding="utf-8")

    manifest = {
        "name": "FlyWithMe",
        "version": version,
        "new_install_prompt_erase": False,
        "new_install_improv_wait_time": 0,
        "builds": [{"chipFamily": "ESP32", "parts": parts}],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    build_info = {
        "version": version,
        "git_commit": commit,
        "board": "ttgo-lora32-v1",
        "profile": PROFILE,
        "environment": env,
        "esp_web_tools": ESP_WEB_TOOLS_VERSION,
    }
    (output / "build-info.json").write_text(json.dumps(build_info, indent=2) + "\n", encoding="utf-8")

    for source in WEB_SRC.iterdir():
        if source.is_file():
            shutil.copyfile(source, output / source.name)

    fetch_esp_web_tools(output / "vendor" / "esp-web-tools")
    (output / ".nojekyll").write_text("", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Genera el sitio del flasheador web de FlyWithMe")
    parser.add_argument("--version", required=True, help="versión publicada, p. ej. v1.0.0")
    parser.add_argument("--commit", default="unknown", help="commit de la build")
    parser.add_argument("--build-root", type=Path, default=ROOT / ".pio" / "build")
    parser.add_argument("--boot-app0", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=ROOT / "site")
    args = parser.parse_args()

    boot_app0 = find_boot_app0(args.boot_app0)
    manifest = build_site(args.version, args.commit, args.build_root, boot_app0, args.output)
    print(f"Flasheador web listo en {args.output} ({len(manifest['builds'][0]['parts'])} particiones)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

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


def build_site(version: str, commit: str, build_root: Path, boot_app0: Path, output: Path,
               binaries_dir: Path | None = None) -> dict:
    env = PROFILES[PROFILE]
    build_dir = build_root / env
    if output.exists():
        shutil.rmtree(output)
    firmware_dir = output / "firmware"
    firmware_dir.mkdir(parents=True)

    parts = []
    checksums = []
    for filename, address in SEGMENTS:
        # binaries_dir = binarios ya publicados por la release. Es el camino correcto en CI:
        # reconstruir aqui produciria un binario distinto (las builds de ESP-IDF no son
        # reproducibles byte a byte) y las hashes del sitio no coincidirian con las de la release.
        if binaries_dir is not None:
            source = binaries_dir / filename
        elif filename == "boot_app0.bin":
            source = boot_app0
        else:
            source = build_dir / filename
        if not source.is_file():
            hint = (f"faltan binarios en {binaries_dir}" if binaries_dir is not None
                    else f"falta {source}; compila primero -e {env}")
            raise FileNotFoundError(hint)
        shutil.copyfile(source, firmware_dir / filename)
        digest = sha256(firmware_dir / filename)
        checksums.append(f"{digest}  {filename}")
        parts.append({"path": f"firmware/{filename}", "offset": int(address, 16)})

    (firmware_dir / "SHA256SUMS").write_text("\n".join(checksums) + "\n", encoding="utf-8")

    # buildId es lo que ESP Web Tools usa para detectar que hay version nueva. Sin el, un
    # usuario con una version anterior instalada no recibe aviso de que puede actualizar.
    # IMPORTANTE: va DENTRO de cada build, no en la raiz. ESP Web Tools lee builds[].buildId;
    # un buildId en la raiz se ignora y la deteccion de actualizaciones sigue sin funcionar.
    build_id = f"{version}-{commit[:12]}" if commit and commit != "unknown" else version
    manifest = {
        "name": "FlyWithMe",
        "version": version,
        "new_install_prompt_erase": False,
        "new_install_improv_wait_time": 0,
        "builds": [{"chipFamily": "ESP32", "buildId": build_id, "parts": parts}],
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
    parser.add_argument("--binaries-dir", type=Path, default=None,
                        help="directorio con los cuatro binarios ya publicados por la release "
                             "(reutilizarlos garantiza que las hashes del sitio y de la release "
                             "coincidan)")
    parser.add_argument("--output", type=Path, default=ROOT / "site")
    args = parser.parse_args()

    # Con --binaries-dir los cuatro binarios vienen de ahi, incluido boot_app0.bin, asi que no
    # hace falta localizarlo en el paquete de Arduino.
    boot_app0 = None if args.binaries_dir is not None else find_boot_app0(args.boot_app0)
    manifest = build_site(args.version, args.commit, args.build_root, boot_app0, args.output,
                          args.binaries_dir)
    print(f"Flasheador web listo en {args.output} ({len(manifest['builds'][0]['parts'])} particiones, "
          f"buildId={manifest['builds'][0]['buildId']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

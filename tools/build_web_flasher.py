#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Genera el sitio estático del flasheador web (GitHub Pages).

Una entrada por placa. El manifiesto de cada una publica las cuatro particiones por separado
(bootloader, tabla, boot_app0 y aplicación) en sus offsets reales. No se publica una imagen
"merged": esa imagen rellenaría con 0xFF la NVS (0x9000) y borraría el rol, los parámetros y el
netid de cada placa.

En CI se pasa --binaries-root con los binarios YA publicados por la release, en vez de recompilar:
las builds de ESP-IDF no son reproducibles byte a byte, así que reconstruir aquí haría que las
hashes del sitio no coincidieran con las de la release.
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
from package_firmware_release import BOARDS, find_boot_app0, resolve_segments  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
WEB_SRC = ROOT / "web"
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
               binaries_root: Path | None = None, boards=None) -> dict:
    """Genera el sitio. Devuelve {'boards': [...], 'manifests': {board: dict}}.

    `binaries_root/<board>/` debe contener los cuatro binarios ya publicados por la release. Si es
    None se leen del build de PlatformIO en `build_root`.
    """
    selected = list(boards) if boards else list(BOARDS)
    unknown = [board for board in selected if board not in BOARDS]
    if unknown:
        raise ValueError(f"placa desconocida: {', '.join(unknown)}")

    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    boards_info = []
    manifests = {}
    for board in selected:
        binaries_dir = (binaries_root / board) if binaries_root is not None else None
        segments = resolve_segments(board, build_root, boot_app0, binaries_dir)

        firmware_dir = output / "firmware" / board
        firmware_dir.mkdir(parents=True, exist_ok=True)

        parts = []
        checksums = []
        for filename, address, source in segments:
            shutil.copyfile(source, firmware_dir / filename)
            digest = _sha256(firmware_dir / filename)
            checksums.append(f"{digest}  {filename}")
            parts.append({"path": f"firmware/{board}/{filename}", "offset": int(address, 16)})
        (firmware_dir / "SHA256SUMS").write_text("\n".join(checksums) + "\n", encoding="utf-8")

        # buildId va DENTRO de cada build: ESP Web Tools lee builds[].buildId y descarta uno
        # situado en la raiz, con lo que la deteccion de actualizaciones no funcionaria.
        build_id = f"{version}-{commit[:12]}-{board}" if commit and commit != "unknown" else f"{version}-{board}"
        manifest = {
            "name": f"FlyWithMe ({BOARDS[board]['label']})",
            "version": version,
            "new_install_prompt_erase": False,
            "new_install_improv_wait_time": 0,
            "builds": [{"chipFamily": "ESP32", "buildId": build_id, "parts": parts}],
        }
        manifest_name = f"manifest-{board}.json"
        (output / manifest_name).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

        boards_info.append({
            "board": board,
            "label": BOARDS[board]["label"],
            "beta": BOARDS[board]["beta"],
            "manifest": manifest_name,
            "build_id": build_id,
        })
        manifests[board] = manifest

    build_info = {
        "version": version,
        "git_commit": commit,
        "esp_web_tools": ESP_WEB_TOOLS_VERSION,
        "boards": boards_info,
    }
    (output / "build-info.json").write_text(json.dumps(build_info, indent=2) + "\n", encoding="utf-8")

    for source in WEB_SRC.iterdir():
        if source.is_file():
            shutil.copyfile(source, output / source.name)

    fetch_esp_web_tools(output / "vendor" / "esp-web-tools")
    (output / ".nojekyll").write_text("", encoding="utf-8")
    return {"boards": boards_info, "manifests": manifests}


def _sha256(path: Path) -> str:
    import hashlib
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description="Genera el sitio del flasheador web de FlyWithMe")
    parser.add_argument("--version", required=True, help="versión publicada, p. ej. v1.0.0")
    parser.add_argument("--commit", default="unknown", help="commit de la build")
    parser.add_argument("--build-root", type=Path, default=ROOT / ".pio" / "build")
    parser.add_argument("--boot-app0", type=Path, default=None)
    parser.add_argument("--binaries-root", type=Path, default=None,
                        help="raiz con <placa>/ y los cuatro binarios ya publicados por la release")
    parser.add_argument("--boards", default=None,
                        help="lista separada por comas; por defecto todas las de BOARDS")
    parser.add_argument("--output", type=Path, default=ROOT / "site")
    args = parser.parse_args()

    # Con --binaries-root los cuatro binarios vienen de ahi, incluido boot_app0.bin.
    boot_app0 = None if args.binaries_root is not None else find_boot_app0(args.boot_app0)
    boards = [b for b in args.boards.split(",") if b] if args.boards else None
    result = build_site(args.version, args.commit, args.build_root, boot_app0, args.output,
                        args.binaries_root, boards)
    print(f"Flasheador web listo en {args.output}")
    for info in result["boards"]:
        tag = " (beta)" if info["beta"] else ""
        print(f"  {info['board']}: {info['manifest']} buildId={info['build_id']}{tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
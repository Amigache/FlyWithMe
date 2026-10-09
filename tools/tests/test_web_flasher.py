import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

from tools.build_web_flasher import build_site
from tools.package_firmware_release import BOARDS, SEGMENTS

ROOT = Path(__file__).resolve().parents[2]

V1 = "ttgo-lora32-v1"
V21 = "ttgo-lora32-v21"


def write_fake_binaries(directory: Path, board: str) -> dict:
    """Crea los cuatro binarios de una placa y devuelve su sha256."""
    directory.mkdir(parents=True, exist_ok=True)
    digests = {}
    for index, (filename, _address) in enumerate(SEGMENTS):
        payload = f"contenido-{board}-{filename}-{index}".encode()
        (directory / filename).write_bytes(payload)
        digests[filename] = hashlib.sha256(payload).hexdigest()
    return digests


def write_fake_bundles(release_assets: Path, version: str, boards) -> dict:
    """Crea zips con la misma forma que produce package_firmware_release.

    Deja tambien los binarios "desempaquetados" en <tmp>/published/<placa>, que es la estructura que
    recibe build_web_flasher con --binaries-root.
    """
    release_assets.mkdir(parents=True, exist_ok=True)
    published_root = release_assets.parent / "published"
    expected = {}
    for board in boards:
        source = published_root / board
        digests = write_fake_binaries(source, board)
        bundle = release_assets / f"FlyWithMe-{version}-{board}.zip"
        with zipfile.ZipFile(bundle, "w") as archive:
            for filename, _address in SEGMENTS:
                archive.write(source / filename, arcname=filename)
            archive.writestr("manifest.json", json.dumps({
                "board": board,
                "segments": [{"asset": name, "sha256": digest}
                             for name, digest in digests.items()],
            }))
        expected[board] = digests
    return expected


class BoardRegistryTests(unittest.TestCase):

    def test_the_supported_boards_are_the_expected_ones(self):
        self.assertEqual({V1, V21}, set(BOARDS))

    def test_the_v21_target_is_marked_beta(self):
        """Si esta placa se valida en hardware hay que quitar beta, y el cambio debe ser deliberado."""
        self.assertTrue(BOARDS[V21]["beta"], "el target no validado debe ir marcado como beta")
        self.assertFalse(BOARDS[V1]["beta"])

    def test_every_board_points_at_a_real_environment(self):
        ini = (ROOT / "platformio.ini").read_text(encoding="utf-8")
        for board, info in BOARDS.items():
            self.assertIn(f"[env:{info['env']}]", ini, f"{board} apunta a un entorno inexistente")


class WebFlasherSiteTests(unittest.TestCase):
    """El sitio y la release deben publicar los MISMOS binarios, por placa."""

    def _build(self, tmp: Path, boards, version="v9.9.9"):
        binaries_root = tmp / "published"
        expected = write_fake_bundles(tmp / "release-assets", version, boards)
        output = tmp / "site"
        result = build_site(version, "0123456789abcdef0123", tmp / "build", None, output,
                            binaries_root, boards)
        return output, result, expected

    def test_site_publishes_one_manifest_per_board(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            output, result, _ = self._build(tmp, [V1, V21])
            published = {info["board"] for info in result["boards"]}
            self.assertEqual({V1, V21}, published)
            for info in result["boards"]:
                self.assertTrue((output / info["manifest"]).is_file(), info["manifest"])
                self.assertTrue((output / "firmware" / info["board"] / "SHA256SUMS").is_file())

    def test_site_hashes_come_from_the_given_binaries(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            output, _result, expected = self._build(tmp, [V1, V21])
            for board, digests in expected.items():
                sums = {}
                for line in (output / "firmware" / board / "SHA256SUMS").read_text().splitlines():
                    digest, name = line.split(maxsplit=1)
                    sums[name.strip()] = digest
                self.assertEqual(digests, sums, board)

    def test_site_copies_binaries_verbatim(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            output, _result, _ = self._build(tmp, [V1, V21])
            for board in (V1, V21):
                for filename, _address in SEGMENTS:
                    self.assertEqual(
                        (tmp / "published" / board / filename).read_bytes(),
                        (output / "firmware" / board / filename).read_bytes(),
                        f"{board}/{filename} no se copio literalmente",
                    )

    def test_build_id_is_inside_each_build(self):
        """ESP Web Tools lee builds[].buildId; uno en la raiz se ignora en silencio."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            output, _result, _ = self._build(tmp, [V1, V21])
            for board in (V1, V21):
                manifest = json.loads((output / f"manifest-{board}.json").read_text())
                self.assertIn("buildId", manifest["builds"][0])
                self.assertNotIn("buildId", manifest)
                self.assertIn(board, manifest["builds"][0]["buildId"])

    def test_build_id_changes_with_the_commit(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            write_fake_bundles(tmp / "release-assets", "v1.0.0", [V1])
            write_fake_binaries(tmp / "published" / V1, V1)
            first = build_site("v1.0.0", "a" * 40, tmp / "build", None, tmp / "s1",
                               tmp / "published", [V1])
            second = build_site("v1.0.0", "b" * 40, tmp / "build", None, tmp / "s2",
                                tmp / "published", [V1])
            self.assertNotEqual(first["manifests"][V1]["builds"][0]["buildId"],
                                second["manifests"][V1]["builds"][0]["buildId"])

    def test_build_info_exposes_the_board_list_with_beta_flags(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            output, _result, _ = self._build(tmp, [V1, V21])
            info = json.loads((output / "build-info.json").read_text())
            flags = {b["board"]: b["beta"] for b in info["boards"]}
            self.assertEqual({V1: False, V21: True}, flags)

    def test_unknown_board_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            with self.assertRaises(ValueError):
                build_site("v9.9.9", "abc", tmp / "build", None, tmp / "site", None,
                           ["placa-inexistente"])

    def test_cli_accepts_binaries_root(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            write_fake_bundles(tmp / "release-assets", "v9.9.9", [V1, V21])
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "build_web_flasher.py"),
                 "--version", "v9.9.9", "--commit", "0123456789abcdef0123",
                 "--binaries-root", str(tmp / "release-assets" / ".." / "published"),
                 "--output", str(tmp / "site")],
                capture_output=True, text=True, cwd=ROOT,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            info = json.loads((tmp / "site" / "build-info.json").read_text())
            self.assertEqual({V1, V21}, {b["board"] for b in info["boards"]})


class VerifySiteHashesTests(unittest.TestCase):
    """El gate que usa CI antes de desplegar el sitio."""

    def _run(self, tmp: Path, boards):
        write_fake_bundles(tmp / "release-assets", "v9.9.9", boards)
        build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, tmp / "site",
                   tmp / "published", boards)
        return subprocess.run(
            [sys.executable, str(ROOT / "tools" / "verify_site_hashes.py"),
             "--release-assets", str(tmp / "release-assets"), "--site", str(tmp / "site")],
            capture_output=True, text=True, cwd=ROOT,
        )

    def test_matching_site_passes_for_all_boards(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run(Path(tmp), [V1, V21])
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn(f"OK [{V1}]", result.stdout)
            self.assertIn(f"OK [{V21}]", result.stdout)

    def test_a_diverging_board_fails_the_deploy(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            # El sitio se genera y luego se corrompe UN binario del V21.
            self._run(tmp, [V1, V21])
            target = tmp / "site" / "firmware" / V21 / "firmware.bin"
            target.write_bytes(b"corrupto")
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "verify_site_hashes.py"),
                 "--release-assets", str(tmp / "release-assets"), "--site", str(tmp / "site")],
                capture_output=True, text=True, cwd=ROOT,
            )
            self.assertEqual(1, result.returncode)
            self.assertIn("FALLO", result.stdout)
            self.assertIn("firmware.bin", result.stdout)

    def test_missing_board_in_the_site_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            # La release tiene dos placas pero el sitio solo una.
            write_fake_bundles(tmp / "release-assets", "v9.9.9", [V1, V21])
            build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, tmp / "site",
                       tmp / "published", [V1])
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "verify_site_hashes.py"),
                 "--release-assets", str(tmp / "release-assets"), "--site", str(tmp / "site")],
                capture_output=True, text=True, cwd=ROOT,
            )
            self.assertEqual(1, result.returncode)
            self.assertIn(V21, result.stderr)

    def test_no_bundles_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / "release-assets").mkdir()
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "verify_site_hashes.py"),
                 "--release-assets", str(tmp / "release-assets"), "--site", str(tmp)],
                capture_output=True, text=True, cwd=ROOT,
            )
            self.assertEqual(1, result.returncode)


if __name__ == "__main__":
    unittest.main()
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

from tools.build_web_flasher import build_site
from tools.package_firmware_release import SEGMENTS

ROOT = Path(__file__).resolve().parents[2]


def write_fake_binaries(directory: Path) -> dict:
    """Crea los cuatro binarios con contenido distintivo y devuelve su sha256."""
    digests = {}
    for index, (filename, _address) in enumerate(SEGMENTS):
        payload = f"contenido-sintetico-{filename}-{index}".encode()
        (directory / filename).write_bytes(payload)
        digests[filename] = hashlib.sha256(payload).hexdigest()
    return digests


class VerifySiteHashesTests(unittest.TestCase):
    """El gate que usa CI antes de desplegar el sitio."""

    def _write(self, directory: Path, digests: dict):
        (directory / "manifest.json").write_text(json.dumps(
            {"segments": [{"asset": name, "sha256": digest} for name, digest in digests.items()]},
            indent=2), encoding="utf-8")
        (directory / "SHA256SUMS").write_text(
            "\n".join(f"{digest}  {name}" for name, digest in digests.items()) + "\n",
            encoding="utf-8")

    def _run(self, tmp: Path, digests: dict, sums_digests: dict):
        self._write(tmp, digests)
        sums = tmp / "SHA256SUMS"
        sums.write_text(
            "\n".join(f"{d}  {n}" for n, d in sums_digests.items()) + "\n", encoding="utf-8")
        return subprocess.run(
            [sys.executable, str(ROOT / "tools" / "verify_site_hashes.py"),
             "--manifest", str(tmp / "manifest.json"), "--sha256sums", str(sums)],
            capture_output=True, text=True, cwd=ROOT,
        )

    def test_matching_hashes_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            digests = {"firmware.bin": "a" * 64, "bootloader.bin": "b" * 64}
            result = self._run(Path(tmp), digests, digests)
            self.assertEqual(0, result.returncode, result.stderr)
            self.assertIn("OK", result.stdout)

    def test_mismatched_hash_fails_the_deploy(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run(Path(tmp), {"firmware.bin": "a" * 64}, {"firmware.bin": "c" * 64})
            self.assertEqual(1, result.returncode)
            self.assertIn("FALLO", result.stdout)

    def test_missing_asset_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run(Path(tmp), {"firmware.bin": "a" * 64, "partitions.bin": "d" * 64},
                               {"firmware.bin": "a" * 64})
            self.assertEqual(1, result.returncode)
            self.assertIn("ausentes", result.stdout)

    def test_extra_asset_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self._run(Path(tmp), {"firmware.bin": "a" * 64},
                               {"firmware.bin": "a" * 64, "extra.bin": "e" * 64})
            self.assertEqual(1, result.returncode)
            self.assertIn("no en la release", result.stdout)


class WebFlasherHashTests(unittest.TestCase):
    """El sitio y la release deben publicar los MISMOS binarios.

    Las builds de ESP-IDF no son reproducibles byte a byte, asi que si el flasheador
    recompila en vez de reutilizar los binarios de la release, sus SHA256SUMS no
    coincidiran nunca con el release-manifest.json. Estos tests fijan ese invariante.
    """

    def test_site_hashes_come_from_the_given_binaries(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            expected = write_fake_binaries(published)

            output = tmp / "site"
            manifest = build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, output,
                                  published)

            sums = {}
            for line in (output / "firmware" / "SHA256SUMS").read_text().splitlines():
                digest, name = line.split(maxsplit=1)
                sums[name.strip()] = digest
            self.assertEqual(expected, sums)

            # El manifiesto de la release describe los mismos cuatro assets.
            self.assertEqual(
                sorted(expected),
                sorted(Path(part["path"]).name for part in manifest["builds"][0]["parts"]),
            )

    def test_site_copies_binaries_verbatim(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            write_fake_binaries(published)
            output = tmp / "site"
            build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, output, published)
            for filename, _address in SEGMENTS:
                self.assertEqual(
                    (published / filename).read_bytes(),
                    (output / "firmware" / filename).read_bytes(),
                    f"{filename} no se copio literalmente",
                )

    def test_manifest_has_build_id_so_updates_are_detected(self):
        """Sin buildId, ESP Web Tools no avisa de que hay version nueva."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            write_fake_binaries(published)
            output = tmp / "site"
            manifest = build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, output,
                                  published)
            self.assertIn("buildId", manifest)
            self.assertIn("v9.9.9", manifest["buildId"])
            self.assertIn("0123456789ab", manifest["buildId"])

    def test_build_id_changes_with_the_commit(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            write_fake_binaries(published)
            first = build_site("v1.0.0", "a" * 40, tmp / "build", None, tmp / "s1", published)
            second = build_site("v1.0.0", "b" * 40, tmp / "build", None, tmp / "s2", published)
            self.assertNotEqual(first["buildId"], second["buildId"])

    def test_release_zip_and_site_agree_end_to_end(self):
        """El flujo real del CI: empaquetar y luego generar el sitio desde ese paquete."""
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            expected = write_fake_binaries(published)

            bundle = tmp / "bundle.zip"
            with zipfile.ZipFile(bundle, "w") as archive:
                for filename, _address in SEGMENTS:
                    archive.write(published / filename, arcname=filename)

            unpacked = tmp / "unpacked"
            unpacked.mkdir()
            with zipfile.ZipFile(bundle) as archive:
                archive.extractall(unpacked)

            output = tmp / "site"
            build_site("v9.9.9", "0123456789abcdef0123", tmp / "build", None, output, unpacked)
            sums = dict(
                line.split(maxsplit=1)[::-1] for line in
                (output / "firmware" / "SHA256SUMS").read_text().splitlines()
            )
            self.assertEqual({name.strip(): digest for name, digest in sums.items()}, expected)

    def test_cli_accepts_binaries_dir(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            published = tmp / "published"
            published.mkdir()
            write_fake_binaries(published)
            output = tmp / "site"
            result = subprocess.run(
                [sys.executable, str(ROOT / "tools" / "build_web_flasher.py"),
                 "--version", "v9.9.9", "--commit", "0123456789abcdef0123",
                 "--binaries-dir", str(published), "--output", str(output)],
                capture_output=True, text=True, cwd=ROOT,
            )
            self.assertEqual(0, result.returncode, result.stderr)
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertIn("buildId", manifest)


if __name__ == "__main__":
    unittest.main()
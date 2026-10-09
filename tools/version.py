"""PlatformIO pre-script: inyecta la version del firmware en tiempo de compilacion.

Antes esto era un literal en `config.h` (`"FlyWithMe V1.0"`) que se quedo obsoleto en la primera
release y nadie volvio a mirarlo: la pantalla de arranque seguia anunciando V1.0 con el firmware en
v1.2.x.

No se usa `git describe` a secas. En este repositorio MIENTE en `develop`: los tags viven en commits
de merge de `main` que `develop` no contiene, asi que `git describe` desde develop responde
`v1.0.0-17-g848f1d3` — una version que sugiere estar 17 commits despues de v1.0.0, cuando el
firmware es posterior a v1.2.1. Reglas:

  1. Si hay variable de entorno FWM_BUILD_VERSION (la pasa el workflow de release), se usa tal cual.
  2. Si HEAD es EXACTAMENTE un tag, se usa el tag. Es el caso de una build de release por tag.
  3. En cualquier otro caso, el SHA corto. Siempre es exacto y nunca engana.

Se exponen dos defines:
  FWM_VERSION      cadena corta para la pantalla OLED (p. ej. "v1.2.1" o "848f1d3")
  FWM_VERSION_FULL cadena larga para logs ("FlyWithMe v1.2.1" o "848f1d3-dirty")
"""

import os
import subprocess

Import("env")  # noqa: F821  (lo provee PlatformIO)


def git(*args):
    try:
        out = subprocess.run(("git",) + args, capture_output=True, text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return ""
    return out.stdout.strip() if out.returncode == 0 else ""


def short_version():
    from_env = os.environ.get("FWM_BUILD_VERSION", "").strip()
    if from_env:
        return from_env

    exact_tag = git("describe", "--tags", "--exact-match")
    if exact_tag:
        return exact_tag

    sha = git("rev-parse", "--short", "HEAD")
    return sha or "unknown"


def is_dirty():
    return bool(git("status", "--porcelain"))


version = short_version()
full = f"FlyWithMe {version}" + ("-dirty" if is_dirty() else "")

# CPPDEFINES y no BUILD_FLAGS: BUILD_FLAGS son cadenas y un valor con espacio (por ejemplo
# "FlyWithMe 848f1d3-dirty") se parte en dos argumentos, con lo que el trozo suelto acaba
# interpretado como una libreria y el enlazado falla con "cannot find -l848f1d3-dirty".
# StringifyMacro produce la forma \\"valor\\" ya citada.
env.Append(CPPDEFINES=[  # noqa: F821
    ("FWM_VERSION", env.StringifyMacro(version)),  # noqa: F821
    ("FWM_VERSION_FULL", env.StringifyMacro(full)),  # noqa: F821
])

print(f"[version] OLED/version banner: {version}")

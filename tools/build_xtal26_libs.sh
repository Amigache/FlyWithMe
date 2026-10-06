#!/usr/bin/env bash
#
# Reconstruye las librerias de Arduino-ESP32 para el chip ESP32 con el CRISTAL a 26 MHz.
#
# Motivo: las placas TTGO LoRa32 V1.0 llevan cristal de 26 MHz, pero las librerias
# precompiladas de PlatformIO se compilan a 40 MHz -> la WiFi/BT queda fuera de banda
# (no emite AP ni escanea redes). El LoRa no se ve afectado (SX1276 con cristal propio).
#
# Este script produce un juego de librerias identico al que usa PlatformIO
# (IDF release/v5.5 @ 87912cd291, arduino-esp32 3.3.x) salvo el cristal, que queda a 26 MHz.
#
# Requisitos: Linux (Ubuntu 22.04+), ~10 GB libres, buena conexion a GitHub.
# Uso:
#   bash tools/build_xtal26_libs.sh
#
# Resultado: ~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32
#   -> comprimelo y pasalo:  tar -C ~/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs -czf esp32-libs-xtal26.tar.gz esp32
#
set -euo pipefail

# 0) Red: evita el tipico fallo "HTTP/2 framing layer" en clones grandes de GitHub
git config --global http.version HTTP/1.1 || true
git config --global http.postBuffer 524288000 || true

# 1) Dependencias (omite si ya las tienes)
if command -v apt-get >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y git wget curl flex bison gperf cmake ninja-build ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0 python3 python3-pip python3-venv jq
fi

# 2) Clonar el builder oficial (tag correspondiente a IDF 5.5 / core 3.x)
cd "$HOME"
rm -rf esp32-arduino-lib-builder
git clone -b idf-release_v5.5 https://github.com/espressif/esp32-arduino-lib-builder
cd esp32-arduino-lib-builder

# 3) Cristal 26 MHz en la config de esp32
cat >> configs/defconfig.esp32 <<'EOF'

# --- FlyWithMe: cristal 26 MHz (TTGO LoRa32 V1.0) ---
# CONFIG_XTAL_FREQ_40 is not set
CONFIG_XTAL_FREQ_26=y
CONFIG_XTAL_FREQ=26
EOF

echo ">> defconfig.esp32 (xtal):"
tail -5 configs/defconfig.esp32

# 4) Compilar para ESP32, fijando IDF release/v5.5 @ 87912cd291 y arduino-esp32 idf-release/v5.5
./build.sh -t esp32 -I release/v5.5 -i 87912cd291 -A idf-release/v5.5

# 5) Resultado
OUT="$HOME/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs/esp32"
echo
echo ">> LIBRERIAS EN: $OUT"
ls -la "$OUT"
echo
echo ">> Para enviarmelas:"
echo "   tar -C \"$HOME/esp32-arduino-lib-builder/out/tools/esp32-arduino-libs\" -czf \"$HOME/esp32-libs-xtal26.tar.gz\" esp32"
echo "   (y pasame $HOME/esp32-libs-xtal26.tar.gz)"

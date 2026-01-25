#!/usr/bin/env bash
set -euo pipefail

export PLATFORMIO_HOME_DIR="${PLATFORMIO_HOME_DIR:-$(pwd)/.platformio-home}"

pio run -t upload -e esp32dev --upload-port /dev/ttyUSB0

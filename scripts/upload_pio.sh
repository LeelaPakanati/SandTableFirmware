#!/usr/bin/env bash
set -euo pipefail

export PLATFORMIO_HOME_DIR="${PLATFORMIO_HOME_DIR:-$(pwd)/.platformio-home}"

pio run -t upload

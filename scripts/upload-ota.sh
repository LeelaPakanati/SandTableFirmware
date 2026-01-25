#!/bin/bash
# OTA Upload Script for Sisyphus Table
# Usage: ./upload-ota.sh [IP_ADDRESS]

set -e

# Default IP if not provided
IP="${1:-100.76.149.200}"
AUTH="sandpatterns"
PORT=3232

echo "=== Sisyphus Table OTA Upload ==="
echo "Target IP: $IP"
echo ""

# Check if device is reachable
echo "Checking device connectivity..."
if ! ping -c 1 -W 2 "$IP" > /dev/null 2>&1; then
    echo "ERROR: Device at $IP is not reachable"
    echo "Make sure the Sisyphus Table is powered on and connected to WiFi"
    exit 1
fi
echo "Device is reachable"
echo ""

# Build firmware first
echo "Building firmware..."
pio run -e esp32dev

FIRMWARE=".pio/build/esp32dev/firmware.bin"
if [ ! -f "$FIRMWARE" ]; then
    echo "ERROR: Firmware not found at $FIRMWARE"
    exit 1
fi

# Find espota.py
ESPOTA=$(find ~/.platformio -name "espota.py" 2>/dev/null | head -1)
if [ -z "$ESPOTA" ]; then
    echo "ERROR: espota.py not found in ~/.platformio"
    exit 1
fi

echo ""
echo "Uploading firmware via OTA..."
python3 "$ESPOTA" -i "$IP" -p "$PORT" -a "$AUTH" -f "$FIRMWARE" -d -r

echo ""
echo "=== Upload Complete ==="
echo "The device will reboot automatically"

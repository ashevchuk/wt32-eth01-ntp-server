#!/usr/bin/env bash
# Builds if needed and flashes bootloader, partition table and firmware over the serial port.
# Usage: flash-serial.sh [port]    (default: $PORT or /dev/ttyACM0)
# Required for the first flash and after any change of the partition table.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

PORT="${1:-${PORT:-/dev/ttyACM0}}"
if [ ! -c "$PORT" ]; then
    echo "$PORT is not a character device" >&2
    exit 1
fi
idf.py -p "$PORT" flash

#!/usr/bin/env bash
# Builds the firmware image into build/ntp.bin. Extra arguments go to idf.py.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

if [ ! -f sdkconfig ]; then
    idf.py set-target esp32
fi
idf.py build "$@"
echo "Image: $PROJECT_ROOT/build/ntp.bin"

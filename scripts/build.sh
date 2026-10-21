#!/usr/bin/env bash
# Builds the firmware image into build/ntp.bin. Extra arguments go to idf.py.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]}")/env.sh"

if [ ! -f sdkconfig ]; then
    idf.py set-target esp32
fi
idf.py build "$@"
# A crash dump can only be decoded with the exact ELF it came from; the device reports the SHA-256 prefix.
elf_sha="$(sha256sum build/ntp.elf | cut -c1-16)"
mkdir -p build/elf
cp build/ntp.elf "build/elf/$elf_sha.elf"
echo "ELF: $PROJECT_ROOT/build/elf/$elf_sha.elf"
echo "Image: $PROJECT_ROOT/build/ntp.bin"

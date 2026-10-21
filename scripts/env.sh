# Sourced by the other scripts: puts idf.py on PATH and moves to the project root.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDF_PATH="${IDF_PATH:-/opt/esp-idf}"
export IDF_PATH

if ! command -v idf.py >/dev/null; then
    if [ ! -f "$IDF_PATH/export.sh" ]; then
        echo "ESP-IDF not found: no idf.py on PATH and no $IDF_PATH/export.sh" >&2
        exit 1
    fi
    # shellcheck disable=SC1091
    . "$IDF_PATH/export.sh" >/dev/null
fi

cd "$PROJECT_ROOT"

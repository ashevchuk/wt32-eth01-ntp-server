#!/usr/bin/env bash
# Uploads a firmware image to the running device and waits until it boots from the other slot.
# Usage: flash-ota.sh <host> [image]    (host may also come from $NTP_HOST; image defaults to build/ntp.bin)
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${1:-${NTP_HOST:-}}"
IMAGE="${2:-$PROJECT_ROOT/build/ntp.bin}"
RESTART_TIMEOUT_S=120
FRESH_BOOT_UPTIME_S=90

if [ -z "$HOST" ]; then
    echo "usage: $0 <host> [image]" >&2
    exit 2
fi
if [ ! -f "$IMAGE" ]; then
    echo "image not found: $IMAGE (run scripts/build.sh first)" >&2
    exit 1
fi

# Prints one string or number field of /api/status; fails when the answer is not a status document.
status_field() {
    local status="$1" field="$2"
    case "$status" in
        *'"uptime_s"'*) ;;
        *) echo "unexpected answer from $HOST: $status" >&2; return 1 ;;
    esac
    printf '%s' "$status" | grep -o "\"$field\":\"\?[^\",}]*" | head -n1 | sed "s/\"$field\":\"\?//"
}

before="$(curl -sS --fail -m 5 "http://$HOST/api/status")"
slot_before="$(status_field "$before" partition)"
version_before="$(status_field "$before" version)"
echo "Running $version_before from $slot_before, uploading $(basename "$IMAGE")"

# The device restarts right after answering, so the connection may drop before the answer arrives. Only an
# explicit rejection is a failure here; otherwise the slot change below decides.
set +e
response="$(curl -sS -m 120 -w '\n%{http_code}' -X POST -H 'Content-Type: application/octet-stream' \
    --data-binary "@$IMAGE" "http://$HOST/api/ota")"
curl_status=$?
set -e
if [ "$curl_status" -ne 0 ]; then
    echo "Connection lost while uploading (curl exit $curl_status); checking whether the device installed the image" >&2
else
    http_code="${response##*$'\n'}"
    body="${response%$'\n'*}"
    case "$body" in
        *'"ok"'*) ;;
        *) echo "unexpected answer from the OTA endpoint (HTTP $http_code): $body" >&2; exit 1 ;;
    esac
    if [ "$http_code" != 200 ]; then
        echo "upload rejected (HTTP $http_code): $body" >&2
        exit 1
    fi
fi
echo "Uploaded, waiting for the device to restart"

deadline=$((SECONDS + RESTART_TIMEOUT_S))
sleep 5
while [ "$SECONDS" -lt "$deadline" ]; do
    # Connection errors are expected while the device restarts, so they only mean "not yet".
    if after="$(curl -s --fail -m 3 "http://$HOST/api/status")"; then
        uptime="$(status_field "$after" uptime_s)"
        if [ "$uptime" -lt "$FRESH_BOOT_UPTIME_S" ]; then
            slot_after="$(status_field "$after" partition)"
            version_after="$(status_field "$after" version)"
            if [ "$slot_after" = "$slot_before" ]; then
                echo "Device came back on the same slot ($slot_after): the new image was rejected or rolled back" >&2
                exit 1
            fi
            echo "Now running $version_after from $slot_after"
            exit 0
        fi
    fi
    sleep 2
done
echo "Device did not come back within ${RESTART_TIMEOUT_S}s" >&2
exit 1

#!/usr/bin/env bash
set -euo pipefail

GENERATOR="$(realpath "$1")"
WORK_DIR="$(mktemp -d)"
trap 'rm -rf "${WORK_DIR}"' EXIT

OUTPUT_DIR="${WORK_DIR}/assets/wireless_test_frames/sequence"
(
    cd "${WORK_DIR}"
    "${GENERATOR}" --width 64 --height 48 --count 2
)

test -s "${OUTPUT_DIR}/frame_000.png"
test -s "${OUTPUT_DIR}/frame_001.png"

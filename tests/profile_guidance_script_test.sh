#!/usr/bin/env bash

set -euo pipefail

SCRIPT="$1"
PROJECT_ROOT="$2"

bash -n "${SCRIPT}"
HELP_OUTPUT="$(${SCRIPT} --help)"
grep -q -- "--config PATH" <<<"${HELP_OUTPUT}"
grep -q -- "--duration SECONDS" <<<"${HELP_OUTPUT}"
grep -q -- "真实相机" <<<"${HELP_OUTPUT}"

TEMP_BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "${TEMP_BUILD_DIR}"' EXIT

set +e
ERROR_OUTPUT="$(${SCRIPT} --skip-build --build-dir "${TEMP_BUILD_DIR}" 2>&1)"
STATUS=$?
set -e

[[ "${STATUS}" -ne 0 ]]
grep -q -- "找不到可执行文件" <<<"${ERROR_OUTPUT}"
[[ -d "${PROJECT_ROOT}" ]]

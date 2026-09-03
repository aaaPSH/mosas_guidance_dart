#!/usr/bin/env bash

set -euo pipefail

APP="$1"

HELP_OUTPUT="$(${APP} --help)"
grep -q -- "--config PATH" <<<"${HELP_OUTPUT}"

set +e
ERROR_OUTPUT="$(${APP} --config /definitely/missing/mosas.conf 2>&1)"
STATUS=$?
set -e

[[ "${STATUS}" -eq 2 ]]
grep -q -- "无法读取配置文件" <<<"${ERROR_OUTPUT}"

#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/serial-frame-debug"
SKIP_BUILD=0
APP_ARGS=()

usage() {
    cat <<'EOF'
用法：run_serial_frame_debug.sh [选项] [--] [调试程序参数]

选项：
  --build-dir PATH       构建目录，默认 build/serial-frame-debug
  --skip-build           跳过 CMake 配置和编译
  -h, --help             显示帮助

调试程序参数放在 -- 后，例如：
  -- --device /dev/ttyS3 --baud-rate 115200
EOF
}

while (($# > 0)); do
    case "$1" in
        --build-dir)
            [[ $# -ge 2 ]] || { echo "--build-dir 缺少值" >&2; exit 2; }
            BUILD_DIR="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=1
            shift
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        --)
            shift
            APP_ARGS=("$@")
            break
            ;;
        *)
            echo "未知参数：$1；调试程序参数请放在 -- 后" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" \
        -DMOSAS_BUILD_GUIDANCE_APP=OFF \
        -DMOSAS_BUILD_WIRELESS=OFF
    cmake --build "${BUILD_DIR}" --target serial_frame_debug_app --parallel
fi

APP="${BUILD_DIR}/serial_frame_debug_app"
if [[ ! -x "${APP}" ]]; then
    echo "找不到可执行文件：${APP}，请先移除 --skip-build 或检查构建目录" >&2
    exit 1
fi

exec "${APP}" "${APP_ARGS[@]}"

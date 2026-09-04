#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${PROJECT_ROOT}/config/mosas_guidance.conf"
BUILD_DIR="${PROJECT_ROOT}/build/guidance-app"
SKIP_BUILD=0
APP_ARGS=()

usage() {
    cat <<'EOF'
用法：run_guidance.sh [选项] [--] [主程序参数]

选项：
  --config PATH       统一配置文件，默认 config/mosas_guidance.conf
  --build-dir PATH   构建目录，默认 build/guidance-app
  --skip-build       跳过 CMake 配置和编译
  -h, --help         显示帮助

主程序参数应放在 -- 后；模块参数请修改配置文件。
EOF
}

while (($# > 0)); do
    case "$1" in
        --config)
            [[ $# -ge 2 ]] || { echo "--config 缺少值" >&2; exit 2; }
            CONFIG_PATH="$2"
            shift 2
            ;;
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
            echo "未知参数：$1；模块参数请写入配置文件，主程序参数请放在 -- 后" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}"
    cmake --build "${BUILD_DIR}" --target mosas_guidance_app --parallel
fi

APP="${BUILD_DIR}/mosas_guidance_app"
if [[ ! -x "${APP}" ]]; then
    echo "找不到可执行文件：${APP}，请先移除 --skip-build 或检查构建目录" >&2
    exit 1
fi

exec "${APP}" --config "${CONFIG_PATH}" "${APP_ARGS[@]}"

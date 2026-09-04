#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_PATH="${PROJECT_ROOT}/config/mosas_guidance.conf"
BUILD_DIR="${PROJECT_ROOT}/build/guidance-profile"
REPORT_PATH="/tmp/mosas_guidance_timing.log"
DURATION_SECONDS=10
SKIP_BUILD=0

usage() {
    cat <<'EOF'
用法：profile_guidance.sh [选项]

使用真实相机运行完整制导流水线，并输出各阶段耗时报告。
请将真实相机设备、分辨率和像素格式写入配置文件的 [camera] 节。

选项：··
  --config PATH          统一配置文件，默认 config/mosas_guidance.conf
  --duration SECONDS     测试时长，默认 10 秒
  --build-dir PATH       构建目录，默认 build/guidance-profile
  --report PATH          报告文件，默认 /tmp/mosas_guidance_timing.log
  --skip-build           跳过 CMake 配置和编译
  -h, --help             显示帮助
EOF
}

while (($# > 0)); do
    case "$1" in
        --config)
            [[ $# -ge 2 ]] || { echo "--config 缺少值" >&2; exit 2; }
            CONFIG_PATH="$2"
            shift 2
            ;;
        --duration)
            [[ $# -ge 2 ]] || { echo "--duration 缺少值" >&2; exit 2; }
            DURATION_SECONDS="$2"
            shift 2
            ;;
        --build-dir)
            [[ $# -ge 2 ]] || { echo "--build-dir 缺少值" >&2; exit 2; }
            BUILD_DIR="$2"
            shift 2
            ;;
        --report)
            [[ $# -ge 2 ]] || { echo "--report 缺少值" >&2; exit 2; }
            REPORT_PATH="$2"
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
        *)
            echo "未知参数：$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if ! [[ "${DURATION_SECONDS}" =~ ^[1-9][0-9]*$ ]]; then
    echo "--duration 必须是正整数秒数" >&2
    exit 2
fi

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" -DBUILD_TESTING=OFF
    cmake --build "${BUILD_DIR}" --target mosas_guidance_app --parallel
fi

APP="${BUILD_DIR}/mosas_guidance_app"
if [[ ! -x "${APP}" ]]; then
    echo "找不到可执行文件：${APP}，请先移除 --skip-build 或检查构建目录" >&2
    exit 1
fi

REPORT_PARENT="$(dirname "${REPORT_PATH}")"
mkdir -p "${REPORT_PARENT}"
echo "使用真实相机开始 ${DURATION_SECONDS} 秒耗时测试，报告：${REPORT_PATH}"
"${APP}" --config "${CONFIG_PATH}" --timing >"${REPORT_PATH}" 2>&1 &
APP_PID=$!

cleanup() {
    if kill -0 "${APP_PID}" 2>/dev/null; then
        kill -INT "${APP_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

sleep "${DURATION_SECONDS}"
cleanup

set +e
wait "${APP_PID}"
STATUS=$?
set -e

cat "${REPORT_PATH}"
if [[ "${STATUS}" -ne 0 ]]; then
    echo "真实相机耗时测试失败，退出码：${STATUS}" >&2
    exit "${STATUS}"
fi
if ! grep -q '^\[TIMING\]' "${REPORT_PATH}"; then
    echo "报告中没有耗时统计，请确认程序运行时间和真实相机配置" >&2
    exit 1
fi
echo "耗时测试完成，报告已保存到：${REPORT_PATH}"

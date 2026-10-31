#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/guidance-app"
BUILD_DIR_SPECIFIED=0
SKIP_BUILD=0

usage() {
    cat <<'EOF'
用法：run_sync_calibration.sh [构建选项]

选项：
  --build-dir PATH   构建目录，默认 build/guidance-app
  --skip-build       跳过 CMake 配置和编译
  -h, --help         显示帮助

标定程序无命令行参数；源码版读取 config/mosas_sync_calibration.conf，
安装版读取安装配置目录中的 mosas_sync_calibration.conf（默认 <prefix>/etc/mosas/）。
采集目录、时长、采样率、预算和耗时统计均在对应文件中修改；构建选项仅用于源码树。
始终禁止控制输出；执行器仍须物理断能。
离线分析另用 calibrate_camera_imu_sync.py。
EOF
}

while (($# > 0)); do
    case "$1" in
        --build-dir)
            [[ $# -ge 2 ]] || { echo "--build-dir 缺少值" >&2; exit 2; }
            BUILD_DIR="$2"
            BUILD_DIR_SPECIFIED=1
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
            echo "未知参数：$1；标定参数请修改 config/mosas_sync_calibration.conf" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -f "${PROJECT_ROOT}/CMakeLists.txt" ]]; then
    # 切换工作目录前保留调用方指定的相对构建路径。
    [[ "${BUILD_DIR}" = /* ]] || BUILD_DIR="${PWD}/${BUILD_DIR}"
    if [[ "${SKIP_BUILD}" -eq 0 ]]; then
        cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}"
        cmake --build "${BUILD_DIR}" --target mosas_sync_calibration_app --parallel
    fi
    APP="${BUILD_DIR}/mosas_sync_calibration_app"
else
    if [[ "${BUILD_DIR_SPECIFIED}" -ne 0 ]]; then
        echo "安装版不接受 --build-dir，请直接启动已安装程序" >&2
        exit 2
    fi
    APP="${SCRIPT_DIR}/mosas_sync_calibration_app"
fi
if [[ ! -x "${APP}" ]]; then
    echo "找不到可执行文件：${APP}，请检查构建目录或安装是否完整" >&2
    exit 1
fi

cd "${PROJECT_ROOT}"
exec "${APP}"

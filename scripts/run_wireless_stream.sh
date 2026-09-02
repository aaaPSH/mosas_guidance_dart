#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/wireless-stream"
SKIP_BUILD=0

usage() {
    echo "用法: $0 [--build-dir PATH] [--skip-build] [wireless_stream_probe 参数...]"
    echo "示例: $0 --url rtsp://127.0.0.1:8554/mosas --device /dev/video0 --width 640 --height 480"
    echo "测试图案: $0 --pattern --url rtsp://127.0.0.1:8554/mosas --frames 300"
    echo "图片序列: $0 --image-dir assets/wireless_test_frames/sequence --url rtsp://127.0.0.1:8554/mosas"
}

PROBE_ARGS=()
while [[ $# -gt 0 ]]; do
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
        *)
            PROBE_ARGS+=("$1")
            shift
            ;;
    esac
done

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" -DBUILD_TESTING=OFF
    cmake --build "${BUILD_DIR}" \
        --target wireless_stream_probe wireless_test_frame_generator \
        -j"$(nproc)"
fi

exec "${BUILD_DIR}/wireless_stream_probe" "${PROBE_ARGS[@]}"

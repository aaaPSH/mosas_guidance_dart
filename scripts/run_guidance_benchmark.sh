#!/usr/bin/env bash

set -euo pipefail

# 在目标板上构建并运行制导计算性能测试。
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
BUILD_DIR="${GUIDANCE_BENCH_BUILD_DIR:-${PROJECT_ROOT}/build/guidance-benchmark}"
WIDTH="${GUIDANCE_BENCH_WIDTH:-640}"
HEIGHT="${GUIDANCE_BENCH_HEIGHT:-480}"
ITERATIONS="${GUIDANCE_BENCH_ITERATIONS:-10000}"
MODE="all"
SKIP_BUILD=0

usage() {
    cat <<'EOF'
用法：run_guidance_benchmark.sh [选项]

选项：
  --width N          图像宽度，默认 640
  --height N         图像高度，默认 480
  --iterations N     测量次数，默认 10000
  --mode MODE        all、full-scan 或 roi，默认 all
  --build-dir PATH   Release 构建目录
  --skip-build       跳过 CMake 配置和编译
  -h, --help         显示帮助
EOF
}

while (($# > 0)); do
    case "$1" in
        --width)
            WIDTH="$2"
            shift 2
            ;;
        --height)
            HEIGHT="$2"
            shift 2
            ;;
        --iterations)
            ITERATIONS="$2"
            shift 2
            ;;
        --mode)
            MODE="$2"
            shift 2
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=1
            shift
            ;;
        -h|--help)
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

if ! [[ "$WIDTH" =~ ^[1-9][0-9]*$ && "$HEIGHT" =~ ^[1-9][0-9]*$ && "$ITERATIONS" =~ ^[1-9][0-9]*$ ]]; then
    echo "width、height 和 iterations 必须是正整数" >&2
    exit 2
fi
case "$MODE" in
    all|full-scan|roi) ;;
    *)
        echo "mode 必须是 all、full-scan 或 roi" >&2
        exit 2
        ;;
esac

echo "=== guidance benchmark environment ==="
uname -a
if [[ -r /proc/cpuinfo ]]; then
    awk -F': ' '/^(Hardware|Model name|Processor|CPU architecture):/ {print; seen[$1]++} seen[$1] == 1' /proc/cpuinfo
fi
getconf _NPROCESSORS_ONLN 2>/dev/null || true
echo "project_root=${PROJECT_ROOT}"
echo "build_dir=${BUILD_DIR}"
echo "resolution=${WIDTH}x${HEIGHT} iterations=${ITERATIONS}"

if [[ "$SKIP_BUILD" -eq 0 ]]; then
    cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF
    cmake --build "$BUILD_DIR" --target guidance_timing_probe --parallel
fi

PROBE="${BUILD_DIR}/guidance_timing_probe"
if [[ ! -x "$PROBE" ]]; then
    echo "找不到可执行文件：${PROBE}，请先移除 --skip-build 或检查构建目录" >&2
    exit 1
fi

run_probe() {
    local mode="$1"
    echo "=== mode=${mode} ==="
    if [[ "$mode" == "full-scan" ]]; then
        "$PROBE" --width "$WIDTH" --height "$HEIGHT" \
            --iterations "$ITERATIONS" --full-scan
    else
        "$PROBE" --width "$WIDTH" --height "$HEIGHT" \
            --iterations "$ITERATIONS"
    fi
}

case "$MODE" in
    all)
        run_probe full-scan
        run_probe roi
        ;;
    full-scan|roi)
        run_probe "$MODE"
        ;;
esac

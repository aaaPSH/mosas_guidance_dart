#!/usr/bin/env bash

# 将当前 Git 跟踪的文件同步到目标目录。
set -Eeuo pipefail

SOURCE_INPUT="${SYNC_SOURCE:-$(dirname "${BASH_SOURCE[0]}")}"
DESTINATION_INPUT="${SYNC_DEST:-/mnt/c/temp}"

if ! command -v git >/dev/null 2>&1; then
  echo "错误：未找到 git，无法获取跟踪文件清单。" >&2
  exit 127
fi
if ! command -v rsync >/dev/null 2>&1; then
  echo "错误：未找到 rsync，请先安装 rsync。" >&2
  exit 127
fi
if ! command -v realpath >/dev/null 2>&1; then
  echo "错误：未找到 realpath，无法安全校验路径。" >&2
  exit 127
fi

if ! SOURCE_DIR="$(realpath -e -- "${SOURCE_INPUT}")" || [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "错误：源目录不存在或不可访问：${SOURCE_INPUT}" >&2
  exit 1
fi
if ! SOURCE_DIR="$(git -C "${SOURCE_DIR}" rev-parse --show-toplevel 2>/dev/null)" || \
  ! SOURCE_DIR="$(realpath -e -- "${SOURCE_DIR}")"; then
  echo "错误：源目录不是 Git 仓库：${SOURCE_INPUT}" >&2
  exit 1
fi
if [[ -z "${DESTINATION_INPUT}" ]]; then
  echo "错误：目标目录不能为空。" >&2
  exit 1
fi
if ! DESTINATION_DIR="$(realpath -m -- "${DESTINATION_INPUT}")"; then
  echo "错误：无法解析目标目录：${DESTINATION_INPUT}" >&2
  exit 1
fi

path_is_same_or_nested() {
  local path="$1"
  local parent="$2"
  if [[ "${parent}" == "/" ]]; then
    [[ "${path}" == /* ]]
    return
  fi
  [[ "${path}" == "${parent}" || "${path}" == "${parent}/"* ]]
}

if [[ "${DESTINATION_DIR}" == "/" ]] || \
  path_is_same_or_nested "${DESTINATION_DIR}" "${SOURCE_DIR}" || \
  path_is_same_or_nested "${SOURCE_DIR}" "${DESTINATION_DIR}"; then
  echo "错误：目标目录不能是源目录本身、源目录的子目录或父目录。" >&2
  exit 1
fi

if ! mkdir -p "${DESTINATION_DIR}"; then
  echo "错误：无法创建或访问目标目录：${DESTINATION_DIR}" >&2
  exit 1
fi
if ! DESTINATION_DIR="$(realpath -e -- "${DESTINATION_DIR}")"; then
  echo "错误：无法解析目标目录：${DESTINATION_DIR}" >&2
  exit 1
fi
if [[ "${DESTINATION_DIR}" == "/" ]] || \
  path_is_same_or_nested "${DESTINATION_DIR}" "${SOURCE_DIR}" || \
  path_is_same_or_nested "${SOURCE_DIR}" "${DESTINATION_DIR}"; then
  echo "错误：目标目录不能是源目录本身、源目录的子目录或父目录。" >&2
  exit 1
fi

# Git 的 NUL 分隔清单可以安全处理空格、换行和二进制文件名。
echo "正在同步 Git 跟踪文件到：${DESTINATION_DIR}"
if ! git -C "${SOURCE_DIR}" ls-files -z | \
  rsync --archive --recursive --from0 --files-from=- \
    "${SOURCE_DIR}/" "${DESTINATION_DIR}/"; then
  echo "错误：同步失败。" >&2
  exit 1
fi
echo "同步完成。"

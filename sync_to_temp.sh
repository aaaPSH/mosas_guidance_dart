#!/usr/bin/env bash

# 将项目中的源代码和项目文本文件镜像同步到目标目录。
set -Eeuo pipefail

SOURCE_INPUT="${SYNC_SOURCE:-$(dirname "${BASH_SOURCE[0]}")}"
DESTINATION_INPUT="${SYNC_DEST:-/mnt/c/temp}"

if ! command -v rsync >/dev/null 2>&1; then
  echo "错误：未找到 rsync，请先安装 rsync。" >&2
  exit 127
fi
if ! command -v realpath >/dev/null 2>&1; then
  echo "错误：未找到 realpath，无法安全校验路径。" >&2
  exit 127
fi
if ! command -v grep >/dev/null 2>&1; then
  echo "错误：未找到 grep，无法识别文本文件。" >&2
  exit 127
fi
if ! command -v find >/dev/null 2>&1; then
  echo "错误：未找到 find，无法枚举源文件。" >&2
  exit 127
fi

if ! SOURCE_DIR="$(realpath -e -- "${SOURCE_INPUT}")" || [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "错误：源目录不存在或不可访问：${SOURCE_INPUT}" >&2
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

# 判断文件名是否属于项目源代码、构建配置或项目文本文件。
is_allowed_file() {
  local relative_path="$1"
  case "${relative_path}" in
    assets/*)
      return 0 ;;
    *.c|*.cc|*.cpp|*.cxx|*.h|*.hh|*.hpp|*.hxx|*.inl|*.dart|*.py|*.js|*.ts|*.java|*.go|*.rs|*.cs|*.swift|*.kt|*.m|*.mm)
      return 0 ;;
    *.cmake|CMakeLists.txt|Makefile|GNUmakefile|makefile|*.mk|*.gradle|*.gradle.kts)
      return 0 ;;
    *.sh|*.bash|README*|LICENSE*|NOTICE*|*.md|*.txt|*.json|*.yaml|*.yml|*.toml|*.xml|*.ini|*.cfg|*.conf|*.lock|*.in|*.properties)
      return 0 ;;
    .gitignore|.clang-format|.editorconfig|Dockerfile|.dockerignore|BUILD|WORKSPACE)
      return 0 ;;
    *)
      return 1 ;;
  esac
}

# 文件名符合白名单且内容不含 NUL 字节时，才认定为文本文件。
is_text_file() {
  local file_path="$1"
  local grep_status

  if [[ ! -s "${file_path}" ]]; then
    return 0
  fi
  if LC_ALL=C grep -Iq . -- "${file_path}"; then
    return 0
  else
    grep_status=$?
  fi
  if ((grep_status == 1)); then
    return 1
  fi
  echo "错误：无法读取文件：${file_path}" >&2
  return 2
}

# assets 目录允许同步二进制资源，例如 PNG、JPEG 和视频文件。
is_asset_file() {
  case "$1" in
    assets/*)
      return 0 ;;
    *)
      return 1 ;;
  esac
}

if ! WORK_DIR="$(mktemp -d)"; then
  echo "错误：无法创建临时目录。" >&2
  exit 1
fi
cleanup() {
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

STAGING_DIR="${WORK_DIR}/staging"
SOURCE_FILE_LIST="${WORK_DIR}/source-files"
if ! mkdir -p "${STAGING_DIR}"; then
  echo "错误：无法创建临时 staging 目录。" >&2
  exit 1
fi

if ! find "${SOURCE_DIR}" -mindepth 1 -type d \
  \( -name '.git' -o -name '.agents' -o -name '.codex' -o -name 'build' -o -name 'docs' -o -name 'tests' \) -prune -o \
  -type f -print0 > "${SOURCE_FILE_LIST}"; then
  echo "错误：无法完整枚举源文件。" >&2
  exit 1
fi

shopt -s nocasematch
SOURCE_FILES=()
while IFS= read -r -d '' candidate; do
  relative_path="${candidate#"${SOURCE_DIR}/"}"
  if is_allowed_file "${relative_path}"; then
    if is_asset_file "${relative_path}" || is_text_file "${candidate}"; then
      SOURCE_FILES+=("${relative_path}")
    else
      text_status=$?
      if ((text_status != 1)); then
        exit "${text_status}"
      fi
    fi
  fi
done < "${SOURCE_FILE_LIST}"
shopt -u nocasematch

RSYNC_FILE_OPTIONS=(--archive --recursive --from0)

echo "正在将源文件镜像同步到：${DESTINATION_DIR}"
if ((${#SOURCE_FILES[@]} > 0)); then
  if ! printf '%s\0' "${SOURCE_FILES[@]}" | \
    rsync "${RSYNC_FILE_OPTIONS[@]}" --files-from=- \
      "${SOURCE_DIR}/" "${STAGING_DIR}/"; then
    echo "错误：同步失败。" >&2
    exit 1
  fi
fi

if ! rsync --archive --delete --delete-excluded --prune-empty-dirs \
  "${STAGING_DIR}/" "${DESTINATION_DIR}/"; then
  echo "错误：同步失败。" >&2
  exit 1
fi
echo "同步完成。"

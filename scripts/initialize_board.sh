#!/usr/bin/env bash

# 初始化 A133 板卡的网络、UART0 登录和构建工具链。
set -euo pipefail

INIT_ROOT="${MOSAS_INIT_ROOT:-}"

target_path() {
    printf '%s%s\n' "${INIT_ROOT}" "$1"
}

fail() {
    printf '错误：%s\n' "$1" >&2
    exit 1
}

usage() {
    cat <<'EOF'
用法：initialize_board.sh --ssid SSID --wifi-password PASSWORD

也可通过 WIFI_SSID 和 WIFI_PASSWORD 环境变量提供 Wi-Fi 凭据。
EOF
}

wifi_ssid="${WIFI_SSID:-}"
wifi_password="${WIFI_PASSWORD:-}"

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        --ssid)
            [[ "$#" -ge 2 ]] || fail '--ssid 缺少值'
            wifi_ssid="$2"
            shift 2
            ;;
        --wifi-password)
            [[ "$#" -ge 2 ]] || fail '--wifi-password 缺少值'
            wifi_password="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            fail "未知参数：$1"
            ;;
    esac
done

[[ -n "${wifi_ssid}" ]] || fail '必须提供 Wi-Fi SSID'
[[ -n "${wifi_password}" ]] || fail '必须提供 Wi-Fi 密码'
[[ "$(id -u)" == 0 ]] || fail '初始化必须以 root 身份执行'

for command_name in nmcli apt-get systemctl chpasswd install mktemp rm cp ip; do
    command -v "${command_name}" >/dev/null 2>&1 || \
        fail "缺少必需命令：${command_name}"
done

os_release="$(target_path /etc/os-release)"
[[ -r "${os_release}" ]] || fail '无法读取 /etc/os-release'
grep -Eq '^ID=ubuntu$' "${os_release}" || fail '仅支持 Ubuntu'
grep -Eq '^VERSION_ID="?22\.04"?$' "${os_release}" || fail '仅支持 Ubuntu 22.04'

tty_s0="$(target_path /dev/ttyS0)"
[[ -e "${tty_s0}" ]] || fail '缺少调试串口 ttyS0'

wifi_profile='mosas-initialize-wifi'
if ! nmcli connection show "${wifi_profile}" >/dev/null 2>&1; then
    nmcli connection add type wifi ifname wlan0 con-name "${wifi_profile}" ssid "${wifi_ssid}" \
        >/dev/null
fi
nmcli connection modify "${wifi_profile}" wifi.ssid "${wifi_ssid}" \
    wifi-sec.key-mgmt wpa-psk connection.autoconnect yes \
    >/dev/null

password_file="$(umask 077; mktemp)"
cleanup_password_file() {
    rm -f -- "${password_file}"
}
trap cleanup_password_file EXIT
printf '802-11-wireless-security.psk:%s\n' "${wifi_password}" >"${password_file}"
if ! nmcli connection up id "${wifi_profile}" ifname wlan0 passwd-file "${password_file}" \
    >/dev/null 2>&1; then
    fail '无法连接 Wi-Fi'
fi
cleanup_password_file
trap - EXIT

DEBIAN_FRONTEND=noninteractive apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y g++ cmake libopencv-dev

systemd_dir="$(target_path /etc/systemd/system)"
getty_dir="${systemd_dir}/serial-getty@ttyS0.service.d"
getty_config="${getty_dir}/autologin.conf"
getty_backup=""
getty_existed=0
getty_configured=0
if [[ -e "${getty_config}" ]]; then
    getty_existed=1
    getty_backup="$(mktemp)"
    cp -a -- "${getty_config}" "${getty_backup}"
fi

rollback_getty_config() {
    local exit_status=$?
    if [[ "${exit_status}" -ne 0 && "${getty_configured}" -eq 0 ]]; then
        if [[ "${getty_existed}" -eq 1 ]]; then
            cp -a -- "${getty_backup}" "${getty_config}"
        else
            rm -f -- "${getty_config}"
        fi
        systemctl daemon-reload >/dev/null 2>&1 || true
    fi
    [[ -z "${getty_backup}" ]] || rm -f -- "${getty_backup}"
    exit "${exit_status}"
}
trap rollback_getty_config EXIT

install -d -m 0755 "${getty_dir}"
cat >"${getty_config}" <<'EOF'
[Service]
ExecStart=
ExecStart=-/sbin/agetty --autologin root --noclear --keep-baud 115200,57600,38400,9600 %I $TERM
EOF

systemctl daemon-reload
printf 'root:root\n' | chpasswd
getty_configured=1
trap - EXIT
[[ -z "${getty_backup}" ]] || rm -f -- "${getty_backup}"

printf '板端初始化完成。\n'
wifi_ipv4="$(ip -4 -o addr show dev wlan0 scope global 2>/dev/null | \
    awk 'NR == 1 { print $4 }' || true)"
if [[ -n "${wifi_ipv4}" ]]; then
    printf 'Wi-Fi IPv4 地址：%s\n' "${wifi_ipv4}"
else
    printf '警告：wlan0 暂未获得 IPv4 地址。\n' >&2
fi

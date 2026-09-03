#!/usr/bin/env bash

# Restore the original BlueZ configuration and remove the broken Xbox bond.

set -Eeuo pipefail

readonly DEFAULT_CONTROLLER_MAC='9C:AA:1B:4C:A7:7A'
readonly CONTROLLER_MAC="${1:-${XBOX_CONTROLLER_MAC:-${DEFAULT_CONTROLLER_MAC}}}"
readonly BLUEZ_CONFIG='/etc/bluetooth/main.conf'
readonly ORIGINAL_CONFIG='/etc/bluetooth/main.conf.teleop_joy.bak'

bluetooth_stopped=false

cleanup() {
    if [[ "${bluetooth_stopped}" == true ]]; then
        sudo systemctl start bluetooth >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT INT TERM

die() {
    echo "오류: $*" >&2
    exit 1
}

[[ "${CONTROLLER_MAC}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] \
    || die "MAC 주소 형식이 잘못되었습니다: ${CONTROLLER_MAC}"
for required_command in bluetoothctl sudo systemctl modprobe; do
    command -v "${required_command}" >/dev/null 2>&1 \
        || die "필요한 명령이 없습니다: ${required_command}"
done
[[ -f "${ORIGINAL_CONFIG}" ]] \
    || die "BlueZ 원본 백업이 없습니다: ${ORIGINAL_CONFIG}"

if bluetoothctl show 2>/dev/null | grep -Fq 'Discovering: yes'; then
    die "Bluetooth 검색이 실행 중입니다. 열린 bluetoothctl에서 'scan off', 'quit'을 실행하십시오."
fi

adapter_mac="$(bluetoothctl list 2>/dev/null | awk 'NR == 1 { print $2 }')"
[[ "${adapter_mac}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] \
    || die 'Bluetooth 어댑터 MAC 주소를 찾지 못했습니다.'

device_dir="/var/lib/bluetooth/${adapter_mac}/${CONTROLLER_MAC}"
backup_root='/var/lib/bluetooth/teleop_joy-backups'
timestamp="$(date +%Y%m%d-%H%M%S)"

echo '컨트롤러를 완전히 끈 상태인지 확인하십시오.'
echo 'BlueZ 원본 설정을 복구하고 기존 Xbox 본딩을 백업한 뒤 제거합니다.'
sudo systemctl stop bluetooth
bluetooth_stopped=true

sudo cp --preserve=mode,ownership,timestamps \
    "${BLUEZ_CONFIG}" "${BLUEZ_CONFIG}.before-reset-${timestamp}"
sudo cp --preserve=mode,ownership,timestamps \
    "${ORIGINAL_CONFIG}" "${BLUEZ_CONFIG}"

if sudo test -d "${device_dir}"; then
    sudo mkdir -p -- "${backup_root}"
    sudo chmod 0700 "${backup_root}"
    sudo mv -- "${device_dir}" \
        "${backup_root}/${adapter_mac}_${CONTROLLER_MAC}_${timestamp}"
    echo "기존 본딩 백업: ${backup_root}/${adapter_mac}_${CONTROLLER_MAC}_${timestamp}"
fi

# xpadneo must be present before BlueZ creates the HID device, otherwise the
# malformed first probe is attempted by hid-microsoft and rebound afterwards.
sudo modprobe uhid
sudo modprobe hid_xpadneo
sudo systemctl start bluetooth
bluetooth_stopped=false
sleep 2
bluetoothctl --timeout 3 power on >/dev/null 2>&1 || true

echo '초기화 완료:'
bluetoothctl show | grep -E 'Powered:|Discovering:' || true
lsmod | grep -E '^(hid_xpadneo|uhid)[[:space:]]' || true
echo
echo '컨트롤러를 빠른 점멸 상태로 만든 뒤 ./xbox_pair.sh를 한 번만 실행하십시오.'

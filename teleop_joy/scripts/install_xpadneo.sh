#!/usr/bin/env bash

# Install the official xpadneo Bluetooth driver for the current Raspberry Pi kernel.

set -Eeuo pipefail

readonly XPADNEO_VERSION='v0.10.4'
readonly DEFAULT_XPADNEO_DIR="${HOME}/xpadneo"
readonly XPADNEO_DIR="${XPADNEO_DIR:-${DEFAULT_XPADNEO_DIR}}"
readonly KERNEL_VERSION="$(uname -r)"
readonly KERNEL_HEADERS="linux-headers-${KERNEL_VERSION}"

die() {
    echo "오류: $*" >&2
    exit 1
}

for required_command in apt-get git sudo uname; do
    command -v "${required_command}" >/dev/null 2>&1 \
        || die "필요한 명령이 없습니다: ${required_command}"
done

if [[ ! -d "${XPADNEO_DIR}/.git" ]]; then
    echo "xpadneo ${XPADNEO_VERSION} 다운로드: ${XPADNEO_DIR}"
    git clone --branch "${XPADNEO_VERSION}" --depth 1 \
        https://github.com/atar-axis/xpadneo.git "${XPADNEO_DIR}"
fi

installed_tag="$(git -C "${XPADNEO_DIR}" describe --tags --exact-match 2>/dev/null || true)"
[[ "${installed_tag}" == "${XPADNEO_VERSION}" ]] \
    || die "${XPADNEO_DIR}가 검증된 ${XPADNEO_VERSION} 태그가 아닙니다: ${installed_tag:-unknown}"

echo "현재 커널: ${KERNEL_VERSION}"
echo "xpadneo: ${XPADNEO_VERSION}"
echo '컨트롤러를 완전히 끈 상태에서 설치를 계속합니다.'
echo 'DKMS와 현재 커널 헤더 설치를 위해 sudo 암호가 필요할 수 있습니다.'

sudo apt-get install -y dkms "${KERNEL_HEADERS}"
sudo modprobe uhid

if sudo dkms status 2>/dev/null \
    | grep -Eq "^hid-xpadneo/${XPADNEO_VERSION}.*installed"; then
    echo "DKMS에 이미 설치되어 있습니다: hid-xpadneo/${XPADNEO_VERSION}"
else
    (
        cd -- "${XPADNEO_DIR}"
        sudo ./install.sh
    )
fi

# Loading the driver before a paired controller reconnects avoids the first
# connection being initialized by hid-microsoft and rebound a second time.
printf 'hid_xpadneo\n' | sudo tee /etc/modules-load.d/xpadneo.conf >/dev/null
sudo modprobe hid_xpadneo
sudo udevadm control --reload

echo
echo 'xpadneo 설치 확인:'
sudo dkms status | grep -F 'hid-xpadneo' || true
modinfo hid_xpadneo | grep -E '^(filename|version|description|alias):' | head -n 12
lsmod | grep -E '^(hid_xpadneo|uhid)[[:space:]]' || true
echo
echo '설치 완료. 컨트롤러를 켠 뒤 ./xbox_connect.sh를 실행하십시오.'

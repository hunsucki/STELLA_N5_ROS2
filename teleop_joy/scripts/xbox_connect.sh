#!/usr/bin/env bash

# Reconnect an already paired Xbox controller while its logo is slowly
# flashing. Discovery stays active because this hardware combination needs it.

set -Eeuo pipefail

readonly DEFAULT_CONTROLLER_MAC='9C:AA:1B:4C:A7:7A'
readonly CONTROLLER_MAC="${1:-${XBOX_CONTROLLER_MAC:-${DEFAULT_CONTROLLER_MAC}}}"
readonly SCAN_TIMEOUT_SEC=60

scan_pid=''
connect_pid=''
work_dir="$(mktemp -d)"

cleanup() {
    local pid
    for pid in "${connect_pid}" "${scan_pid}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
            wait "${pid}" 2>/dev/null || true
        fi
    done
    bluetoothctl --timeout 2 scan off >/dev/null 2>&1 || true
    rm -rf -- "${work_dir}"
}
trap cleanup EXIT INT TERM

die() {
    echo "오류: $*" >&2
    exit 1
}

has_property() {
    local property="$1"
    bluetoothctl info "${CONTROLLER_MAC}" 2>/dev/null \
        | grep -Fq "${property}: yes"
}

wait_for_connection() {
    local attempt
    for attempt in {1..40}; do
        if has_property Connected; then
            return 0
        fi
        if ! kill -0 "${connect_pid}" 2>/dev/null; then
            return 1
        fi
        sleep 1
    done
    return 1
}

[[ "${CONTROLLER_MAC}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] \
    || die "MAC 주소 형식이 잘못되었습니다: ${CONTROLLER_MAC}"
command -v bluetoothctl >/dev/null 2>&1 \
    || die "bluetoothctl이 없습니다. bluez 패키지를 설치하십시오."

has_property Paired \
    || die '페어링 정보가 없습니다. xbox_pair.sh를 먼저 실행하십시오.'
has_property Bonded \
    || die '본딩 정보가 없습니다. xbox_pair.sh를 먼저 실행하십시오.'

if has_property Connected; then
    echo "이미 연결되어 있습니다: ${CONTROLLER_MAC}"
    exit 0
fi

echo "Xbox 컨트롤러 재연결: ${CONTROLLER_MAC}"
echo 'Xbox 버튼이 일반(느린) 점멸 상태인지 확인하십시오.'

bluetoothctl --timeout 2 power on >/dev/null 2>&1 \
    || die 'Bluetooth 전원을 켤 수 없습니다.'

bluetoothctl --timeout "${SCAN_TIMEOUT_SEC}" scan on \
    >"${work_dir}/scan.log" 2>&1 &
scan_pid=$!
sleep 2
kill -0 "${scan_pid}" 2>/dev/null \
    || die "검색 시작에 실패했습니다: $(cat "${work_dir}/scan.log")"

echo '검색을 유지한 상태로 연결 중...'
bluetoothctl --timeout 45 connect "${CONTROLLER_MAC}" \
    >"${work_dir}/connect.log" 2>&1 &
connect_pid=$!
if ! wait_for_connection; then
    cat "${work_dir}/connect.log" >&2
    die '연결에 실패했습니다. 컨트롤러가 켜져 있는지 확인하십시오.'
fi
kill "${connect_pid}" 2>/dev/null || true
wait "${connect_pid}" 2>/dev/null || true
connect_pid=''

bluetoothctl --timeout 2 trust "${CONTROLLER_MAC}" >/dev/null 2>&1 \
    || die '컨트롤러 trust 설정에 실패했습니다.'

echo '재연결 완료: Trusted/Connected=yes'
bluetoothctl info "${CONTROLLER_MAC}" | grep -E \
    'Name:|Paired:|Bonded:|Trusted:|Connected:|Battery Percentage:' || true

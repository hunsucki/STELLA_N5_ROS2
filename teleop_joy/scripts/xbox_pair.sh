#!/usr/bin/env bash

# Pair and connect an Xbox controller while it is rapidly flashing.
# This Raspberry Pi/controller combination requires discovery to stay active
# throughout both pairing and connection.

set -Eeuo pipefail

readonly DEFAULT_CONTROLLER_MAC='9C:AA:1B:4C:A7:7A'
readonly CONTROLLER_MAC="${1:-${XBOX_CONTROLLER_MAC:-${DEFAULT_CONTROLLER_MAC}}}"
readonly SCAN_TIMEOUT_SEC=90

scan_pid=''
pair_pid=''
connect_pid=''
work_dir="$(mktemp -d)"

cleanup() {
    local pid
    for pid in "${pair_pid}" "${connect_pid}" "${scan_pid}"; do
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

wait_for_device() {
    local attempt
    for attempt in {1..20}; do
        if bluetoothctl info "${CONTROLLER_MAC}" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_pairing() {
    local pid="$1"
    local attempt
    for attempt in {1..45}; do
        if has_property Paired && has_property Bonded; then
            return 0
        fi
        if ! kill -0 "${pid}" 2>/dev/null; then
            return 1
        fi
        sleep 1
    done
    return 1
}

wait_for_connection() {
    local pid="$1"
    local attempt
    for attempt in {1..40}; do
        if has_property Connected; then
            return 0
        fi
        if ! kill -0 "${pid}" 2>/dev/null; then
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

echo "Xbox 컨트롤러 페어링: ${CONTROLLER_MAC}"
echo '컨트롤러의 페어링 버튼을 빠르게 점멸할 때까지 누르십시오.'

bluetoothctl --timeout 2 power on >/dev/null 2>&1 \
    || die 'Bluetooth 전원을 켤 수 없습니다.'
bluetoothctl --timeout 2 pairable on >/dev/null 2>&1 \
    || die 'Bluetooth 어댑터를 pairable 상태로 만들 수 없습니다.'

# Keep a separate bluetoothctl client alive so discovery remains active while
# the pair/connect clients run.
bluetoothctl --timeout "${SCAN_TIMEOUT_SEC}" scan on \
    >"${work_dir}/scan.log" 2>&1 &
scan_pid=$!
sleep 2
kill -0 "${scan_pid}" 2>/dev/null \
    || die "검색 시작에 실패했습니다: $(cat "${work_dir}/scan.log")"

wait_for_device \
    || die '컨트롤러를 찾지 못했습니다. 빠른 점멸 상태와 MAC을 확인하십시오.'

if ! has_property Paired; then
    echo '검색을 유지한 상태로 페어링 중...'
    bluetoothctl --agent NoInputNoOutput --timeout 60 \
        pair "${CONTROLLER_MAC}" >"${work_dir}/pair.log" 2>&1 &
    pair_pid=$!
    if ! wait_for_pairing "${pair_pid}"; then
        cat "${work_dir}/pair.log" >&2
        die '페어링에 실패했습니다.'
    fi
    kill "${pair_pid}" 2>/dev/null || true
    wait "${pair_pid}" 2>/dev/null || true
    pair_pid=''
else
    echo '이미 페어링된 컨트롤러입니다.'
fi

bluetoothctl --timeout 2 trust "${CONTROLLER_MAC}" >/dev/null 2>&1 \
    || die '컨트롤러 trust 설정에 실패했습니다.'

if ! has_property Connected; then
    echo '검색을 유지한 상태로 연결 중...'
    bluetoothctl --timeout 45 connect "${CONTROLLER_MAC}" \
        >"${work_dir}/connect.log" 2>&1 &
    connect_pid=$!
    if ! wait_for_connection "${connect_pid}"; then
        cat "${work_dir}/connect.log" >&2
        die '컨트롤러 연결에 실패했습니다.'
    fi
    kill "${connect_pid}" 2>/dev/null || true
    wait "${connect_pid}" 2>/dev/null || true
    connect_pid=''
fi

sleep 1
has_property Paired || die '최종 상태에서 Paired=yes가 아닙니다.'
has_property Bonded || die '최종 상태에서 Bonded=yes가 아닙니다.'
has_property Trusted || die '최종 상태에서 Trusted=yes가 아닙니다.'
has_property Connected || die '최종 상태에서 Connected=yes가 아닙니다.'

echo '페어링 및 연결 완료: Paired/Bonded/Trusted/Connected=yes'
bluetoothctl info "${CONTROLLER_MAC}" | grep -E \
    'Name:|Paired:|Bonded:|Trusted:|Connected:|Battery Percentage:' || true

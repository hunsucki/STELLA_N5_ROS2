#!/usr/bin/env bash

# Reconnect an already paired Xbox controller while its logo is slowly flashing.

set -Eeuo pipefail

readonly DEFAULT_CONTROLLER_MAC='9C:AA:1B:4C:A7:7A'
readonly CONTROLLER_MAC="${1:-${XBOX_CONTROLLER_MAC:-${DEFAULT_CONTROLLER_MAC}}}"
readonly SCAN_TIMEOUT_SEC=75

scan_pid=''
work_dir="$(mktemp -d)"

stop_scan() {
    if [[ -n "${scan_pid}" ]] && kill -0 "${scan_pid}" 2>/dev/null; then
        kill "${scan_pid}" 2>/dev/null || true
        wait "${scan_pid}" 2>/dev/null || true
    fi
    scan_pid=''
    bluetoothctl --timeout 2 scan off >/dev/null 2>&1 || true
}

cleanup() {
    stop_scan
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

is_discovering() {
    bluetoothctl show 2>/dev/null | grep -Fq 'Discovering: yes'
}

discover_controller() {
    local attempt

    bluetoothctl --timeout "${SCAN_TIMEOUT_SEC}" scan on \
        >"${work_dir}/scan.log" 2>&1 &
    scan_pid=$!
    sleep 1
    kill -0 "${scan_pid}" 2>/dev/null \
        || die "검색 시작에 실패했습니다: $(cat "${work_dir}/scan.log")"

    for attempt in {1..30}; do
        if grep -Fq "${CONTROLLER_MAC}" "${work_dir}/scan.log"; then
            stop_scan
            return 0
        fi
        sleep 1
    done
    stop_scan
    return 1
}

wait_for_stable_connection() {
    local attempt stable=0
    for attempt in {1..20}; do
        if has_property Connected; then
            ((stable += 1))
            ((stable >= 5)) && return 0
        else
            stable=0
        fi
        sleep 1
    done
    return 1
}

verify_input_access() {
    local attempt event
    for attempt in {1..15}; do
        for event in /dev/input/event*; do
            [[ -e "${event}" ]] || continue
            if udevadm info -q property -n "${event}" 2>/dev/null \
                | grep -Fq 'ID_BUS=bluetooth' && \
               udevadm info -a -n "${event}" 2>/dev/null \
                | grep -Fq 'ATTRS{id/vendor}=="045e"'; then
                if [[ ! -r "${event}" ]]; then
                    echo "경고: ${event} 읽기 권한이 없어 ROS joy 입력이 동작하지 않습니다." >&2
                    echo 'stella_bringup/create_udev_rules.sh를 실행한 뒤 패드를 재연결하십시오.' >&2
                    return 1
                fi
                echo "Xbox 입력 장치 확인: ${event}"
                return 0
            fi
        done
        sleep 1
    done
    echo '경고: 연결은 되었지만 Bluetooth 게임패드 입력 장치를 찾지 못했습니다.' >&2
    return 1
}

[[ "${CONTROLLER_MAC}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] \
    || die "MAC 주소 형식이 잘못되었습니다: ${CONTROLLER_MAC}"
for required_command in bluetoothctl timeout udevadm; do
    command -v "${required_command}" >/dev/null 2>&1 \
        || die "필요한 명령이 없습니다: ${required_command}"
done

bluetoothctl --timeout 2 power on >/dev/null 2>&1 \
    || die 'Bluetooth 전원을 켤 수 없습니다.'

if is_discovering; then
    die "이미 다른 프로그램이 Bluetooth 검색을 사용 중입니다.
현재 열린 bluetoothctl에서 'scan off', 'quit'을 실행한 뒤 다시 시도하십시오."
fi

has_property Paired \
    || die '페어링 정보가 없습니다. 빠른 점멸 상태에서 xbox_pair.sh를 실행하십시오.'
has_property Bonded \
    || die '본딩 정보가 없습니다. 빠른 점멸 상태에서 xbox_pair.sh를 실행하십시오.'

bluetoothctl --timeout 5 trust "${CONTROLLER_MAC}" >/dev/null 2>&1 \
    || die '컨트롤러 trust 설정에 실패했습니다.'

if has_property Connected; then
    echo "이미 연결되어 있습니다: ${CONTROLLER_MAC}"
    wait_for_stable_connection || die 'Bluetooth 링크가 다시 끊겼습니다.'
    verify_input_access
    exit 0
fi

echo "Xbox 컨트롤러 재연결: ${CONTROLLER_MAC}"
echo 'Xbox 버튼이 일반(느린) 점멸 상태인지 확인하십시오.'

discover_controller \
    || die '컨트롤러 광고 패킷을 받지 못했습니다. 패드가 점멸하는지 확인하십시오.'
is_discovering && die '검색을 종료하지 못했습니다. 다른 Bluetooth 설정 창을 닫으십시오.'

echo '컨트롤러 발견 및 검색 종료. 연결 중...'
set +e
timeout --signal=TERM --kill-after=5 55 \
    bluetoothctl connect "${CONTROLLER_MAC}" \
    2>&1 | tee "${work_dir}/connect.log"
connect_rc=${PIPESTATUS[0]}
set -e

if ((connect_rc != 0)) || \
   grep -Fq 'Failed to connect:' "${work_dir}/connect.log" || \
   ! grep -Fq 'Connection successful' "${work_dir}/connect.log"; then
    die '연결에 실패했습니다. 패드가 켜져 있고 느리게 점멸하는지 확인하십시오.'
fi

is_discovering && die '검색이 계속 실행 중입니다. 다른 Bluetooth 설정 창을 닫으십시오.'
wait_for_stable_connection || die '연결 직후 Bluetooth 링크가 다시 끊겼습니다.'

echo '안정 연결 완료: Paired/Bonded/Trusted/Connected=yes'
bluetoothctl info "${CONTROLLER_MAC}" | grep -E \
    'Name:|Paired:|Bonded:|Trusted:|Connected:|Battery Percentage:' || true
verify_input_access

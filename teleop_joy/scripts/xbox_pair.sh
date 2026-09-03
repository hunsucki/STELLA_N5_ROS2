#!/usr/bin/env bash

# Pair and connect an Xbox controller while its logo is rapidly flashing.

set -Eeuo pipefail

readonly DEFAULT_CONTROLLER_MAC='9C:AA:1B:4C:A7:7A'
readonly CONTROLLER_MAC="${1:-${XBOX_CONTROLLER_MAC:-${DEFAULT_CONTROLLER_MAC}}}"
readonly SCAN_TIMEOUT_SEC=120
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly AGENT_HELPER="${SCRIPT_DIR}/xbox_bluez_agent.py"

scan_pid=''
agent_pid=''
state_root="${XDG_STATE_HOME:-${HOME}/.local/state}/teleop_joy"
mkdir -p -- "${state_root}"
work_dir="$(mktemp -d "${state_root}/pair.XXXXXX")"
started_at="$(date --iso-8601=seconds)"

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
    if [[ -n "${agent_pid}" ]] && kill -0 "${agent_pid}" 2>/dev/null; then
        kill "${agent_pid}" 2>/dev/null || true
        wait "${agent_pid}" 2>/dev/null || true
    fi
    bluetoothctl show >"${work_dir}/adapter-final.log" 2>&1 || true
    bluetoothctl info "${CONTROLLER_MAC}" \
        >"${work_dir}/device-final.log" 2>&1 || true
    journalctl -u bluetooth --since "${started_at}" --no-pager \
        -o short-precise >"${work_dir}/bluetooth-journal.log" 2>&1 || true
    echo "진단 로그: ${work_dir}"
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
    local log_file="$1"
    local attempt

    bluetoothctl --timeout "${SCAN_TIMEOUT_SEC}" scan on \
        >"${log_file}" 2>&1 &
    scan_pid=$!
    sleep 1
    kill -0 "${scan_pid}" 2>/dev/null \
        || die "검색 시작에 실패했습니다: $(cat "${log_file}")"

    for attempt in {1..30}; do
        if grep -Fq "${CONTROLLER_MAC}" "${log_file}"; then
            # BlueZ now has a fresh LE advertising report. Stop active
            # discovery before pairing so scan traffic cannot disrupt GATT.
            stop_scan
            return 0
        fi
        sleep 1
    done
    stop_scan
    return 1
}

wait_for_stable_pairing() {
    local attempt stable=0
    for attempt in {1..15}; do
        if has_property Paired && has_property Bonded; then
            ((stable += 1))
            ((stable >= 3)) && return 0
        else
            stable=0
        fi
        sleep 1
    done
    return 1
}

wait_for_stable_connection() {
    local attempt stable=0
    for attempt in {1..15}; do
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

print_failure_hint() {
    echo >&2
    echo '확인 사항:' >&2
    echo '  1. 다른 bluetoothctl/설정 창을 모두 닫으십시오.' >&2
    echo '  2. 실패할 때마다 페어링 버튼을 다시 길게 눌러 빠른 점멸을 새로 시작하십시오.' >&2
    echo '  3. ./reset_xbox_bluetooth.sh를 실행했다면 setup/tune 스크립트는 사용하지 마십시오.' >&2
}

[[ "${CONTROLLER_MAC}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] \
    || die "MAC 주소 형식이 잘못되었습니다: ${CONTROLLER_MAC}"
for required_command in bluetoothctl timeout udevadm python3 journalctl; do
    command -v "${required_command}" >/dev/null 2>&1 \
        || die "필요한 명령이 없습니다: ${required_command}"
done
[[ -r "${AGENT_HELPER}" ]] || die "BlueZ 에이전트가 없습니다: ${AGENT_HELPER}"

bluetoothctl --timeout 2 power on >/dev/null 2>&1 \
    || die 'Bluetooth 전원을 켤 수 없습니다.'

if is_discovering; then
    die "이미 다른 프로그램이 Bluetooth 검색을 사용 중입니다.
현재 열린 bluetoothctl에서 'scan off', 'quit'을 실행한 뒤 다시 시도하십시오."
fi

echo "Xbox 컨트롤러 새 페어링: ${CONTROLLER_MAC}"
echo '페어링 버튼을 Xbox 버튼이 빠르게 점멸할 때까지 누르십시오.'

bluetoothctl --timeout 2 pairable on >/dev/null 2>&1 \
    || die 'Bluetooth 어댑터를 pairable 상태로 만들 수 없습니다.'

python3 "${AGENT_HELPER}" >"${work_dir}/agent.log" 2>&1 &
agent_pid=$!
for attempt in {1..50}; do
    grep -Fq 'READY:' "${work_dir}/agent.log" 2>/dev/null && break
    if ! kill -0 "${agent_pid}" 2>/dev/null; then
        cat "${work_dir}/agent.log" >&2
        die 'BlueZ 인증 에이전트를 등록하지 못했습니다.'
    fi
    sleep 0.1
done
grep -Fq 'READY:' "${work_dir}/agent.log" \
    || die 'BlueZ 인증 에이전트 준비 시간이 초과되었습니다.'

# A failed Xbox pairing can leave a temporary BlueZ object. Removing it here
# is safe because this script is specifically for a fresh pairing operation.
bluetoothctl --timeout 5 remove "${CONTROLLER_MAC}" >/dev/null 2>&1 || true

discover_controller "${work_dir}/scan.log" || {
    print_failure_hint
    die '컨트롤러를 찾지 못했습니다.'
}
is_discovering && die '검색을 종료하지 못했습니다. 다른 Bluetooth 설정 창을 닫으십시오.'

echo '컨트롤러 발견 및 검색 종료. 페어링 중...'
set +e
timeout --signal=TERM --kill-after=5 70 \
    bluetoothctl pair "${CONTROLLER_MAC}" \
    2>&1 | tee "${work_dir}/pair.log"
pair_rc=${PIPESTATUS[0]}
set -e

if ((pair_rc != 0)) || \
   grep -Fq 'Failed to pair:' "${work_dir}/pair.log" || \
   ! grep -Fq 'Pairing successful' "${work_dir}/pair.log" || \
   ! wait_for_stable_pairing; then
    cat "${work_dir}/agent.log" >&2
    print_failure_hint
    die '페어링이 완료되지 않았습니다.'
fi

bluetoothctl --timeout 5 trust "${CONTROLLER_MAC}" \
    | tee "${work_dir}/trust.log"
has_property Trusted || die '컨트롤러 trust 설정에 실패했습니다.'

if ! has_property Connected; then
    echo '연결용 광고 패킷을 다시 확인합니다...'
    discover_controller "${work_dir}/connect-scan.log" || \
        die '컨트롤러 광고 패킷을 받지 못했습니다. 패드가 점멸하는지 확인하십시오.'
    echo '검색 종료 후 연결 중...'
    set +e
    timeout --signal=TERM --kill-after=5 55 \
        bluetoothctl connect "${CONTROLLER_MAC}" \
        2>&1 | tee "${work_dir}/connect.log"
    connect_rc=${PIPESTATUS[0]}
    set -e

    if ((connect_rc != 0)) || \
       grep -Fq 'Failed to connect:' "${work_dir}/connect.log" || \
       ! grep -Fq 'Connection successful' "${work_dir}/connect.log"; then
        print_failure_hint
        die '컨트롤러 연결 명령이 실패했습니다.'
    fi
fi

is_discovering && die '검색이 계속 실행 중입니다. 다른 Bluetooth 설정 창을 닫으십시오.'
wait_for_stable_connection || die '연결 직후 Bluetooth 링크가 다시 끊겼습니다.'

has_property Paired || die '최종 상태에서 Paired=yes가 아닙니다.'
has_property Bonded || die '최종 상태에서 Bonded=yes가 아닙니다.'
has_property Trusted || die '최종 상태에서 Trusted=yes가 아닙니다.'

echo '페어링 및 안정 연결 완료: Paired/Bonded/Trusted/Connected=yes'
bluetoothctl info "${CONTROLLER_MAC}" | grep -E \
    'Name:|Paired:|Bonded:|Trusted:|Connected:|Battery Percentage:' || true
verify_input_access

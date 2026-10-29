# battery

STELLA N5의 배터리 전압·션트 전류 측정과 XY-SK120 충전 출력을 제어한다.
`stella_bringup/launch/robot.launch.py`에서 포함하며, `launch_battery`로 실행 여부를 제어한다.

## 센서 및 계측 범위

2026-10-16 실물 I2C 확인 결과 주소 `0x40`의 센서는 **INA226**이다.
제조사 ID `0x5449`, 칩 ID `0x2260`을 확인했다. 기존 문서의 INA219 표기는 잘못됐다.
`auto`는 INA226 ID를 확인하며, 실제 INA219 장치를 쓰면 `sensor_type:=ina219`를 명시한다.

- INA226: 버스 전압 1.25mV/bit, 부호 있는 션트 전압 2.5µV/bit.
- INA219: 버스 전압 `(register >> 3) × 4mV`, 션트 전압 10µV/bit.
- SMBus의 바이트 순서를 변환한 다음 전류의 16비트 부호를 처리한다.
- 전류 = 션트 전압 / `shunt_resistor` (기본 **0.01Ω**, 실물 저항값 검증 필요).
- INA 전압과 충전기 단자 전압은 서로 다른 측정점이므로 평균 내지 않는다.

기준: [TI INA226 데이터시트](https://www.ti.com/lit/ds/symlink/ina226.pdf),
[TI INA219 데이터시트](https://www.ti.com/lit/ds/symlink/ina219.pdf).

**배터리 순충전 전류를 검증한 구성은 아니다.** 기존 연결 설명상 INA는 부하 측에 있으며,
충전 중 `/battery_state.current`는 기존 도킹 검증기 호환을 위해 SK120 출력 전류를 사용한다.
SK120 전류에는 배터리 외 부하로 흐르는 전류가 포함될 수 있다. 배선과 션트 저항을 실측하기 전에는
이 값을 배터리 순전류·충전량으로 적분하면 안 된다. 방전 중에는 INA 전류의 부호를 반전한다.
`percentage`도 기존 17.5~25.2V 선형 추정으로, 정확한 6S LiPo 잔량이나 완충 판정이 아니다.
노드에는 셀별 전압·온도·BMS 정보가 없으므로 팩 보호나 완충 제어를 대신하지 않는다.

## 충전 동작

1. SK120 Modbus에 연결하고 기존 출력이 OFF인지 확인한다. 이전 프로세스의 ON 상태를 인계하지 않는다.
2. ON 명령을 받으면 배터리 전압을 읽고, OFF → 전압/전류 설정 → ON 순서로 명령한다.
   명령 응답과 출력·설정 레지스터 readback을 확인한 경우에만 세션을 시작한다.
3. 각 설정 변경 후 `settle_interval`을 기다리고, 같은 설정에서 실제 전류 ≥0.05A를
   연속 2회 읽었으며 `ramp_interval`도 경과했을 때만 다음 단계로 올린다.
   기본 주기에서는 한 단계에 보통 8~10초가 걸린다. 목표값은 상한이며 실측 전류를 보장하지 않는다.
4. 안정화 대기 후 실제 전류 <0.05A가 3회 연속이면 마지막으로 확인된 하위 설정까지만 복구한다.
   복구 가능한 하위 단계가 없거나 그 단계에서도 전류가 사라지면 OFF한다.
   이 조건만으로 무선 커플링 불량, 입력 전원 문제, 완충 근접 상태를 구분할 수 없다.
5. 통신 실패 시 오래된 충전값을 무효화하고 램프업을 멈춘다. 연속 3회 실패하면 OFF를 시도하고
   포트를 닫아 재연결한다. 통신 장애만으로 물리적 도킹 해제를 단정하거나 전류 상한을 초기화하지 않는다.
6. I2C 오류에도 SK120 보호 처리는 먼저 수행한다. 센서 오류·전압 범위 이탈·설정 불일치·실측 출력 OFF는
   충전 세션을 중단한다. OFF 확인 실패는 로그에 남기고 다음 상태 확인에서 재시도한다.
7. 보호 정지 후 반복 ON으로 자동 재시작하지 않는다. 명시적인 OFF와 출력 OFF 확인 후에만 다시 ON할 수 있다.
   확인된 전류 상한은 프로세스 생존 기간 유지되며 노드 재시작 시 초기화된다.

`/sk120/available`은 기존 Nav2 호환 **통신 가능 신호**이다. 충전기 응답만으로 실제 도킹·입력 전원·충전을
증명하지 않는다. `/sk120/output_on`은 출력 레지스터 관측값이며 실측 전류와 별도로 확인한다.

## 토픽

| 방향 | 토픽 | 타입 | 의미 |
| --- | --- | --- | --- |
| 구독 | `/sk120/cmd_output` | `std_msgs/Bool` | true: 시작, false: 정지 및 확인된 보호 정지 해제 |
| 발행 | `/battery_state` | `sensor_msgs/BatteryState` | INA 전압, 선형 SoC 추정, 아래 상태와 전류 |
| 발행 | `/sk120/available` | `sensor_msgs/BatteryState` | present 및 CHARGING=통신 가능, DISCHARGING=미연결 (호환 신호) |
| 발행 | `/sk120/output_on` | `std_msgs/Bool` | 최근 확인된 출력 레지스터; 읽기 실패 중에는 발행하지 않음 |
| 발행 | `/sk120/current_set` | `std_msgs/Float32` | 실제 읽은 설정 전류 [A] |
| 발행 | `/sk120/current_out` | `std_msgs/Float32` | 충전기 실측 출력 전류 [A] |
| 발행 | `/sk120/voltage_out` | `std_msgs/Float32` | 충전기 실측 출력 전압 [V] |
| 발행 | `/sk120/voltage_in` | `std_msgs/Float32` | 충전기 실측 입력 전압 [V] |
| 발행 | `/sk120/power_out` | `std_msgs/Float32` | 충전기 실측 출력 전력 [W] |

상태 및 출력 계측은 OFF 상태에서도 계속 갱신한다. 통신 실패 시 전류·전압·설정 토픽은 NaN으로 무효화한다.
`output_on`은 Bool에 unknown 값이 없으므로 수신자는 마지막 수신 시각도 확인해야 한다.

`/battery_state.power_supply_status`:

- `CHARGING`: 요청 세션 + 최신 출력 ON + SK120 실측 전류 ≥0.05A. 배터리 순전류 검증은 별도다.
- `NOT_CHARGING`: 세션은 있으나 실측 전류가 기준 미만이다.
- `DISCHARGING`: 충전 요청 세션이 없고 INA가 정상이다.
- `UNKNOWN`: 센서 오류, 충전 중 통신 오류, 정지 요청에도 출력 ON이 관측되는 경우.

미계측 `temperature`, `charge`와 계측 실패 값은 NaN, `power_supply_health`는 UNKNOWN이다.
단순히 전압이 정상이라는 이유로 팩 건강 상태를 GOOD으로 단정하지 않는다.

## 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `port` | `/dev/SK120` | SK120 시리얼 포트 |
| `baudrate` | `115200` | 8N1 Modbus RTU |
| `slave_id` | `1` | Modbus 주소 |
| `voltage_set` | `25.2` | 출력 전압 [V], 최대 25.2 |
| `start_current` | `0.1` | 시작 전류 설정 [A] |
| `target_current` | `1.8` | 전류 설정 상한 [A] |
| `ramp_step` | `0.1` | 단계 증가량 [A] |
| `ramp_interval` | `5.0` | 단계 간 최소 간격 [s] |
| `settle_interval` | `5.0` | 설정 변경 후 안정화 대기 [s] |
| `current_offset` | `0.0` | 설정에 더할 보정 [A], 실측값에는 적용하지 않음 |
| `status_interval` | `2.0` | 배터리·출력 계측 주기 [s] |
| `poll_interval` | `5.0` | 미연결 상태 재시도 주기 [s] |
| `sensor_type` | `auto` | auto / ina226 / ina219 |
| `shunt_resistor` | `0.01` | 실제 션트 저항 [Ω] |

시작 ≤ 목표이며, 보정을 포함한 설정 전류는 최대 1.8A이다. NaN·음수·잘못된 주기는 시작 시 거부한다.
파라미터는 시작 시 읽으므로 변경하려면 노드를 재시작한다. 실행 중 설정 변경 요청은 거부한다.
GND 우회 배선 문제를 전류 보정만으로 해결했다고 판단하지 않는다.

## 실행 및 점검

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch battery battery.launch.py
# 실제 INA219로 교체한 경우:
# ros2 launch battery battery.launch.py sensor_type:=ina219

ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool '{data: true}'
ros2 topic echo /battery_state
ros2 topic echo /sk120/current_out
ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool '{data: false}'
```

같은 포트를 여는 배터리 노드는 하나만 실행한다. pyserial 독점 잠금으로 중복 소유를 방지한다.
Modbus 응답의 CRC뿐 아니라 슬레이브 주소·바이트 수·쓰기 전체 echo를 검증하며,
읽기·쓰기 제한 시간은 각각 0.25초이다. 장치가 실제로 응답하지 않으면 OFF를 보장할 수 없고,
SIGKILL·호스트 전원 상실 시에도 소프트웨어 종료 정리가 실행되지 않는다.

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select battery
# 운영 로봇과 분리한 DDS 도메인에서 모의장치 회귀 테스트
ROS_DOMAIN_ID=68 colcon test --packages-select battery --return-code-on-test-failure
colcon test-result --test-result-base build/battery --verbose
```

## SK120 레지스터

| 이름 | 주소 | 단위 |
| --- | --- | --- |
| V_SET | 0x0000 | 0.01V |
| I_SET | 0x0001 | 0.001A |
| VOUT | 0x0002 | 0.01V |
| IOUT | 0x0003 | 0.001A |
| POWER | 0x0004 | 0.01W |
| UIN | 0x0005 | 0.01V |
| ONOFF | 0x0012 | 0/1 |

기존 실물 시험과 이번 점검 기록은 `~/docking_diagnostics/2026-10-16/`에 보관한다.

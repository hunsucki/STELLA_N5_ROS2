# battery

STELLA N5 로봇의 배터리 모니터링 및 XY-SK120 무선 충전 제어 통합 패키지.

INA219(I2C) 센서로 배터리 상태를 항상 발행하면서, 로봇이 무선 충전 패드에 도킹하면 SK120 모듈을 자동 감지하여 전류 램프업 충전을 수행한다.

---

## 하드웨어 구성

```
[배터리 (6S LiPo, 13Ah)]
        │
        ├─── INA219 (I2C, 0x40) ─── Raspberry Pi (항상 연결)
        │     shunt: 10mΩ
        │
        └─── 부하 (모터, 센서 등)

[무선 충전 TX 코일]
        │
        └─── XY-SK120 충전 모듈 ─── USB-TTL (pl2303) ─── /dev/SK120
              Modbus RTU, 115200bps

[무선 충전 RX 코일] ─── [배터리]
```

### SK120 배선

| SK120 핀 | USB-TTL 핀 | 비고 |
|----------|-----------|------|
| TX       | RXD       | 교차 연결 |
| RX       | TXD       | 교차 연결 |
| GND      | GND       | ⚠ GND 루프 주의 (아래 참고) |

> **GND 루프 주의**: USB-TTL GND를 SK120 출력 GND와 공유하면 충전 전류 일부가 TTL선으로 우회하여 SK120이 실제보다 낮은 전류를 인식한다. 이 경우 `current_offset` 파라미터로 보정하거나 USB 아이솔레이터를 사용한다.

---

## 패키지 구조

```
battery/
├── battery/
│   ├── battery_status.py   # 통합 노드 (INA219 + SK120 제어)
│   ├── modbus_rtu.py       # Modbus RTU 통신 (pyserial 기반)
│   └── sk120_driver.py     # XY-SK120 레지스터 드라이버
├── launch/
│   └── battery.launch.py   # 독립 실행용 launch 파일
├── resource/
│   └── battery
├── package.xml
├── setup.py
└── README.md
```

> `robot.launch.py` (stella_bringup)에도 포함되어 있어 별도 실행 없이 로봇 전체 launch 시 자동으로 함께 시작된다.
> 자동 실행 여부는 `stella_bringup/param/robot_launch_param.yaml`의 `launch_battery`로 제어한다.

---

## 동작 흐름

### 상태 전이

```
[노드 시작]
    │
    ▼
 ┌──────────────────────────────────────────────────────────┐
 │  IDLE / 방전 중                                           │
 │  • INA219로 배터리 전압·전류 측정                         │
 │  • /battery_state 발행 (status=DISCHARGING)              │
 │  • 5초마다 SK120 Modbus 연결 시도                        │
 └───────────────────────┬──────────────────────────────────┘
                         │ SK120 응답 수신
                         ▼
 ┌──────────────────────────────────────────────────────────┐
 │  AVAILABLE / 도킹 감지됨                                  │
 │  • /sk120/available 발행 (BatteryState status=CHARGING)   │
 │  • INA219 계속 동작 중                                   │
 │  • 충전 명령 대기                                        │
 └───────────────────────┬──────────────────────────────────┘
                         │ /sk120/cmd_output: true
                         ▼
 ┌──────────────────────────────────────────────────────────┐
 │  CHARGING / 충전 중                                       │
 │  • SK120 출력 ON                                         │
 │  • 전류 램프업: start_current → target_current           │
 │  • /battery_state 발행 (status=CHARGING)                 │
 │    - voltage: (INA219 + SK120) / 2                      │
 │    - current: SK120 실측값 (양수)                        │
 │  • 충전 전류 감지 체크 (무선 코일 이탈 경고)             │
 └──────────────────────────────────────────────────────────┘
```

### 충전 종료 조건

| 조건 | 처리 | `_ramp_limit` |
|------|------|--------------|
| `/sk120/cmd_output: false` | 충전 중지 | **유지** |
| SK120 통신 3회 연속 실패 | 충전 중지 + 연결 해제 | **유지** |
| 도킹 해제 (SK120 전원 꺼짐) | 충전 중지 + 연결 해제 | **초기화** |

### 램프업 중단 보호 로직

무선 충전 특성상 초기 고전류 인가 시 코일이 커플링을 거부할 수 있다. 램프업 중 충전이 중단되면 그 시점의 전류를 `_ramp_limit`에 저장하고, 재충전 시 그 전류까지만 올린다. 로봇이 실제로 도킹 해제(SK120 전원 오프)된 경우에만 초기화된다.

```
예시:
  1회 충전: 0.7A → 1.2A 에서 중단
            _ramp_limit = 1.2A

  2회 충전: 0.7A → 1.2A (이번 세션 상한)
            _ramp_limit = 1.2A 유지

  도킹 해제 후 재도킹:
            _ramp_limit = None (초기화)

  3회 충전: 0.7A → 1.8A (전체 램프업 재시작)
```

---

## 토픽

### Subscribe

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/sk120/cmd_output` | `std_msgs/Bool` | `true`: 충전 시작 / `false`: 충전 중지 |

### Publish

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/battery_state` | `sensor_msgs/BatteryState` | 배터리 전체 상태 (항상 발행) |
| `/sk120/available` | `sensor_msgs/BatteryState` | SK120 연결 가능 여부 (도킹 여부). 연결 가능하면 `power_supply_status=CHARGING`, 불가능하면 `DISCHARGING` |
| `/sk120/output_on` | `std_msgs/Bool` | SK120 출력 ON/OFF 상태 |
| `/sk120/current_set` | `std_msgs/Float32` | 현재 설정 전류 \[A\] |
| `/sk120/current_out` | `std_msgs/Float32` | SK120 실측 충전 전류 \[A\] |
| `/sk120/voltage_out` | `std_msgs/Float32` | SK120 실측 출력 전압 \[V\] |

### `/battery_state` 필드 설명

| 필드 | 방전 중 | 충전 중 |
|------|---------|---------|
| `voltage` | INA219 측정값 | (INA219 + SK120) / 2 평균 |
| `current` | INA219 값 **음수** (방전=음수) | SK120 실측값 **양수** |
| `percentage` | SoC 0.0 ~ 1.0 | SoC 0.0 ~ 1.0 |
| `power_supply_status` | `DISCHARGING (2)` | `CHARGING (1)` |

> **전류 부호**: ROS `BatteryState` 관례에 따라 충전=양수, 방전=음수. INA219 shunt는 방전 방향이 양수로 측정되므로 방전 시 부호를 반전하여 발행한다.

### `/sk120/available` 필드 설명

Nav2 docking에서 충전 상태로 인식할 수 있도록 `sensor_msgs/BatteryState` 형식으로 발행한다. 실제 배터리 계측값은 `/battery_state`를 사용하고, `/sk120/available`의 나머지 값들은 더미값으로 채운다.

| SK120 상태 | `power_supply_status` | `present` | `percentage` |
|------------|------------------------|-----------|--------------|
| 연결 가능 | `CHARGING (1)` | `true` | `1.0` |
| 연결 불가 | `DISCHARGING (2)` | `false` | `0.0` |

---

## 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `port` | `/dev/SK120` | SK120 시리얼 포트 |
| `baudrate` | `115200` | 통신 속도 |
| `slave_id` | `1` | Modbus 슬레이브 ID |
| `voltage_set` | `25.2` V | SK120 출력 전압 설정 |
| `start_current` | `0.7` A | 램프업 시작 전류 (실제값) |
| `target_current` | `1.8` A | 램프업 목표 전류 (실제값) |
| `ramp_step` | `0.1` A | 램프업 1스텝 증가량 |
| `ramp_interval` | `5.0` s | 램프업 스텝 주기 |
| `current_offset` | `0.0` A | GND 루프 보정값 (SK120 설정값 = 실제값 + offset) |
| `status_interval` | `2.0` s | `/battery_state` 발행 주기 |
| `poll_interval` | `5.0` s | SK120 도킹 감지 폴링 주기 |

> `current_offset`: USB-TTL GND 연결 시 발생하는 GND 루프로 SK120이 실제보다 낮은 전류를 인식하는 경우 보정. 예) 실제 1.6A 목표인데 1.3A로 제한될 경우 `current_offset:=0.3` 설정.

---

## 실행

### robot.launch.py와 함께 (권장)

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch stella_bringup robot.launch.py
```

battery 노드가 자동으로 포함된다.

자동 실행을 끄려면 `stella_bringup/param/robot_launch_param.yaml`에서:

```yaml
launch_battery: false
```

### 단독 실행

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch battery battery.launch.py
```

파라미터 변경 시:

```bash
ros2 launch battery battery.launch.py \
  voltage_set:=25.2 \
  start_current:=0.7 \
  target_current:=1.8 \
  current_offset:=0.3
```

---

## 충전 제어

### 충전 시작

도킹 후 `/sk120/available`의 `power_supply_status: 1` (`CHARGING`) 확인 뒤:

```bash
ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool "data: true"
```

### 충전 중지

```bash
ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool "data: false"
```

### 상태 모니터링

```bash
# 배터리 전체 상태
ros2 topic echo /battery_state

# 충전 전류 실시간 확인
ros2 topic echo /sk120/current_out

# 도킹 여부 확인
ros2 topic echo /sk120/available
```

---

## 진단 스크립트

아래 스크립트는 현재 `battery` 패키지에 포함된 파일이 아니라, 별도로 둔 현장 진단용 스크립트를 사용하는 예시다.

SK120 연결 문제 발생 시 포트와 보드레이트를 자동 탐색:

```bash
python3 ~/sk120_diag.py
```

SK120 충전 시작 단계별 테스트:

```bash
python3 ~/sk120_charge_test.py
```

---

## 빌드

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select battery
source install/setup.bash
```

---

## SK120 Modbus 레지스터 참고

| 레지스터 | 주소 | 단위 | 설명 |
|---------|------|------|------|
| `REG_V_SET` | `0x0000` | 0.01 V | 출력 전압 설정 |
| `REG_I_SET` | `0x0001` | 0.001 A | 출력 전류 설정 |
| `REG_VOUT` | `0x0002` | 0.01 V | 출력 전압 실측 |
| `REG_IOUT` | `0x0003` | 0.001 A | 출력 전류 실측 |
| `REG_POWER` | `0x0004` | 0.01 W | 출력 전력 실측 |
| `REG_UIN` | `0x0005` | 0.01 V | 입력 전압 실측 |
| `REG_ONOFF` | `0x0012` | 0/1 | 출력 ON/OFF |

통신: Modbus RTU, 115200bps, 8N1, 슬레이브 ID 1

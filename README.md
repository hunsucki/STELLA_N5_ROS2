# STELLA N5 SBC Package

1. **Default Hardware**  
SBC : Raspberry Pi 5 8GB   
LIDAR : RPLIDAR C1

2. **Default Software**  
ROS version : ROS2 Jazzy   

3. **For More Details**  
For additional information, please refer to our [Menual](https://idearobot.gitbook.io/stella-n5)

## RealSense + AprilTag 안정화

`ros2 launch stella_bringup robot.launch.py`와 AprilTag 인식 노드를 함께 실행할 때
RealSense color image가 밀리거나 AprilTag에서 아래와 같은 동기화 경고가 반복되는 문제 발생

### 원인

- `stella_md_node`가 모터 상태를 매우 짧은 주기로 시리얼 폴링하면서 CPU 사용률 과다
- `/camera/camera/color/image_raw` 발행/수신률이 30Hz에서 약 10~14Hz 수준까지 떨어짐
- RealSense image publisher와 AprilTag subscriber의 QoS 조합 및 intra-process 설정으로 AprilTag노드가 이미지를 받지 못하는 상황 확인

### 수정 내용

- `stella_md_node`
  - 모터 모니터링 주기를 `monitoring_rate_hz` 파라미터로 조절하도록 변경
  - 현재 기본값은 원래 동작과 같은 `10Hz`로 유지함
  - 관련 파일:
    - `stella/stella_md/src/main.cpp`
    - `stella/stella_md/launch/stella_md_launch.py`

- `stella_ahrs_node`
  - AHRS read/publish 주기를 분리해 외부 publish 주기만 낮게 유지함
  - 기본값:
    - `read_rate_hz: 900`
    - `publish_rate_hz: 50`
  - 관련 파일:
    - `stella/stella_ahrs/mw/mw_ahrs.cpp`
    - `stella/stella_ahrs/launch/stella_ahrs_launch.py`

- RealSense QoS
  - AprilTag와 안정적으로 연결되도록 color image/camera_info QoS를 `SENSOR_DATA`로 설정
  - 관련 파일:
    - `stella_bringup/param/realsense_apriltag.yaml`
    - `stella_bringup/launch/robot.launch.py`

기대 상태:

- `/camera/camera/color/image_raw`가 약 25Hz 이상으로 유지
- `/odom`은 기본 10Hz로 발행
- `/camera/camera/color/image_raw`의 QoS가 `BEST_EFFORT`

## Nav2 odom/IMU yaw 안정화

Nav2 실행 중 제자리 회전하거나 방향을 크게 바꿀 때 RViz에서 LiDAR scan이 순간적으로 확 돌아가 보이는 문제가 있었음.
정지 상태에서는 `/imu/yaw`와 `/odom` yaw가 잘 맞았지만, 회전 중에는 odom yaw가 늦거나 튀는 증상이 의심됨.

### 기존 구조

```text
stella_ahrs_node
  publishes:
    /imu/data
    /imu/data_raw
    /imu/yaw

stella_md_node
  subscribes:
    /imu/yaw
    /cmd_vel

  publishes:
    /odom
    /tf
```

기존 `stella_md_node`는 timestamp가 없는 `std_msgs/Float64` 타입의 `/imu/yaw`를 받아 odom yaw로 사용했다.
따라서 이 yaw가 방금 측정된 값인지, serial buffer에 밀린 오래된 값인지 판단할 수 없었다.

또한 이전 설정은 아래처럼 동작했다.

```text
stella_ahrs_node read_rate_hz      = 200
stella_ahrs_node publish_rate_hz   = 50
stella_md_node monitoring_rate_hz  = 10
```

AHRS 센서가 더 빠르게 데이터를 내보내는 상황에서 read loop가 낮으면 serial backlog가 생길 수 있고,
그 경우 publish는 50Hz로 정상처럼 보여도 내용은 과거 yaw일 수 있다.
정지 상태에서는 문제가 잘 드러나지 않지만 회전 중에는 `odom -> base_footprint` yaw가 늦게 따라와 Nav2와 scan 표시가 흔들릴 수 있다.

### 현재 적용한 구조

기본 odom yaw 경로는 원래 구조와 같이 `/imu/yaw`를 사용한다.
`/imu/data` 기반 odom yaw와 yaw rate-limit 필터는 실험용 파라미터로 남겨두었지만 기본값은 꺼져 있다.
즉, 현재 기본 동작은 원래 잘 동작하던 구조를 최대한 유지하고,
주기 변경으로 생긴 serial backlog 가능성과 serial 예외로 노드가 죽는 문제만 보수적으로 줄이는 방향이다.

```text
stella_ahrs_node
  publishes:
    /imu/data        sensor_msgs/Imu, debug/optional용
    /imu/data_raw
    /imu/yaw         std_msgs/Float64, odom yaw 기본 소스

stella_md_node
  subscribes:
    /imu/yaw         odom yaw 기본 소스
    /imu/data        옵션이 켜진 경우에만 사용
    /cmd_vel

  publishes:
    /odom
    /tf
```

주요 변경점:

- AHRS read 기본값을 `200Hz -> 900Hz`로 올림
- AHRS publish 기본값은 `50Hz`로 유지
- `/imu/data.header.stamp`를 publish 시각이 아니라 실제 AHRS 패킷을 읽은 시각으로 기록
- `/imu/data.header.frame_id` 기본값을 `imu_link`로 설정
- `stella_md_node`의 기본 odom yaw 경로는 `/imu/yaw`로 유지
- serial read/write 예외가 발생해도 `stella_ahrs_node`, `stella_md_node`가 abort되지 않도록 방어
- `/imu/data` 기반 yaw, stale guard, yaw rate-limit 필터는 파라미터로 켤 수 있지만 기본값은 비활성
- `robot_launch_param.yaml`에 `launch_lidar2_filter`를 추가해 `/scan_filtered` 실행 여부를 별도로 제어 가능

현재 기본 파라미터:

```text
stella_bringup/param/robot_launch_param.yaml:
  launch_lidar2: true
  launch_lidar2_filter: false

stella_ahrs_node:
  read_rate_hz: 900
  publish_rate_hz: 50
  read_idle_sleep_us: 1000
  frame_id: imu_link
  parent_frame_id: base_link
  publish_tf: false

stella_md_node:
  monitoring_rate_hz: 10
  use_imu_data_orientation: false
  imu_timeout_sec: 0.0
  use_imu_yaw_filter: false
```

관련 파일:

- `stella/stella_ahrs/mw/mw_ahrs.cpp`
- `stella/stella_ahrs/include/mw/mw_ahrs.hpp`
- `stella/stella_ahrs/launch/stella_ahrs_launch.py`
- `stella/stella_md/src/main.cpp`
- `stella/stella_md/src/main.hpp`
- `stella/stella_md/launch/stella_md_launch.py`
- `stella/stella_md/CMakeLists.txt`
- `stella/stella_md/package.xml`
- `stella_bringup/launch/robot.launch.py`
- `stella_bringup/param/robot_launch_param.yaml`

### 실험 옵션

아래 옵션들은 문제 분리나 테스트용이다. 기본 운용에서는 꺼둔다.

`/imu/data` orientation을 odom yaw에 사용하려면:

```bash
ros2 param set /stella_md_node use_imu_data_orientation true
ros2 param set /stella_md_node imu_timeout_sec 0.25
```

AHRS yaw가 순간적으로 크게 튀는지 확인하고 rate-limit를 걸려면:

```bash
ros2 param set /stella_md_node use_imu_yaw_filter true
ros2 param set /stella_md_node imu_yaw_max_rate 2.0
ros2 param set /stella_md_node imu_yaw_filter_tau_sec 0.0
```

두 번째 라이다의 기존 각도 필터는 예전 하단 장착 방향 기준이므로, 후방 상단으로
이동한 현재 구성에서는 기본적으로 비활성화한다.

```yaml
launch_lidar2_filter: false
```

이 경우 `/scan_filtered`는 발행되지 않고 `/scan`, `/scan_2`만 남는다.

### 기대 상태

```bash
ros2 param get /stella_ahrs_node read_rate_hz
ros2 param get /stella_ahrs_node publish_rate_hz
ros2 param get /stella_md_node monitoring_rate_hz
ros2 param get /stella_md_node use_imu_data_orientation
ros2 param get /stella_md_node use_imu_yaw_filter
```

기대값:

```text
read_rate_hz: 900
publish_rate_hz: 50
monitoring_rate_hz: 10
use_imu_data_orientation: false
use_imu_yaw_filter: false
```

토픽 주기 확인:

```bash
ros2 topic hz /imu/data
ros2 topic hz /odom
```

기대 상태:

```text
/imu/data  약 50Hz
/odom      약 10Hz
```

정지 상태에서 `/imu/yaw`와 `/odom` orientation yaw가 거의 같아야 한다.
회전 중에도 두 yaw가 같은 방향으로 부드럽게 변해야 하며, 갑자기 180도 또는 360도 튀면 yaw wrap 처리나 IMU 데이터 지연을 다시 확인해야 한다.

회전 테스트 전후에는 반드시 정지 명령을 보내 모터 명령이 남지 않게 한다.

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

짧은 제자리 회전 테스트 예시:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.2}}"
```

테스트가 끝나면 `Ctrl-C` 후 다시 정지 명령을 보낸다.

## Battery 모니터링 및 무선 충전

`battery` 패키지는 STELLA N5의 배터리 상태 모니터링과 XY-SK120 무선 충전 제어를 담당함

- INA219(I2C, 기본 주소 `0x40`)로 배터리 전압, 전류, SoC를 측정
- XY-SK120 충전 모듈을 Modbus RTU로 제어
- `/battery_state`에 ROS 표준 `sensor_msgs/BatteryState` 발행
- `/sk120/available`로 SK120 연결 가능 여부, 즉 도킹/충전 가능 상태 발행
- `/sk120/cmd_output` 명령으로 충전 출력 ON/OFF 제어

`robot.launch.py`에 기본 포함되어 있어 일반 bringup 실행 시 자동으로 함께 시작됨

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch stella_bringup robot.launch.py
```

자동 실행 여부는 `stella_bringup/param/robot_launch_param.yaml`에서 제어함

```yaml
launch_battery: true
```

배터리 노드만 단독으로 실행하려면:

```bash
source ~/colcon_ws/install/setup.bash
ros2 launch battery battery.launch.py
```

주요 launch 파라미터:

```bash
ros2 launch battery battery.launch.py \
  port:=/dev/SK120 \
  baudrate:=115200 \
  slave_id:=1 \
  voltage_set:=25.2 \
  start_current:=0.7 \
  target_current:=1.8 \
  current_offset:=0.0
```

주요 토픽:

- `/battery_state`: 전체 배터리 상태. 충전 중 전류는 양수, 방전 중 전류는 음수로 발행
- `/sk120/available`: SK120 응답 가능 여부. 도킹 감지/충전 가능 상태 확인용
- `/sk120/cmd_output`: `std_msgs/Bool`, `true`면 충전 시작, `false`면 충전 중지
- `/sk120/output_on`: SK120 출력 ON/OFF 상태
- `/sk120/current_set`: 현재 설정 전류
- `/sk120/current_out`: SK120 실측 출력 전류
- `/sk120/voltage_out`: SK120 실측 출력 전압

충전 시작/중지 예시:

```bash
ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool "data: true"
ros2 topic pub --once /sk120/cmd_output std_msgs/msg/Bool "data: false"
```

상태 확인:

```bash
ros2 topic echo /battery_state
ros2 topic echo /sk120/available
ros2 topic echo /sk120/current_out
```

필요 패키지:

```bash
sudo apt install python3-serial python3-smbus2
```

참고: SK120 USB-TTL 기본 포트는 udev 고정 장치명 `/dev/SK120`로 설정되어 있음. 장비에서 포트가 달라지면
`stella_bringup/stella.rules`의 SK120 규칙 또는 `port:=...` launch 파라미터를 현장 설정에 맞게 변경

## 통합 도킹 실행

기존 도킹 절차는 아래 명령들을 순서대로 실행해야 한다

```bash
ros2 launch ~/camera_36h11.launch.yml
python3 apriltag_bridge.py
ros2 run opennav_docking opennav_docking --ros-args --params-file ~/docking_1.yaml
ros2 lifecycle set /docking_server configure
ros2 lifecycle set /docking_server activate
python3 dock_turn_backup.py
```

이를 `docking` 패키지로 통합, `robot.launch.py` bringup이 이미 실행 중인 상태에서
아래 명령 하나로 AprilTag, AprilTag bridge, OpenNav docking server를 실행하고,
`docking_server`를 configure/activate 한 뒤 도킹, 180도 회전, 후진을 수행
시퀀스가 끝나면 이 명령이 띄운 하위 프로세스들은 종료됨

```bash
ros2 run docking dock_turn_backup
```

패키지에 포함된 주요 파일:

- `docking/launch/apriltag_36h11.launch.py`: AprilTag 인식 노드 실행
- `docking/docking/apriltag_bridge.py`: AprilTag TF를 `detected_dock_pose`로 변환
- `docking/docking/dock_turn_backup.py`: 전체 도킹 시퀀스 조율
- `docking/docking/stack_manager.py`: AprilTag, bridge, docking server 실행/종료
- `docking/docking/lifecycle.py`: docking server lifecycle configure/activate
- `docking/docking/motion.py`: odom 기반 180도 회전, 후진, 정지 명령
- `docking/docking/docking_lidar.py`: `/scan_2` freshness, frame, TF를 공통 검증
- `docking/docking/lidar_geometry.py`: LaserScan을 `base_link` 좌표로 투영하고 뒤 간격 계산
- `docking/docking/lidar_alignment.py`: `/scan_2` 평면 검출 및 yaw 미세 보정
- `docking/docking/charging.py`: 충전기 접촉, 충전 시작, 안정 전류 확인
- `docking/docking/safety.py`: 종료 코드와 단일 인스턴스 lock
- `docking/config/docking.yaml`: OpenNav docking server 파라미터
- `docking/config/tags_36h11.yaml`: AprilTag 인식 파라미터

### 개발 테스트 모드와 운영 모드

현재 소스의 `development_test_mode` 기본값은 **`true`**이다.
따라서 별도 파라미터 없이 실행하면 AprilTag 접근, 회전, LiDAR 정렬과 후진을 완료한 뒤
충전 접촉 및 전류를 확인하지 않고 종료 코드 `0`을 반환한다.

```bash
ros2 run docking dock_turn_backup
```

테스트 모드를 끄고 운영 모드로 실행하면 후진 완료 후 다음 순서까지 성공해야
종료 코드 `0`을 반환한다. 테스트 모드를 끄는 명령은 README 맨 아래에 정리되어 있다.

1. `/sk120/available`에서 SK120 접촉 상태 확인
2. `/sk120/cmd_output`에 `true`를 발행해 충전 시작
3. `/battery_state`가 `CHARGING`이고 전류가 기본 `0.05A` 이상인지 확인
4. 위 상태가 기본 `3초` 동안 안정적으로 유지되는지 확인

충전 접촉 또는 전류 확인이 기본 `18초` 안에 완료되지 않으면 로봇을 정지하고 종료 코드 `6`으로 끝난다.
이미 실제 충전 전류가 흐르는 상태에서 명령을 다시 실행하면 로봇을 움직이지 않고 성공 처리한다.

### drive_manager SSH 실행 계약

Jetson의 `drive_manager`는 stdout이나 ROS 토픽을 성공 신호로 해석하지 않고,
SSH로 실행한 `dock_turn_backup` 프로세스의 종료 코드만 사용한다. 이 패키지는 다음 계약을 적용한다.

- 프로세스는 전체 도킹이 끝날 때까지 포그라운드에서 실행됨
- 전체 내부 제한 시간은 기본 `100초`로 Jetson의 `120초` 제한보다 짧음
- 정상, 실패, 예외, timeout, 신호 종료에서 `/cmd_vel` 0을 여러 번 발행함
- 충전 시작 명령을 보낸 뒤 성공을 확인하지 못한 종료 경로에서는 `/sk120/cmd_output: false`도 반복 발행함
- SIGINT, SIGTERM, SIGHUP을 중단 요청으로 처리하고 진행 중인 action을 취소함
- Linux parent-death signal을 설정해 SSH 명령의 직접 부모 프로세스가 종료되면 SIGTERM 정리 경로로 진입함
- 실행 중 시작한 AprilTag, bridge, docking server의 프로세스 그룹을 SIGINT, SIGTERM, SIGKILL 순서로 정리함
- `/tmp/stella_dock_turn_backup.lock`의 advisory lock으로 중복 실행을 거부함
- lock은 프로세스 종료 시 커널이 자동 해제하므로 stale lock 파일이 남아도 재실행 가능함
- `DOCKING_LOCK_FILE` 환경 변수로 lock 경로를 변경할 수 있음

종료 코드:

| 코드 | 의미 |
| ---: | --- |
| `0` | 성공. 개발 모드로 실행한 경우 거리 기반 후진 완료, 운영 모드에서는 안정 충전 확인 완료 |
| `1` | 처리되지 않은 내부 오류 또는 cleanup 오류 |
| `2` | 잘못된 파라미터 또는 중복 실행 거부 |
| `3` | 필수 센서, TF, odom, AprilTag pose, docking server 사용 불가 |
| `4` | 도킹 접근, 회전, LiDAR 정렬 또는 후진 실패 |
| `5` | 전체 `100초` timeout |
| `6` | 접촉 또는 충전 전류 확인 실패 |
| `129` | SIGHUP 중단 |
| `130` | SIGINT 중단 |
| `143` | SIGTERM 중단 |

주요 안전/운영 파라미터:

| 파라미터 | 현재 기본값 | 설명 |
| --- | ---: | --- |
| `development_test_mode` | `true` | `true`이면 후진 완료 후 충전 확인 없이 성공(벤치 전용) |
| `total_timeout_sec` | `100.0` | 전체 도킹 내부 제한 시간 |
| `charger_wait_timeout_sec` | `18.0` | 접촉 및 충전 확인 제한 시간 |
| `charging_stable_sec` | `3.0` | 충전 상태 최소 유지 시간 |
| `charging_min_current` | `0.05` | 성공 판정 최소 충전 전류(A) |
| `existing_charging_wait_sec` | `2.5` | 시작 시 이미 충전 중인지 확인하는 시간 |
| `motion_timeout_sec` | `45.0` | 개별 회전/후진 제한 시간 |
| `docking_lidar_scan_max_age_sec` | `0.30` | 이 시간보다 오래된 scan이면 즉시 정지/실패 |
| `backup_max_travel` | `0.60` | LiDAR 후진의 독립적인 odom 이동 한계(m) |
| `max_staging_time` | `40.0` | DockRobot staging 제한 시간 |
| `dock_pose_wait_timeout_sec` | `10.0` | `detected_dock_pose` 입력 대기 시간 |

베이스 드라이버 `stella_md_node`에도 독립적인 `/cmd_vel` watchdog을 추가했다.
`cmd_vel_timeout_sec`의 현재 기본값은 `0.5초`이며, 마지막 속도 명령 이후 이 시간이 지나면
모터 드라이버에 직접 0속도를 보낸다. `0.0`으로 설정하면 watchdog이 비활성화되므로 운영에서는 권장하지 않는다.
parent-death signal은 직접 부모 종료를 감지하는 Linux 보호 장치이며 네트워크 자체의 heartbeat는 아니다.
SSH 서버와 원격 부모가 비정상적으로 계속 살아 있는 특수 장애까지 포함하려면 향후
`drive_manager`와 로봇 측 supervisor 사이에 별도 heartbeat 또는 stop service를 추가해야 한다.

AprilTag 입력은 RealSense의 `BEST_EFFORT` 영상 QoS와 맞도록
`docking/config/tags_36h11.yaml`의 `qos_profile`을 `sensor_data`로 사용한다.
`dock_turn_backup`은 `detected_dock_pose`를 먼저 확인한 뒤 DockRobot action을 요청하므로,
태그 ID `0`이 보이지 않거나 QoS/TF가 끊긴 상태에서는 움직임 단계로 진입하지 않는다.

`dock_turn_backup`은 `/cmd_vel`을 직접 발행하므로 `drive_manager`가 Nav2 navigation을
PAUSE한 뒤에만 실행해야 한다. 도킹 중에는 teleop이나 별도 `/cmd_vel` publisher를 실행하지 않는다.
현재 `dock_turn_backup` 안에는 다른 `/cmd_vel` publisher를 자동 차단하는 기능이 없으므로,
실행 전 아래 명령으로 publisher를 확인하고 teleop·navigation 명령을 운영 계층에서 중지해야 한다.
현재 원격 명령에는 Nav2 PAUSE를 로봇 측에서 독립 확인할 인터페이스가 없으므로 이 선행 조건은
Jetson의 `drive_manager`가 계속 보장해야 한다. 현장 점검에는 다음 명령을 사용한다.

```bash
ros2 topic info /cmd_vel --verbose
```

변경 후 빌드와 테스트:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  docking stella_md stella_description stella_bringup sllidar_ros2 sllidar2_ros2
source install/setup.bash
colcon test --packages-select docking stella_md
colcon test-result --verbose
```

실행 전 `robot.launch.py`에서 RealSense, `/scan_2`, odom/TF, battery, motor driver가 먼저 올라와 있어야 한다.

```bash
# 터미널 1
ros2 launch stella_bringup robot.launch.py

# 터미널 2: 기본 개발 테스트 모드(충전 확인 생략)
ros2 run docking dock_turn_backup
```

이미 AprilTag나 docking server를 따로 실행해 둔 상태에서 테스트하려면 필요한 자동 실행만 끌 수 있는 기능 포함

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p start_apriltag:=false \
  -p start_bridge:=false \
  -p start_docking_server:=false
```

이 경우에도 `/docking_server` lifecycle configure/activate는 기본으로 수행함. lifecycle까지 직접 관리하고 싶을 경우
`-p activate_docking_server:=false`를 추가

### LiDAR 평면 기반 회전 보정

`dock_turn_backup`은 AprilTag 도킹 접근 후 odom 기반 180도 회전을 먼저 수행하고,
후진하기 전에 `/scan_2`에서 벽/태그 평면을 찾아 yaw를 미세 보정함

카메라 접근 완료 반경은 `docking_threshold: 0.15`를 유지한다. 이 반경을 수 cm로
줄이면 태그 앞의 한 점에 수렴하려고 전후진을 반복하거나 도킹 포드에 너무 가까이
붙어서 180도 회전 공간을 잃을 수 있다. 추가 정밀도는 Nav2 접근이 끝난 뒤 고정된
`/dock_pose`를 사용하는 저속 태그 pose 보정 단계에서 확보한다. 실험했던 좌우 LiDAR
가이드 중앙 추종은 현재 운영 경로에서 사용하지 않는다.

180도 회전은 매 odom 변화량을 누적하지 않고, 회전 직전 절대 odom yaw에 180도를
더한 yaw를 목표로 사용한다. 목표 40도 전부터 감속하며 odom 오차가 1도 이내이고
`/imu/data.angular_velocity.z`가 0.5도/초 이하인 상태가 연속 5회 확인된 뒤 종료하므로
정지 관성에 의한 초과 회전을 줄인다. `/odom` yaw에는 이미 `/imu/data` orientation이
융합되어 있으므로 raw IMU orientation을 다시 합치지 않고 독립적인 각속도를 사용한다.
AHRS의 `/imu/data`는 RELIABLE QoS로 구독한다. 회전 도중 raw IMU 전달이 잠시 끊겨도
fresh한 odom pose로 절대 목표각 추종을 계속하고, 정지 판정의 각속도만 odom에 포함된
encoder 기반 `twist.angular.z`로 임시 전환한다. IMU가 복구되면 자동으로 IMU 각속도를
다시 사용한다. 회전 시작 시점부터 IMU가 잠시 unavailable이어도 odom이 fresh하면 같은
fallback으로 시작한다. 따라서 0.25초의 일시적인 IMU 공백만으로 회전을 중단하지 않지만,
odom 자체가 오래되면 기존과 같이 즉시 정지하고 실패한다.

현재 도킹 시나리오에서는 180도 회전이 끝난 뒤 로봇이 후진으로 벽/태그 쪽에 접근하므로,
보정에 사용할 평면은 로봇 기준 후방에 위치함. 따라서 실제 운용에서는
`base_link` 기준 후방인 `lidar_align_sector_center_base:=3.14159`를 사용함.

기본 동작:

- `/scan_2`의 `frame_id`가 `base_scan2`인지 확인하고 URDF의 `base_scan2 -> base_link` TF를 조회
- LaserScan 점을 `base_link` 좌표로 변환한 뒤 `lidar_align_sector_center_base` 중심 영역만 사용
- 현재 권장값은 `base_link` 기준 후방 `pi` 방향의 60도 영역(180도 +/-30도)
- 180도 회전 후 기대되는 후면 패널 방향에서 15도 이상 벗어난 RANSAC 선분은
  긴 측면 프레임이어도 제외하고, 남은 후보의 지지점 수와 실제 길이를 함께 평가함
- 평면 접선이 로봇의 좌우축(`pi/2`)과 평행해지도록 `/cmd_vel.angular.z`로 저속 보정
- 서로 다른 최신 scan 5개에서 오차가 1도 이내이고 IMU 또는 wheel odom이 정지 상태이면 후진 단계로 넘어감

주요 튜닝 파라미터:

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_lidar_alignment:=true \
  -p docking_lidar_topic:=/scan_2 \
  -p docking_lidar_frame:=base_scan2 \
  -p lidar_align_sector_center_base:=3.14159 \
  -p lidar_align_sector_width:=1.0472 \
  -p lidar_align_tolerance:=0.01745 \
  -p lidar_align_angular_speed:=0.06 \
  -p lidar_align_timeout_sec:=12.0
```

정렬 방향이 반대로 보이면 `-p lidar_align_kp:=-0.8`로 부호를 바꿔 테스트할 수 있음
LiDAR 평면이 잘 안 잡히면 `lidar_align_sector_width`, `lidar_align_max_range`,
`lidar_align_ransac_threshold`를 현장 구조에 맞춰 조정
`lidar_align_sector_center_base`는 센서 로컬 각도가 아니라 `base_link` 기준 각도임.

평면 정렬 RANSAC은 후방 60도(180도 +/-30도)를 보고, 후진 충돌 감시도
위해 후방 60도를 유지한다. 최종 거리 판정은 중앙 후방 20도만 사용한다.

후진 중에도 같은 RANSAC 후면 평면의 법선 오차를 추적하지만, 좌우 프로파일 중심
추종은 현재 비활성이다. `use_lidar_guide_centering` 기본값은 `false`이고 메인 노드도
guide estimator를 후진 제어기에 연결하지 않는다. 관련 추정 코드와 테스트는 실험
자료로 남아 있으나, 플래그만 켜서 운영 기능으로 사용할 수는 없다.

참고: STELLA N5 URDF에서 `base_scan2`는 `base_link` 대비 yaw가 `pi`라서 `/scan_2`의 0도 방향이 로봇 후방을 향함.
현재 RealSense 포함 URDF의 장착 위치는 `xyz="-0.166 0.0 0.223"`,
`rpy="0.0 0.0 3.1415"`이며 도킹 기본값은 이 2번 LiDAR를 사용함.

후진 단계는 기본적으로 odom 누적 이동거리 대신 LiDAR 후방 거리로 종료함.
LaserScan 끝점을 TF로 `base_link`에 투영한 뒤 차체 collision box의 뒤 끝
`backup_rear_reference_x:=-0.2295`와 비교한다. 따라서 센서 x 위치나 yaw를 코드에
offset으로 중복 저장하지 않는다. 코드 기본값은 뒤 기준면이 벽/태그에서 약 `0.01m`
남는 것이지만, 현재 `docking.yaml`은 반복 시험 결과를 반영해 목표를 `0.020m`,
허용 오차를 `0.005m`로 덮어쓴다.

현재 성공 조건은 중앙 20도 영역에서 차체 후단 간격 `0.015~0.025m`를 만족하는
인접 beam 군집이 서로 다른 scan 3개에서 확인되는 것이다. 보호 fan의 최솟값이 먼저
`0.025m` 이하가 되면 안전을 위해 정지하지만, 중앙 군집이 성공 조건을 만족하지 않으면
1초 뒤 `blocked outside the completion condition`으로 실패한다. 별도의
`contact_envelope` 우회 성공 판정은 현재 코드에 없다.

큰 회전 직후에는 절대 IMU yaw가 정착하면서 로봇이 멈춰 있어도 융합 odom yaw가
변할 수 있다. 따라서 후진 중 5도 이탈 안전 판정과 LiDAR 상실 시 방향 유지는
자력계 orientation이 아닌 `/imu/data.angular_velocity.z` 적분각을 사용한다.
도킹 벽면이 충분히 보이는 동안에는 LiDAR 평면 오차를 연속적으로 우선 사용하고,
벽면이 너무 가까워져 유효한 평면을 얻지 못할 때 마지막 LiDAR 정렬 시점의 gyro
방향으로 자동 전환한다. raw IMU가 일시적으로 끊긴 경우에는 encoder-only wheel yaw와
odom의 wheel 각속도로 자동 전환한다. 5도 dead-reckoning 제한은 RANSAC 평면까지
사라졌을 때만 적용하며, 정상적인 RANSAC 벽면을 따라 수행하는 의도적인 각도 보정은
실패로 처리하지 않는다. 후진 중에도 ROS `angular.z`와 실측 회전 응답의 부호는 같으므로
`backup_reverse_angular_command_sign:=1.0`을 적용한다. IMU yaw rate는 회전 감쇠에도
사용한다. 1도 이내의 작은 오차는 허용하고 각속도를 최대 `0.004rad/s`, 명령 변화율을
`0.010rad/s^2`로 제한한다. 후진 중 각도 보정을 위해 선속도를 멈추지 않으며, RANSAC이
끊기거나 거부되면 직전 각속도를 유지하지 않고 `angular.z=0`으로 직선 후진한다.
보호 간격이 0.10m 이하가 되면 RANSAC 각도 보정 자체를 끈다. 로그에는
`wheel_yaw_drift`, 비교용 `odom_yaw_drift`, `imu_yaw_from_start`,
`imu_heading_drift`, `stationary_source`, 실제 `heading_cmd`가 각각 출력된다.

새 위치에서 목표 시 센서 정면 거리는 약 `0.0735m`이므로 드라이버의 `range_min=0.05m`와
코드의 `backup_lidar_min_range=0.05m`를 함께 사용한다. 시작할 때 목표 거리가 실제
LaserScan 유효 범위 안인지 검사하며, 최소 3도에 걸친 인접 beam 5개와 서로 다른 최신 scan 3개가
동시에 조건을 만족해야 완료로 인정한다. 단일 짧은 노이즈는 정지는 시키지만 성공으로
처리하지 않는다. 후방 라이다 로컬 0도(로봇 기준 후방 180도)를 중심으로 좌우 30도,
총 60도 fan에서 장애물을 별도 감시한다. 기존 좌우 75도 영역은 현장 스테이션 측면
프레임을 직선 후진 경로의 장애물로 오인했기 때문에 제외했다. 보호 거리에서 1초 이상
막힌 상태가 계속되면 45초 동안 정지해 기다리지 않고 진단값과 함께 실패한다. scan이
`0.30초` 이상 끊기거나 odom 후진량이 `0.60m`, yaw 편차가 5도에 도달하면 실패한다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_lidar_backup:=true \
  -p docking_lidar_topic:=/scan_2 \
  -p docking_lidar_frame:=base_scan2 \
  -p backup_lidar_sector_center_base:=3.14159 \
  -p backup_lidar_sector_width:=0.3491 \
  -p backup_rear_reference_x:=-0.2295 \
  -p backup_lidar_min_range:=0.05 \
  -p backup_target_rear_clearance:=0.02
```

### 도킹 실행 전 TF 확인

OpenNav docking server는 `/odom` 토픽뿐 아니라 `odom -> base_link` TF transform이 필요함
`dock_turn_backup`은 도킹 goal을 보내기 전에 `/odom` 메시지와 `odom -> base_link` TF가 준비될 때까지 대기
또한 `/scan_2`, `frame_id=base_scan2`, `base_link <- base_scan2` TF와 근거리 측정 가능성을
확인한 뒤에만 이동을 시작함. 동일하거나 오래된 `header.stamp`를 반복 발행한 scan도
새 측정으로 인정하지 않음.
또한 `docking_server` activate 직후 내부 TF buffer가 `/tf`를 받을 수 있도록 기본 2초 대기(파라미터 설정가능)

아래와 같은 에러가 보이면 `/odom` 토픽은 있어도 docking server가 아직 `odom` frame을 받기 전에 goal이 들어간 경우임

```text
Transform error: "odom" passed to lookupTransform argument target_frame does not exist.
```

도킹 중 아래처럼 future extrapolation 에러가 나면 `/tf` 발행 주기와 docking controller 조회 시점이 너무 빠듯한 경우임

```text
Lookup would require extrapolation into the future
```

`docking/config/docking.yaml`의 `controller.transform_tolerance`는 `0.3`초로 설정했고,
`apriltag_bridge`는 `detected_dock_pose` stamp를 기본 `0.1`초 과거로 발행해 TF 시간 차이를 흡수함

확인 명령:

```bash
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_scan2
ros2 topic echo /scan_2 --once --field header.frame_id
```

`tf2_echo`가 처음 1초 정도 대기 메시지를 출력한 뒤 transform을 계속 출력하면 정상
계속 대기 상태라면 bringup의 `stella_md_node`, `robot_state_publisher`, `/tf`, `/tf_static` 상태 확인
필요하면 `-p docking_server_tf_warmup_sec:=3.0`처럼 대기 시간을 늘려 테스트

### 테스트 모드 끄기

실제 충전 접촉과 충전 전류까지 확인하는 운영 모드로 실행하려면
`development_test_mode`를 `false`로 지정한다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p development_test_mode:=false
```

## Update 0825 - wheel odometry 분리 및 IMU/엔코더 주기 개선

> 이 섹션은 2026-08-25 기준의 현재 구현과 실측 결과를 기록한다. README 위쪽에 남아 있는
> `stella_md` 10 Hz, AHRS 50 Hz 및 `/imu/yaw` 기반 odom 설명은 이전 작업 이력이며,
> 현재 기본 설정은 아래 내용과 `stella_bringup/param/robot_launch_param.yaml`을 기준으로 한다.

### 변경 목적과 현재 기본 구조

기존에는 `stella_md_node`가 모터 제어, 엔코더 읽기, IMU yaw 구독 및 `/odom` 계산을
모두 담당했다. 기존 계산 코드는 삭제하지 않고 `enable_legacy_odom` 파라미터로
비활성화했으며, 새로운 `wheel_odometry` 패키지로 odom 계산을 분리했다.

```text
MW-MDC24D100D-v2
  -> stella_md_node
  -> /wheel/encoders (sensor_msgs/JointState, timestamp 포함)
                                      \
                                       -> wheel_odometry -> /odom
                                      /                  -> odom -> base_footprint TF
MW-AHRS X1 -> stella_ahrs_node -> /imu/data (timestamp 포함)
                                  /imu/data_raw
                                  /imu/mag
                                  /imu/yaw (기존 호환용, Float64라 header 없음)
```

현재 `robot.launch.py`의 기본값은 다음과 같다.

- `launch_wheel_odometry: true`
- `enable_legacy_odom: false`
- 엔코더 polling 목표: `30 Hz`
- AHRS sensor sync 요청: `5 ms` (`200 Hz` 주기 요청)
- ROS IMU topic 발행 상한: `100 Hz`
- AHRS serial read: `0` (인위적인 rate 제한 없이 수신 버퍼를 계속 비움)
- AHRS decoded-packet yield: `1250 us` (reader/publisher thread starvation 방지)
- motor driver CPU affinity: core `2`
- AHRS driver CPU affinity: core `3`

`stella_md_node`의 모터 제어 및 `/cmd_vel` 구독 기능은 그대로 유지된다. 분리된 것은
odom 계산과 발행 책임이며, `stella_md_node`는 항상 timestamp가 있는
`/wheel/encoders`를 발행한다.

### 적용한 하드웨어 값과 이론 해상도

확인한 하드웨어와 현재 계산값은 다음과 같다.

- 모터: `MD36NP51-24V`, 감속비 `51:1`
- 엔코더: 광전식 `500 PPR`, 4체배 시 모터축 `2,000 count/rev`
- 감속기 출력축 한 바퀴: `2,000 x 51 = 102,000 count/rev`
- 모터 드라이버: `MW-MDC24D100D-v2`, serial `115200 bps`
- 드라이버에서 확인한 양 채널 encoder PPR: 각각 `2,000`
- 현재 wheel radius: `0.0875 m`
- 현재 wheel separation: `0.36 m`
- 현재 반지름 기준 바퀴 둘레: 약 `0.549779 m`
- 엔코더 1 count당 이론 이동거리: 약 `0.000005390 m` (`5.39 um`)

바퀴가 명목상 직경 `180 mm`여도 하중, 타이어 눌림, 바닥 재질 때문에 유효 반지름은
정확히 `0.09 m`가 아닐 수 있다. 요청에 따라 기존 값 `0.0875 m`를 그대로 유지했다.
만약 실제 유효 반지름이 정확히 `0.09 m`라면 현재 odom 직선거리는 약 `2.78%` 작게
계산될 수 있으나, 이는 실제 주행 거리 측정 전의 단순 기하학적 비교일 뿐이다.

### `stella_md` 변경 사항

- 기존 `/odom` 계산 코드는 삭제하지 않고 `enable_legacy_odom` 기본값을 `false`로 변경
- `encoder_poll_rate_hz` 파라미터 추가, 기본값 `30`
- 예전 `monitoring_rate_hz`는 호환성을 위해 남겨 두었으며 `0`보다 크면
  `encoder_poll_rate_hz`를 대신하는 deprecated 파라미터로 동작
- 좌/우 절대 엔코더 위치와 계산된 속도를 `/wheel/encoders`로 발행
- 두 모터 채널을 순차 질의하므로, timestamp는 첫 질의 시작 시각과 두 번째 질의 종료
  시각의 중간값으로 기록하여 좌/우 측정 시간 편차를 줄임
- legacy odom을 실행 중 켜거나 끌 수 있으며, 켤 때 현재 encoder 위치를 새 baseline으로
  잡아 갑작스러운 pose 점프를 방지
- legacy odom을 끄면 `/odom` publisher와 TF broadcaster 자체를 해제

주의: legacy odom의 twist에는 원래 코드의 commanded velocity 사용 방식이 남아 있다.
새 `wheel_odometry`는 실제 encoder 변화량과 timestamp 차이로 twist를 계산한다.

### 새 `wheel_odometry` 패키지

`wheel_odometry`는 `/wheel/encoders`와 timestamp가 있는 `/imu/data` orientation을 받아
`/odom`과 `odom -> base_footprint` TF를 발행한다.

계산 순서는 다음과 같다.

1. 좌/우 바퀴 회전 변화량에 반지름을 곱해 양쪽 이동거리를 계산한다.
2. 양쪽 이동거리의 평균으로 중심 이동거리를 구한다.
3. `(right_distance - left_distance) / wheel_separation`으로 wheel yaw 변화를 예측한다.
4. 엔코더 timestamp를 앞뒤로 감싸는 IMU yaw가 있으면 최단 각도 방향으로 보간하고,
   그렇지 않으면 timeout 이내의 최신 과거 샘플만 사용한다.
5. 새롭고 유효한 IMU 측정값일 때만 complementary correction으로 누적 yaw drift를
   보정한다.
6. 이전 yaw와 새 yaw의 중간 방향으로 `x`, `y`를 적분한다.
7. 실제 중심 이동거리와 휠 차동 yaw 변화량을 encoder timestamp 간격으로 나누어
   twist를 계산한다. IMU heading 보정량은 `twist.angular.z`에 포함하지 않는다.

기본 yaw 설정은 다음과 같다.

- `use_imu_orientation: true`
- `relative_imu_yaw: false` - 기존 동작처럼 IMU의 절대 orientation으로 시작
- `imu_timeout_sec: 0.25`
- `imu_history_duration_sec: 1.0`
- `imu_correction_time_constant_sec: 0.5`
- `max_imu_correction_rate_rad_s: 1.0`
- `max_wheel_speed_m_s: 2.0`

IMU가 잠시 늦거나 stale이면 odom을 중단하지 않고 wheel yaw만으로 계속 적분한다.
`imu_correction_time_constant_sec: 0.0`이면 매 encoder sample에서 IMU yaw를 즉시
적용하는 동작이 된다.

좌우 휠 중 하나라도 `max_wheel_speed_m_s`를 넘는 변화가 검출되면 counter reset,
rollover 또는 손상된 serial sample로 간주해 해당 변화를 odom에 반영하지 않고 현재
엔코더 값을 새 baseline으로 설정한다. 현재 기본값 `2.0 m/s`는 모터 정격 출력축
속도와 현재 휠 반지름으로 계산한 약 `1.12 m/s`보다 여유 있게 설정한 안전 한계다.

비교용 `/wheel_odometry/yaw_diagnostics`는
`geometry_msgs/Vector3Stamped`이며 다음 값을 담는다.

- `vector.x`: wheel encoder만 적분한 yaw
- `vector.y`: 같은 시점에 사용할 수 있는 최신 IMU yaw, 없으면 `NaN`
- `vector.z`: 최종 융합 yaw

기본 기구/융합/covariance 파라미터는
`wheel_odometry/config/wheel_odometry.yaml`에서 관리한다. 현재 covariance는 실측으로
추정한 값이 아니라 초기 운용을 위한 값이므로 향후 rosbag 기반 보정이 필요하다.

### IMU timestamp와 serial 처리 개선

공식 기본 드라이버의 `/imu/data`에도 `header.stamp`는 있었지만, 센서 packet을 받은
시각이 아니라 ROS publish loop가 실행된 시각을 넣었다. 또한 센서 sync보다 publish
loop가 훨씬 빠르면 같은 측정값이 새 timestamp로 반복 발행될 수 있었다.

현재 구현은 다음처럼 변경했다.

- ACC/GYRO/orientation/MAG packet을 실제로 읽은 시점에 해당 메시지 timestamp 갱신
- `publish_only_on_new_data: true`일 때 이전과 같은 timestamp의 측정값은 재발행하지 않음
- 새 orientation이 발행될 때 `/imu/data`와 호환용 `/imu/yaw`를 함께 발행
- `wheel_odometry`는 header가 없는 `/imu/yaw` 대신 timestamp가 있는 `/imu/data` 사용
- `sync_period_ms`를 파라미터화하고 설정 후 장치에서 값을 다시 읽어 로그로 확인
- `read_rate_hz: 0`이면 별도의 packet/s 상한은 두지 않되, reader thread가 publisher의
  mutex를 독점하지 않도록 정상 packet 뒤에 `read_success_sleep_us: 1250`만큼 양보
- 수신 packet이 없을 때는 `read_idle_sleep_us`만큼 sleep
- 종료/재시작 때 streaming 중지 명령을 응답 대기 없이 보낸 후 serial input을 flush하여
  남아 있던 sync packet이 다음 연결의 설정 응답으로 오인되는 문제 완화
- AHRS YAML이 실제 노드명 `stella_ahrs_node`에 적용되도록 수정하고 config 설치 추가

`/imu/yaw`는 `std_msgs/Float64`이므로 메시지 형식상 timestamp를 넣을 수 없다.
새 odom이나 sensor fusion에는 `/imu/data`를 사용해야 한다.

### 센서 주기 실측 결과

측정은 로봇에 실제 연결된 모터 드라이버와 AHRS로 수행했으며, 안전을 위해
`/cmd_vel`은 한 번도 발행하지 않고 정지 상태에서 확인했다.

모터 드라이버는 좌/우 encoder를 각각 요청/응답하는 blocking serial 방식이다.
공식 초기 코드의 1 ms timer는 1,000 Hz 목표일 뿐 실제 1,000회의 좌/우 측정을
완료할 수 없다. 이 로봇에서 paired encoder query의 실측 포화점은 약 `31.25 Hz`였다.

| encoder 목표 | 실측 | `stella_md_node` CPU 참고값 |
|---:|---:|---:|
| 10 Hz | 약 10.05 Hz | 약 30% |
| 20 Hz | 약 20.07 Hz | 약 39.5% |
| 30 Hz | 약 30 Hz | 약 68.6% |
| 50 Hz | 약 31.25 Hz로 포화 | 약 85.9% |

따라서 현재 `30 Hz`는 이 드라이버/라이브러리 조합에서 정보를 거의 최대로 받으면서
불필요한 busy-wait를 더 늘리지 않는 현실적인 상한이다. 이는 encoder 자체의 내부
카운팅 속도가 아니라 SBC가 양쪽 누적 count를 읽어 오는 주기이다.

AHRS는 `5 ms` sync를 요청하고 ROS publish 상한을 `100 Hz`로 설정했다. `200 Hz`
publish 실험에서는 topic rate가 약 200 Hz로 보이더라도 새 sensor timestamp 기준
유효 데이터가 그보다 낮거나 반복되는 경우가 있었다. reader/publisher mutex 경합을
막기 위해 정상 packet 뒤의 양보 시간을 실측했으며, `1250 us`에서 전체
`robot.launch.py` 실행 중 안정적인 약 `100 Hz`를 확인했다.

| 항목 | 전체 bringup 실측 |
|---|---:|
| `/wheel/encoders` | 약 29.995 Hz |
| `/imu/data` | 약 99.938 Hz |
| `/odom` | 약 29.995 Hz |

최종 odom 갱신은 encoder가 기준이므로 약 30 Hz이다. IMU 100 Hz이면 encoder 한 주기
사이에 대략 3개의 새 orientation을 받을 수 있어 현재 구조에는 충분하다. 더 높은
IMU 주기가 필요한 경우 baud rate, sync data 종류 및 고유 timestamp 비율을 함께
측정해야 하며 topic 표시 주기만 보고 센서가 실제로 새 값을 냈다고 판단하면 안 된다.

### Raspberry Pi CPU affinity와 전체 bringup 확인

`stella_md_node`와 `stella_ahrs_node` launch에 `taskset` prefix를 추가했다.

- `motor_cpu_affinity: 2`: 모터 드라이버 process와 그 thread를 CPU 2에서만 실행
- `imu_cpu_affinity: 3`: AHRS process와 그 thread를 CPU 3에서만 실행
- 제한하지 않으려면 각 값을 `0-3`으로 설정

이 설정은 두 process가 사용할 수 있는 CPU를 제한하는 것이며 CPU core를 독점 예약하는
것은 아니다. 다른 process도 scheduler 설정에 따라 같은 core에서 실행될 수 있다.

RealSense, 양쪽 LiDAR, battery 및 linear motor를 포함한 전체 bringup의 최종 정지
검증에서 `stella_md_node`는 약 `78.3%`, `stella_ahrs_node`는 약 `6.0%`,
`wheel_odometry`는 약 `3.0%` of one core를 사용했다. 이때 진단 subscriber와
`tf2_monitor`도 함께 실행 중이었다. 두 serial driver의 affinity가 각각 CPU 2와 3으로
적용된 것은 PID affinity로 확인했다.

### 설정 위치와 실행 방법

통합 설정 파일:

```text
stella_bringup/param/robot_launch_param.yaml
```

현재 주요 값:

```yaml
encoder_poll_rate_hz: 30
imu_sync_period_ms: 5
imu_publish_rate_hz: 100
imu_read_rate_hz: 0
imu_read_success_sleep_us: 1250
motor_cpu_affinity: 2
imu_cpu_affinity: 3
launch_wheel_odometry: true
enable_legacy_odom: false
```

빌드 및 실행:

```bash
cd ~/colcon_ws
colcon build --packages-select wheel_odometry stella_md stella_ahrs stella_bringup --symlink-install
source install/setup.bash
ros2 launch stella_bringup robot.launch.py
```

동작 확인:

```bash
ros2 topic hz /wheel/encoders
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 topic info /odom -v
ros2 topic echo /wheel_odometry/yaw_diagnostics
ros2 run tf2_ros tf2_echo odom base_footprint
```

전체 변경 후 네 패키지의 `colcon build`를 완료했고, 전체 bringup에서 새
`wheel_odometry`만 `/odom`을 발행하는 것을 확인했다. 종료 시 기존
`linear_motor_node`가 `rclpy.shutdown()`을 두 번 호출하여 exit code 1이 되는 문제는
이번 odometry 변경과 무관한 기존 문제라 수정하지 않았다.

### 새 odom과 legacy odom 전환

두 구현이 동시에 `/odom` 및 같은 TF를 발행하면 안 된다. `robot.launch.py`는 설정
파일에서 `launch_wheel_odometry`와 `enable_legacy_odom`이 동시에 `true`이면 실행을
중단하도록 검사한다.

실행 중 새 odom에서 legacy odom으로 전환:

```bash
ros2 param set /wheel_odometry enabled false
ros2 param set /stella_md_node enable_legacy_odom true
```

legacy에서 새 odom으로 복귀:

```bash
ros2 param set /stella_md_node enable_legacy_odom false
ros2 param set /wheel_odometry enabled true
```

반드시 현재 publisher를 먼저 끈 다음 다른 쪽을 켠다. 실제 runtime 전환 시험에서
각 상태의 `/odom` publisher가 하나만 존재하는 것을 확인했다.

### 정밀도 한계와 향후 보정

encoder 분해능 자체는 충분히 높지만 실제 odom 오차는 주로 다음 항목의 영향을 받는다.

- 좌/우 바퀴의 실제 유효 반지름 차이와 명목 반지름 오차
- 실제 wheel separation 오차
- 가감속/회전 시 미끄러짐과 바닥 상태
- 기어 backlash 및 차체 하중 분포
- 모터, 철제 차체 및 주변 자기장에 의한 9축 IMU magnetometer yaw 왜곡
- 현재 covariance와 complementary filter 값이 실주행 데이터로 추정되지 않았다는 점

직선 실측 보정의 시작값은 다음 식으로 계산할 수 있다.

```text
new_wheel_radius = current_wheel_radius * actual_distance / odom_distance
```

제자리 회전 실측으로 wheel separation을 보정할 때의 시작값은 다음과 같다.

```text
new_wheel_separation = current_wheel_separation * odom_rotation / actual_rotation
```

이 값은 여러 번 왕복/양방향 회전한 평균으로 정하고, 보정 전후 rosbag에서
wheel yaw, IMU yaw 및 fused yaw를 비교해야 한다. 이번 시험은 `/cmd_vel`을 발행하지
않은 정지 시험이므로 실제 직선 거리, 회전각, slip 및 장시간 drift 오차는 아직
정량 검증되지 않았다.

### 향후 `robot_localization` 전환 방침

현재 구현은 별도 EKF 없이 바로 쓸 수 있도록 `wheel_odometry` 안에서 wheel yaw와
IMU yaw를 complementary fusion한다. 향후 Nav2 운용에서 `robot_localization` EKF를
도입할 때는 같은 IMU를 두 번 융합하지 않도록 구조를 바꿔야 한다.

권장 전환값과 데이터 흐름:

```yaml
# wheel_odometry/config/wheel_odometry.yaml
use_imu_orientation: false
odom_topic: /wheel/odom
publish_tf: false
```

```text
/wheel/odom (wheel encoder만 사용) ----\
                                       -> robot_localization EKF
/imu/data (원본 IMU) ------------------/    -> /odometry/filtered
                                            -> odom -> base_footprint TF
```

이때 최종 TF는 EKF 한 곳에서만 발행해야 한다. 현재처럼 IMU가 이미 들어간 `/odom`과
동일한 `/imu/data`를 EKF 입력으로 동시에 넣으면 서로 독립적이지 않은 같은 측정을
중복 반영하여 covariance를 실제보다 과도하게 신뢰하게 된다.

## Update 0903 - 정밀 도킹, 센서 시간 정렬 및 하드웨어 운용 안정화

> 이 섹션은 2026-09-03 작업 트리를 마지막 커밋
> `6d265b0`(2026-08-25, 모터 드라이버·IMU·wheel odometry 변경)과 비교한 결과다.
> 비교 시점에는 수정 파일 25개와 신규 파일 6개, 총 31개 변경 항목이 있었다.
> 아래 내용이 이전의 중간 실험 기록과 충돌하면 이 섹션과 현재 설정 파일을 우선한다.

### 변경 범위 요약

| 영역 | 핵심 변경 | 현재 기본 상태 |
|---|---|---|
| 도킹 접근 | Nav2 접근 뒤 고정 `/dock_pose` 기반 저속 정밀 보정 추가 | 활성 |
| 회전 | yaw 변화량 누적 방식에서 시작 odom yaw 기준 절대 180도 목표로 변경 | 활성 |
| LiDAR 정렬 | 후방 패널 RANSAC의 획득·추적·재획득과 법선 기준 정렬 강화 | 활성 |
| LiDAR 후진 | 후방 거리 감속, 보호 영역, 약한 평면 heading 보정, blocked 진단 추가 | 활성 |
| 좌우 가이드 | 양쪽 프로파일 추정 실험 코드와 테스트 추가 | 운영 경로에서는 비활성·미연결 |
| odometry | encoder 시각에 맞춘 IMU yaw 보간, 비현실적 encoder 변화 거부 | 활성 |
| AHRS | 무제한 serial reader가 publisher thread를 굶기지 않도록 packet yield 추가 | 1,250 us |
| 배터리/리니어모터 | ROS 외부 종료와 하드웨어 resource 정리 강화 | 활성 |
| udev | GPIO, I2C, Xbox 입력 권한 및 설치 후 검증 추가 | 규칙 설치 필요 |
| Xbox | 검색·페어링·연결 단계 분리, 임시 BlueZ agent, xpadneo 도구 추가 | 소스 트리 스크립트로 제공 |
| 문서/테스트 | 현재 동작, 파라미터, 회귀 사례, 운용 절차 문서화 | 추가·갱신 |

### 1. 도킹 동작 변경

#### 최종 실행 순서

현재 `ros2 run docking dock_turn_backup`의 순서는 다음과 같다.

```text
중복 실행 lock 및 전체 timeout 설정
  -> 필요하면 AprilTag/bridge/docking_server 자동 시작
  -> docking_server lifecycle configure/activate
  -> /odom, odom->base_link, /scan_2, base_link<-base_scan2 확인
  -> AprilTag detected_dock_pose 확인
  -> Nav2 DockRobot으로 태그 전방 접근
  -> 필터링된 /dock_pose를 고정 목표로 태그 pose 정밀 보정
  -> 선택적 태그 전방 검증(현재 false)
  -> 선택적 회전 전 직진(현재 false)
  -> 시작 odom yaw + 180도의 절대 목표로 감속 회전
  -> 정지 상태에서 후방 패널 RANSAC 법선 정렬
  -> 후방 LiDAR 거리 기반 감속 후진
  -> 개발 모드에서는 거리 성공, 운영 모드에서는 실제 충전까지 확인
```

#### Nav2 접근 위치와 회전 공간

- `simple_charging_dock.external_detection_translation_x`를 `-0.80m`에서
  `-0.95m`로 변경했다.
- 정밀 보정이 목표를 더 정확히 따라가면서 로봇이 이전보다 스테이션 가까이에서
  회전하던 현상을 보완하기 위해 회전 전 목표를 약 15cm 더 멀리 둔 것이다.
- `docking_threshold`는 `0.15m`를 유지한다. 이 값을 수 cm로 줄이면 AprilTag pose
  노이즈에 맞추려고 전후진을 반복하거나 회전 공간을 잃을 수 있다.
- `navigate_to_staging_pose`는 `false`이므로 `staging_x_offset=-0.80m`는 현재 action
  경로에서 staging 이동에 사용되지 않는다.
- Nav2 controller의 `use_collision_detection`은 현재 `false`이므로 접근 및 회전 공간은
  운영자가 물리적으로 확보해야 한다.

#### 태그 pose 정밀 보정

Nav2의 넓은 성공 반경은 그대로 두고 action 완료 뒤 별도의 제한된 정밀 단계를 추가했다.

- Nav2가 필터링한 `/dock_pose`를 `RELIABLE + VOLATILE` QoS로 구독한다.
- 이전에 사용했던 `TRANSIENT_LOCAL`은 Jazzy 환경의 실제 발행자와 호환되지 않아
  메시지를 받지 못했던 원인이므로 제거했다.
- action이 성공한 순간 최신 목표를 `odom` 좌표계에서 고정한다. 이후 흔들리는 태그를
  계속 쫓지 않는다.
- 제어 명령은 현재 로봇 좌표계의 오차로 만들고, 완료 판정은 고정 dock 목표축의
  종방향·횡방향·yaw 오차로 판단한다.
- action이 이미 15cm 반경 안에서 즉시 성공하는 경우를 위해 `/dock_pose` 콜백을
  최대 1초 기다린다.
- 목표 오차를 만족한 상태가 5주기 연속이어야 성공한다.

현재 주요 값:

```yaml
use_tag_pose_refinement: true
tag_refinement_target_pose_topic: /dock_pose
tag_refinement_target_wait_timeout_sec: 1.0
tag_refinement_target_max_age_sec: 1.5
tag_refinement_timeout_sec: 18.0
tag_refinement_longitudinal_tolerance: 0.04
tag_refinement_lateral_tolerance: 0.025
tag_refinement_yaw_tolerance: 0.034906585  # 2 deg
tag_refinement_stable_cycles: 5
tag_refinement_max_linear_speed: 0.025
tag_refinement_max_angular_speed: 0.08
tag_refinement_max_initial_longitudinal: 0.18
tag_refinement_max_initial_lateral: 0.10
tag_refinement_max_initial_yaw: 0.436332313  # 25 deg
tag_refinement_max_travel: 0.18
tag_refinement_max_yaw_excursion: 0.523598776  # 30 deg
tag_refinement_abort_on_failure: true
```

정밀 보정이 현장에서 불리하면 `use_tag_pose_refinement:=false`로 끄면 Nav2 접근 뒤
바로 회전하던 이전 순서로 돌아간다. 이전에 임시로 추가했던 시간 기반 1초 직진도
독립 파라미터로 남겼지만 현재는 꺼져 있다.

```yaml
use_pre_spin_forward: false
pre_spin_forward_duration_sec: 1.0
pre_spin_forward_speed: 0.03
pre_spin_forward_max_distance: 0.05
```

#### odom 절대 목표 180도 회전

기존에는 매 odom 메시지의 yaw 변화량 절댓값을 누적해 약 180도가 되면 정지했다.
노이즈와 제동 중 변화량까지 누적되고 일정 각속도로 끝까지 회전해 overshoot가 생길 수 있었다.

현재 구현은 다음과 같다.

- 회전 직전 `start_yaw`를 `/odom.pose.pose.orientation`에서 읽는다.
- `target_yaw = normalize(start_yaw + pi)`로 절대 목표를 한 번 계산한다.
- 매 제어 주기 목표까지 남은 최단 각도 오차를 다시 계산한다.
- 정확히 180도일 때 수치 wrap 때문에 회전 방향이 바뀌지 않도록 `spin_yaw`의 부호를 유지한다.
- 최대 `0.15rad/s`로 시작하고 목표 40도 전부터 최소 `0.025rad/s`까지 감속한다.
- odom 목표 오차 1도 이내이며 실제 각속도가 0.5도/초 이하인 상태가 5회 연속일 때 끝낸다.
- 정지 각속도는 신선한 `/imu/data.angular_velocity.z`를 우선하고, IMU가 잠시 stale이면
  encoder 기반 `/odom.twist.twist.angular.z`로 대체한다.

IMU orientation을 회전 목표에 다시 합치지는 않는다. 절대 목표는 odom yaw이며,
IMU gyro는 정지 확인과 후진 중 단기 상대 방향 변화 측정에 사용한다.

#### 후방 LiDAR와 RANSAC 평면 정렬

도킹 전용 센서를 전방 LiDAR가 아닌 후방 LiDAR 계약으로 고정했다.

```text
topic: /scan_2
frame: base_scan2
base_link 기준 후방: pi rad (180 deg)
실제 장착 TF 예: xyz=(-0.166, 0.000, 0.223)m, yaw 약 180deg
```

LaserScan의 로컬 각도를 직접 가정하지 않고 모든 광선과 끝점을 TF로 `base_link`에
투영한다. scan 수신 age, header age, timestamp 단조 증가, frame, range metadata를
검사하며 조건을 어기면 움직이지 않는다.

회전 후 정렬의 현재 조건:

| 항목 | 현재값 |
|---|---:|
| 관측 영역 | 후방 180도 ±30도, 총 60도 |
| range | 0.15~2.0m |
| 최소 점 / inlier | 20 / 12 |
| RANSAC 반복 | 100 |
| inlier 거리 threshold | 0.035m |
| 최소 직선 길이 | 0.15m |
| 기대 방향 대비 후보 최대 오차 | 15도 |
| 정렬 tolerance | 1도 |
| 최대/최소 각속도 | 0.06 / 0.012rad/s |
| 첫 획득 | 고유 scan 3개, 상호 잔차 3도 이내 |
| tracking soft/hard 잔차 | 5도 / 12도 |
| 최대 추가 회전 | 18도 |
| timeout | 12초 |

RANSAC 후보는 단순 inlier 수뿐 아니라 `inlier 수 × 실제 선분 길이`로 평가한다.
회전 직후 첫 왜곡 scan 하나로 움직이지 않고 3개 scan의 합의를 기다린다. 추적 중
5~12도 jump는 정지 후 재획득하고, 12도 초과만 다른 구조물로 전환된 위험으로 보고
실패한다. 평면의 접선이 로봇 좌우축과 평행하도록 맞추므로 패널 법선과 로봇 전후축이
일치하고, 결과적으로 패널을 향해 수직으로 후진한다.

#### LiDAR 거리 후진과 약한 heading 보정

- 완료 거리 영역은 후방 `180도 ±10도`다.
- 충돌 보호 영역은 후방 `180도 ±30도`이며 차체 반폭 0.22m + margin 0.02m 안의
  점만 사용한다.
- 이전 총 150도 보호 fan은 정상적인 좌우 가이드 프레임을 장애물로 포함해 진입 직후
  멈추는 원인이 되어 총 60도로 줄였다.
- LiDAR 위치와 실제 차체 후단 `backup_rear_reference_x=-0.2295m`를 분리해 차체 기준
  clearance를 계산한다.
- 15cm부터 `0.05m/s`에서 `0.015m/s`까지 감속한다.
- 최대 이동 0.60m, 단계 timeout 45초다.
- 성공에는 인접 beam 5개 이상, 최소 3도 폭, 서로 다른 scan 3회의 연속 확인이 필요하다.
- 한 개의 가까운 반사점은 로봇을 멈출 수 있지만 성공으로 만들지는 않는다.

후진 중 RANSAC 보정은 흔들림을 줄이기 위해 다음처럼 제한했다.

```text
법선 오차 LPF coefficient: 0.15
각도 deadband: 1 deg
최대 각속도: 0.004 rad/s
최대 각속도 변화율: 0.010 rad/s^2
단일 sample jump 거부: 2.5 deg
품질 조건: 오차 <= 5 deg, inlier ratio >= 0.70, line length >= 0.15m
활성화 전 안정성: 고유 scan 3개
보정 비활성 거리: protective clearance <= 0.10m
```

후진 중 각도를 고치기 위해 선속도를 멈추지 않는다. RANSAC이 끊기거나 품질에서
탈락하면 직전 angular 명령을 유지하지 않고 `angular.z=0`으로 직진한다. 유효한
평면이 없을 때 IMU 적분 또는 encoder-only wheel yaw drift가 5도를 넘으면 실패한다.

좌우 알루미늄 가이드의 양쪽 직선을 검출해 중앙을 구하는 코드와 회귀 테스트도
추가했지만 현재 운영 경로에서는 사용하지 않는다.

- `use_lidar_guide_centering` 기본값은 `false`다.
- 메인 노드는 guide estimator를 `MotionController`에 연결하지 않는다.
- 따라서 플래그만 `true`로 변경해도 중앙 추종이 활성화되지 않는다.
- 현재는 후면 패널 RANSAC의 각도 보정만 사용한다.

#### 최종 거리 성공 기준 변경

초기 코드 기본 목표는 1.0cm였고, 시험 중 1.5cm로 한 번 조정했다. 최근 물리적으로
성공한 위치에서 중앙 clearance가 2.3cm였지만 소프트웨어 상한 2.0cm를 3mm 초과해
`blocked outside the completion condition`으로 끝난 사례를 반영해 현재 YAML은 다음과 같다.

```yaml
backup_target_rear_clearance: 0.020
backup_clearance_tolerance: 0.005
```

즉 현재 중앙 완료 허용 범위는 `0.015~0.025m`다. 보호 fan이 먼저 2.5cm 이하가 되면
안전상 정지하고 중앙 군집을 기다린다. 1초 안에 중앙 군집이 완료 조건을 만족하지
않으면 여전히 실패한다. 단순히 timeout을 늘리는 것은 측정 위치가 바뀌지 않으므로
판정 불일치의 해결책이 아니다.

`development_test_mode` 기본값은 `true`이므로 LiDAR 거리 성공 뒤 실제 충전 접촉과
전류를 확인하지 않는다. 실제 운영에서는 `false`로 실행해야 `/sk120/available`,
`/sk120/cmd_output`, `/battery_state.current >= 0.05A`의 3초 안정성을 확인한다.

#### 도킹 문서와 테스트

- 신규 `docking/README.md`에 전체 상태 순서, 센서 계약, 모든 핵심 파라미터,
  로그 해석, 실패 사례, 튜닝 순서, exit code를 기록했다.
- `test_lidar_alignment.py`를 추가해 긴 후면 패널 선택, 회전 중 동일 평면 추적,
  실제 실패값 13.54도→6.22도 획득 jump, 5.14도 tracking jump 재획득,
  양쪽 가이드 검출과 한쪽 가이드 거부를 검증한다.
- `test_motion_control.py`를 추가해 절대 회전 목표, 감속, 정지 안정성, IMU 적분,
  heading LPF·jump reject·deadband·rate limit, 정밀 태그 보정 수렴을 검증한다.
- 기존 LiDAR 기본값·기하 테스트에 후방 60도 보호 sector, 평면 법선 오차,
  YAML 핵심값과 정밀 보정 파라미터 검증을 추가했다.
- YAML을 읽는 테스트를 위해 `docking/package.xml`에 `python3-yaml` test dependency를 추가했다.

### 2. wheel odometry 시간 정렬과 이상치 방어

`wheel_odometry`가 최신 IMU 하나를 모든 encoder sample에 반복 적용하던 방식에서,
timestamp 이력을 유지해 encoder 측정시각에 맞는 yaw를 선택하는 방식으로 변경됐다.

- `std::deque<ImuYawSample>`에 기본 1초 동안 IMU yaw와 timestamp를 저장한다.
- out-of-order IMU timestamp는 경고 후 버린다.
- 동일 timestamp가 다시 오면 같은 sample의 yaw만 갱신한다.
- encoder timestamp 앞뒤에 IMU sample이 있으면 yaw wrap을 고려한 최단 각도 방향으로
  보간한다.
- 과거 sample만 있으면 `imu_timeout_sec=0.25` 안의 causal sample만 사용한다.
- 미래 sample 하나만으로 과거 encoder 측정값을 보정하지 않는다.
- 같은 IMU 측정값은 한 번만 complementary correction에 사용한다.
- IMU correction으로 pose yaw가 변해도 `/odom.twist.twist.angular.z`에는 이를 넣지 않고
  실제 좌우 wheel 차동 yaw rate만 발행한다.
- 어느 한쪽 wheel 속도라도 `max_wheel_speed_m_s=2.0`을 넘으면 counter reset,
  rollover 또는 손상 sample로 보고 pose에 반영하지 않은 채 baseline만 재설정한다.

추가된 설정:

```yaml
imu_history_duration_sec: 1.0
max_wheel_speed_m_s: 2.0
```

`/wheel_odometry/yaw_diagnostics`의 의미는 다음과 같다.

```text
vector.x = encoder-only 누적 yaw
vector.y = encoder 시각에 정렬된 IMU yaw, 없으면 NaN
vector.z = 최종 fused odom yaw
```

### 3. AHRS reader/publisher 경합 완화

`read_rate_hz=0`은 serial buffer를 제한 없이 비우지만 vendor read와 ROS publisher가
같은 mutex를 사용한다. reader가 계속 packet을 얻으면 publisher thread가 mutex를
잡지 못해 `/imu/data`가 목표 100Hz에 못 미치는 현상이 있었다.

이를 위해 정상 packet을 decode한 뒤 reader가 잠깐 양보하는 파라미터를 추가했다.

```yaml
read_success_sleep_us: 1250
```

- `mw_ahrs.cpp`가 이 값을 선언·검증하고 uncapped read loop에 적용한다.
- `stella_ahrs_launch.py`가 launch argument로 노출한다.
- `stella_ahrs/config/config.yaml`과 `robot_launch_param.yaml`에 기본값을 기록한다.
- `robot.launch.py`가 통합 설정값을 AHRS launch로 전달한다.
- 최종 정지 bringup 측정에서 `/imu/data` 약 99.938Hz,
  `/wheel/encoders`와 `/odom` 약 29.995Hz를 확인했다.

### 4. 배터리와 리니어모터 종료 안정화

#### 배터리 노드

- `ExternalShutdownException`을 정상 종료 경로로 처리한다.
- `BatteryNode` 생성 전에 예외가 나도 `node`를 참조하지 않도록 `None`으로 시작한다.
- INA219가 사용하는 SMBus를 `StellaN5Monitor.close()`로 명시적으로 닫는다.
- SK120 close와 SMBus close 뒤 `Node.destroy_node()`가 실행되도록 정리 순서를 보강했다.
- 이미 shutdown된 context에서 `rclpy.shutdown()`을 다시 호출하지 않는다.

#### 리니어모터 노드

- `ExternalShutdownException`을 정상 종료로 처리한다.
- 종료 시 `motor.stop()` 뒤 `motor.close()`로 GPIO resource를 해제한다.
- node 생성 실패와 이미 종료된 ROS context를 안전하게 처리한다.
- 런타임 의존성 `python3-gpiozero`, `python3-lgpio`를 `package.xml`에 추가했다.

이 변경으로 launch 종료 시 발생하던 double shutdown 또는 GPIO/I2C resource 잔류 가능성을
줄였다.

### 5. bringup 설정과 udev 권한

`stella_bringup/create_udev_rules.sh`는 단순 규칙 복사에서 설치·재로드·권한 검증까지
수행하도록 확장됐다.

- `install -m 0644`로 `/etc/udev/rules.d/stella.rules`를 설치한다.
- `udevadm control --reload-rules`, `trigger`, `settle`을 순서대로 실행한다.
- 기존 serial alias와 함께 `/dev/gpiochip4`, `/dev/i2c-1` 존재 및 현재 사용자 read/write를 확인한다.
- Microsoft vendor `045e` Bluetooth gamepad event를 찾아 읽기 권한을 확인한다.
- 필수 GPIO/I2C 권한이 준비되지 않으면 exit 1로 명확히 실패한다.

추가 udev 규칙:

```text
/dev/gpiochip4 -> group plugdev, mode 0660
/dev/i2c-1     -> group plugdev, mode 0660
Xbox event/js  -> Microsoft 045e + joystick 장치만 group plugdev, mode 0660
```

모든 input 장치를 전역 공개하지 않고 Xbox joystick 장치만 허용한다. 규칙을 설치한 뒤에는
기기를 재연결해야 새 권한으로 device node가 생성된다.

### 6. Xbox Bluetooth 페어링·재연결 개선

기존 스크립트는 검색을 켠 채 pair/connect를 진행했다. 현재는 Xbox 광고 packet을
확인한 뒤 active scan을 먼저 끄고 GATT pairing/connection을 수행한다.

#### `xbox_pair.sh`

- MAC 형식과 필수 명령을 먼저 검사한다.
- 다른 UI 또는 `bluetoothctl`이 이미 discovery 중이면 충돌을 피하기 위해 거부한다.
- 새 임시 `KeyboardDisplay` BlueZ D-Bus agent를 실행한다.
- 이전 실패가 남긴 temporary device object를 제거하고 fresh pairing을 시작한다.
- 광고를 찾은 뒤 scan을 끄고, 명시적 timeout과 `Pairing successful` 출력으로 성공을 검증한다.
- `Paired`와 `Bonded`가 3회 연속 유지되는지 확인한다.
- trust를 적용하고 필요하면 다시 광고를 받은 뒤 scan off 상태에서 connect한다.
- `Connected`가 5회 연속 유지되는지 확인한다.
- Microsoft Bluetooth input event를 찾아 현재 사용자가 읽을 수 있는지 확인한다.
- 성공·실패와 관계없이 adapter, device, agent, bluetooth journal 로그를
  `~/.local/state/teleop_joy/pair.*`에 남긴다.

#### `xbox_connect.sh`

- 기존 `Paired/Bonded` 상태를 먼저 확인한다.
- trust를 재확인하고 광고 packet을 받은 뒤 scan을 끈다.
- connect 명령의 return code와 `Connection successful` 문자열을 함께 검사한다.
- 연결 상태 5회 연속 유지와 input event 읽기 권한을 확인한다.
- 임시 scan process와 작업 디렉터리를 trap으로 정리한다.

#### 신규 보조 도구

- `xbox_bluez_agent.py`: PIN, passkey, confirmation, authorization 요청을 처리하는
  실행 중 임시 D-Bus agent이며 종료 시 등록을 해제한다.
- `reset_xbox_bluetooth.sh`: `/etc/bluetooth/main.conf.teleop_joy.bak`의 원본 설정을
  복구하고 현재 설정과 기존 bond 디렉터리를 timestamp 백업한 뒤 Bluetooth를 재시작한다.
- `install_xpadneo.sh`: 검증 태그 `v0.10.4`를 `~/xpadneo`에 clone하고 현재 실행 커널과
  정확히 일치하는 header 및 DKMS를 설치한 뒤 `hid_xpadneo` 자동 로드를 구성한다.

이 세 파일과 pair/connect 스크립트는 `teleop_joy/setup.py`의 `data_files`에 포함되지
않으므로 현재는 install space 명령이 아니라 소스 트리
`teleop_joy/scripts/`에서 직접 실행해야 한다.

### 7. 변경 파일 전체 목록

마지막 커밋 이후 확인된 31개 항목을 빠짐없이 정리하면 다음과 같다.

#### 수정 파일 25개

| 파일 | 변경 내용 |
|---|---|
| `README.md` | RealSense, docking, battery, Update 0825 내용을 확장하고 현재 Update 0903 및 중간 실험 설명의 불일치 수정 |
| `battery/battery/battery_status.py` | 외부 shutdown 처리와 INA219 SMBus 정리 |
| `docking/config/docking.yaml` | 태그 목표 -0.95m, 정밀 보정·RANSAC 안정성·선택 직진·최종 2.0cm 목표 설정 |
| `docking/docking/dock_turn_backup.py` | 태그 정밀 보정, 선택 위치 검증/직진, QoS·action race 처리, 기본 YAML 자동 로드 |
| `docking/docking/lidar_alignment.py` | 평면 법선 RANSAC, 길이 가중 선택, 획득·추적·재획득, 실험용 양쪽 guide 추정 |
| `docking/docking/lidar_geometry.py` | 평면 법선 오차와 양쪽 guide center 자료형 추가 |
| `docking/docking/motion.py` | 절대 odom 회전, IMU/휠 yaw 보조, 선택 직진, 후진 heading·blocked·보호 제어 |
| `docking/package.xml` | 테스트용 `python3-yaml` 의존성 추가 |
| `docking/test/test_lidar_defaults.py` | 현재 기본 파라미터, YAML, 태그 보정 회귀 테스트 확대 |
| `docking/test/test_lidar_geometry.py` | 후방 60도 sector와 평면 법선 기하 테스트 추가 |
| `linear_motor/linear_motor/motor_control.py` | 외부 shutdown, motor stop/close, 중복 shutdown 방지 |
| `linear_motor/package.xml` | gpiozero와 lgpio runtime dependency 추가 |
| `stella/stella_ahrs/config/config.yaml` | `read_success_sleep_us: 1250` 추가 |
| `stella/stella_ahrs/launch/stella_ahrs_launch.py` | reader yield launch argument 전달 |
| `stella/stella_ahrs/mw/mw_ahrs.cpp` | 정상 packet 뒤 reader yield 및 설정 로그 추가 |
| `stella_bringup/create_udev_rules.sh` | 규칙 설치·reload·settle·장치 권한 검증 강화 |
| `stella_bringup/launch/robot.launch.py` | 통합 AHRS reader yield 설정 전달 |
| `stella_bringup/param/robot_launch_param.yaml` | `imu_read_success_sleep_us` 기본값 추가 |
| `stella_bringup/stella.rules` | GPIO4, I2C1, Microsoft Xbox input 권한 규칙 추가 |
| `teleop_joy/README.md` | scan-off pairing, reset, udev, xpadneo, 진단 로그 운용법 문서화 |
| `teleop_joy/scripts/xbox_connect.sh` | 광고 확인 후 scan off, 안정 연결 및 input 권한 검증 |
| `teleop_joy/scripts/xbox_pair.sh` | 임시 agent, 단계별 timeout, 안정 상태, 영구 진단 로그 추가 |
| `wheel_odometry/README.md` | encoder-time IMU 보간, wheel-only twist, encoder 이상치 처리 문서화 |
| `wheel_odometry/config/wheel_odometry.yaml` | IMU history 1초와 wheel 속도 2.0m/s 안전 상한 추가 |
| `wheel_odometry/src/wheel_odometry_node.cpp` | timestamp 보간, causal sample 선택, 중복 IMU 보정 방지, encoder 이상치 거부 |

#### 신규 파일 6개

| 파일 | 목적 |
|---|---|
| `docking/README.md` | 도킹 설계·센서 계약·파라미터·로그·튜닝·실패 사례 종합 문서 |
| `docking/test/test_lidar_alignment.py` | RANSAC 획득·추적·재획득 및 guide 추정 회귀 테스트 |
| `docking/test/test_motion_control.py` | 절대 회전·태그 보정·후진 heading 제어 회귀 테스트 |
| `teleop_joy/scripts/install_xpadneo.sh` | xpadneo v0.10.4 및 현재 커널 DKMS 설치 |
| `teleop_joy/scripts/reset_xbox_bluetooth.sh` | 원본 BlueZ 설정 복구와 bond 백업·초기화 |
| `teleop_joy/scripts/xbox_bluez_agent.py` | Xbox 인증 요청을 처리하는 임시 BlueZ D-Bus agent |

### 8. 호환성 및 운용 주의사항

- `dock_turn_backup`은 `/cmd_vel` publisher를 코드에서 자동 배타 검사하지 않는다.
  실행 전 `ros2 topic info /cmd_vel --verbose`로 teleop/navigation publisher를 확인한다.
- `development_test_mode=true`의 성공은 LiDAR 거리 성공이지 충전 성공이 아니다.
- Nav2 docking server의 `Robot is charging!` 로그도 battery/stall 검증이 꺼진 현재
  plugin 설정에서는 실제 전류 확인으로 해석하면 안 된다.
- LiDAR 좌우 guide 코드는 존재하지만 운영 경로에는 연결되지 않았다.
- 최종 중앙 성공 범위는 현재 `1.5~2.5cm`다. 보호 영역의 단일 가까운 점 때문에 먼저
  멈추고 중앙 군집이 완성되지 않으면 여전히 blocked 실패가 가능하다.
- `reset_xbox_bluetooth.sh`는 원본 백업
  `/etc/bluetooth/main.conf.teleop_joy.bak`이 있을 때만 동작한다.
- Xbox 보조 스크립트는 Bluetooth 서비스, DKMS, udev 상태를 실제로 변경하므로 내용을
  확인하고 컨트롤러 상태 안내에 맞춰 실행한다.
- udev 규칙은 현재 사용자가 `plugdev` 그룹에 속한다는 전제를 가진다.
- build/install space의 YAML을 실행하므로 소스 설정을 바꾼 뒤에는 반드시 다시 빌드하고
  `source ~/colcon_ws/install/setup.bash`를 실행한다.

### 9. 검증 결과와 재현 명령

2026-09-03 현재 수행한 비하드웨어 검증:

```text
docking pytest: 61 passed
변경 Python 파일 py_compile: 통과
xbox/udev shell bash -n: 통과
관련 ROS 2 패키지 7개 symlink build: 통과
```

빌드한 패키지:

```text
battery
docking
linear_motor
stella_ahrs
stella_bringup
teleop_joy
wheel_odometry
```

재현 명령:

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select \
  docking wheel_odometry stella_ahrs battery linear_motor teleop_joy stella_bringup

cd ~/colcon_ws/src/STELLA_N5_ROS2/docking
python3 -m pytest test -q

cd ~/colcon_ws/src/STELLA_N5_ROS2
bash -n stella_bringup/create_udev_rules.sh \
  teleop_joy/scripts/xbox_connect.sh \
  teleop_joy/scripts/xbox_pair.sh \
  teleop_joy/scripts/install_xpadneo.sh \
  teleop_joy/scripts/reset_xbox_bluetooth.sh
```

위 검증은 compile, unit test, shell syntax 범위다. 실제 Xbox 재페어링, udev 재설치,
GPIO/I2C 하드웨어 제어, 충전 전류 확인, 반복 실주행 도킹은 장치 상태를 바꾸므로 이번
문서 작업에서는 다시 실행하지 않았다. 도킹은 기존 현장 실행에서 물리 성공을 확인했고,
마지막 거리 판정 불일치를 반영해 목표를 2.0cm로 조정한 상태다.

### 10. 권장 커밋 메시지

전체 변경을 하나의 커밋으로 남길 경우 권장 제목은 다음과 같다.

```text
feat: 정밀 도킹과 센서·하드웨어 운용 안정성 강화
```

권장 본문:

```text
- AprilTag 정밀 보정과 odom 절대 180도 회전을 추가
- 후방 LiDAR RANSAC 획득·재획득 및 보수적 후진 제어를 강화
- IMU/encoder 시간을 정렬하고 encoder 이상치를 거부
- AHRS reader/publisher 경합과 하드웨어 노드 종료 처리를 개선
- GPIO/I2C/Xbox udev 권한과 Xbox 페어링 도구를 추가
- 변경 설계, 파라미터, 실패 사례와 검증 방법을 문서화
```

변경 범위가 넓어 이력을 더 읽기 쉽게 만들려면 실제로는 아래처럼 네 커밋으로 나누는
것이 가장 좋다.

```text
feat(docking): AprilTag·odom·LiDAR 기반 정밀 도킹 안정화
fix(odometry): IMU와 encoder 시간 정렬 및 이상치 방어
fix(hardware): AHRS pacing과 배터리·리니어모터 종료 안정화
feat(teleop): Xbox 페어링·xpadneo·udev 지원 강화
```

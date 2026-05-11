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
  - 기본값은 `10Hz`로 설정함
  - 관련 파일:
    - `stella/stella_md/src/main.cpp`
    - `stella/stella_md/launch/stella_md_launch.py`

- `stella_ahrs_node`
  - AHRS read/publish 주기를 낮춰 CPU 부하를 줄임
  - 기본값:
    - `read_rate_hz: 200`
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
  port:=/dev/ttyUSB4 \
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

참고: SK120 USB-TTL 기본 포트는 `/dev/ttyUSB4`로 설정되어 있음. 장비에서 포트가 달라지면
`port:=...` launch 파라미터 또는 `battery/launch/battery.launch.py` 기본값을 현장 설정에 맞게 변경

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
- `docking/docking/lidar_alignment.py`: `/scan` 평면 검출 및 yaw 미세 보정
- `docking/config/docking.yaml`: OpenNav docking server 파라미터
- `docking/config/tags_36h11.yaml`: AprilTag 인식 파라미터

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
후진하기 전에 `/scan`에서 벽/태그 평면을 찾아 yaw를 미세 보정함

현재 도킹 시나리오에서는 180도 회전이 끝난 뒤 로봇이 후진으로 벽/태그 쪽에 접근하므로,
보정에 사용할 평면은 로봇 기준 후방에 위치함. 따라서 실제 운용에서는
`lidar_align_sector_center:=3.14159`를 사용해 후방 60도 영역을 보는 설정을 권장함.

기본 동작:

- `lidar_align_sector_center`를 중심으로 `lidar_align_sector_width`만큼의 LaserScan 점만 사용
- 현재 후진 도킹 시나리오 권장값은 후방 60도 영역
- RANSAC으로 가장 그럴듯한 직선 평면을 찾음
- 평면의 normal 방향이 로봇 뒤쪽(`pi`)을 향하도록 `/cmd_vel.angular.z`로 저속 보정
- 오차가 약 2도 이내로 안정되면 후진 단계로 넘어감

주요 튜닝 파라미터:

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_lidar_alignment:=true \
  -p lidar_align_sector_center:=3.14159 \
  -p lidar_align_sector_width:=1.0472 \
  -p lidar_align_tolerance:=0.0349 \
  -p lidar_align_angular_speed:=0.08 \
  -p lidar_align_timeout_sec:=8.0
```

정렬 방향이 반대로 보이면 `-p lidar_align_kp:=-0.8`로 부호를 바꿔 테스트할 수 있음
LiDAR 평면이 잘 안 잡히면 `lidar_align_sector_width`, `lidar_align_max_range`,
`lidar_align_ransac_threshold`를 현장 구조에 맞춰 조정
LiDAR `/scan` 프레임에서 0도가 전방이 아닌 장착 구조라면 `lidar_align_sector_center`를 실제 평면이 보이는 방향으로 조정

참고: 코드의 기본값은 `lidar_align_sector_center:=0.0`이므로 파라미터를 따로 주지 않으면 전방 60도 영역을 사용함.
현재 후진 도킹 시나리오에서는 위 예시처럼 `3.14159`를 명시하는 것이 안전함

### 도킹 실행 전 TF 확인

OpenNav docking server는 `/odom` 토픽뿐 아니라 `odom -> base_link` TF transform이 필요함
`dock_turn_backup`은 도킹 goal을 보내기 전에 `/odom` 메시지와 `odom -> base_link` TF가 준비될 때까지 대기
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
```

`tf2_echo`가 처음 1초 정도 대기 메시지를 출력한 뒤 transform을 계속 출력하면 정상
계속 대기 상태라면 bringup의 `stella_md_node`, `robot_state_publisher`, `/tf`, `/tf_static` 상태 확인
필요하면 `-p docking_server_tf_warmup_sec:=3.0`처럼 대기 시간을 늘려 테스트

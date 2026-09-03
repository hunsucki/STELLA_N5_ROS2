# teleop_joy

Xbox One 블루투스 게임패드의 입력을 STELLA N5용 `/cmd_vel`
(`geometry_msgs/msg/Twist`)로 변환하는 ROS 2 패키지입니다. SDL의 표준
GameController 매핑을 사용하는 `joy/game_controller_node`도 launch 파일에서
함께 실행합니다.

## 기본 조작

| 조작 | 동작 |
|---|---|
| `B + RT` | 누른 양에 비례하여 전진 |
| `B + LT` | 누른 양에 비례하여 후진/제동 |
| `B + 왼쪽 스틱 좌/우` | 좌/우 회전 |
| `B + LT + RT` | 두 입력의 차이만큼 주행; 같은 양이면 정지 |
| `B`를 놓음 | 즉시 정지 |

직진 속도는 `(RT - LT) × max_linear_speed`로 계산합니다. 조향은 이 계산과
독립적이므로 RT를 끝까지 누른 최대 전진 상태에서도 왼쪽 스틱으로 회전할 수
있습니다. 모든 명령은 B 버튼을 누르고 있을 때만 전달됩니다. B 버튼을 놓거나
패드 입력이 0.5초 이상 끊기면 즉시 정지 명령을 발행합니다.

## Xbox 패드 블루투스 연결

컨트롤러의 Xbox 버튼을 켠 뒤 페어링 버튼을 길게 눌러 Xbox 버튼이 빠르게
깜빡이게 합니다. Ubuntu에서 다음과 같이 연결할 수 있습니다.

```bash
bluetoothctl
power on
agent on
default-agent
scan on
# 출력에서 Xbox Wireless Controller의 주소를 확인합니다.
scan off
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

스크립트는 검색 중 Xbox 광고 패킷을 확인한 뒤 검색을 종료하고 pair/connect를
실행합니다. 활성 검색을 연결 과정까지 유지하지 마십시오.

이 저장소의 이전 연결 파라미터 실험을 적용했다면 한 번 원본 BlueZ 설정과
본딩 상태로 초기화합니다. 기존 설정과 본딩은 삭제하지 않고 별도 위치에
백업되며 Bluetooth 서비스가 한 번 재시작됩니다.

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/teleop_joy/scripts
./reset_xbox_bluetooth.sh
```

초기화할 때는 컨트롤러를 완전히 끄고, 열려 있는 `bluetoothctl`과 Bluetooth
설정 창도 모두 닫아야 합니다. 초기화 후 컨트롤러를 빠른 점멸 상태로 만들어
`./xbox_pair.sh`를 실행합니다.

Ubuntu에서 `/dev/input/event*`가 `root:input` 전용으로 생성되면 연결은
성공해도 `/joy`가 발행되지 않습니다. 이 저장소의 udev 규칙에는 Xbox 입력을
`plugdev` 그룹에 허용하는 설정이 포함되어 있습니다. 최초 한 번 적용합니다.

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/stella_bringup
./create_udev_rules.sh
```

적용 후 컨트롤러를 껐다 켜거나 아래 재연결 스크립트를 실행해야 새 권한으로
입력 장치가 다시 생성됩니다.

컨트롤러가 빠르게 점멸하는 최초 페어링 모드일 때:

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/teleop_joy/scripts
./xbox_pair.sh
```

페어링 한 번이 실패하면 컨트롤러의 빠른 점멸도 종료될 수 있습니다. 재시도할
때마다 페어링 버튼을 다시 길게 눌러 빠른 점멸을 새로 시작해야 합니다. 이
스크립트는 Xbox의 숫자 확인 인증 요청을 처리할 수 있도록
`KeyboardDisplay` 에이전트를 사용합니다.
에이전트는 페어링 스크립트가 실행되는 동안에만 등록되며, 종료할 때 자동으로
해제됩니다. 실패 시 상세 로그는 `~/.local/state/teleop_joy/pair.*`에 남습니다.

Xbox One S Bluetooth 전용 커널 드라이버인 `xpadneo`를 설치할 수 있습니다.
설치 스크립트는 검증된 안정 태그 `v0.10.4`를
사용하고 현재 실행 중인 Raspberry Pi 커널과 정확히 일치하는 헤더를 설치한
뒤 DKMS 모듈을 빌드합니다.

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/teleop_joy/scripts
./install_xpadneo.sh
```

설치할 때는 컨트롤러를 완전히 끄고, 설치 후 다시 켜서
`./xbox_connect.sh`를 실행합니다. 내려받은 공식 저장소는 `~/xpadneo`에
보관되며 `~/xpadneo/uninstall.sh`로 제거할 수 있습니다.

이미 페어링된 컨트롤러가 일반적으로 점멸하며 재연결을 기다릴 때:

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/teleop_joy/scripts
./xbox_connect.sh
```

다른 컨트롤러를 사용할 때는 MAC을 첫 번째 인자로 전달합니다.

```bash
./xbox_pair.sh AA:BB:CC:DD:EE:FF
./xbox_connect.sh AA:BB:CC:DD:EE:FF
```

ROS 드라이버가 없다면 ROS 2 Jazzy 기준으로 설치합니다.

```bash
sudo apt update
sudo apt install ros-jazzy-joy
```

## 빌드 및 실행

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select teleop_joy
source install/setup.bash
ros2 launch teleop_joy teleop_joy.launch.py
```

다른 SDL 장치가 0번을 사용하고 있으면 장치 목록을 확인한 다음 ID를
지정합니다.

```bash
ros2 run joy joy_enumerate_devices
ros2 launch teleop_joy teleop_joy.launch.py device_id:=1
```

기존 `/joy` 발행기를 사용하려면 내장 드라이버만 끌 수 있습니다.

```bash
ros2 launch teleop_joy teleop_joy.launch.py launch_joy_node:=false
```

패드를 확인할 때는 로봇 바퀴를 지면에서 띄우거나 비상 정지 가능한 상태에서
아래 토픽을 확인합니다.

```bash
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```

## 속도와 버튼 설정

기본 설정은 [`config/xbox_one.yaml`](config/xbox_one.yaml)에 있습니다.

- 최대 속도: `0.70 m/s`, `1.8 rad/s`
- SDL Xbox 표준 매핑: 왼쪽 X축 `0`, LT/RT 축 `4/5`
- 출력 주기: `20 Hz`, 패드 timeout: `0.5초`

별도 YAML을 복사해 값을 수정한 뒤 다음처럼 적용할 수 있습니다.

```bash
ros2 launch teleop_joy teleop_joy.launch.py \
  params_file:=/absolute/path/to/my_xbox.yaml
```

회전 방향이 실제 패드 환경에서 반대로 보이면 `max_angular_speed` 값을 음수로
바꾸지 말고 `steering_axis` 입력 방향을 코드 또는 매핑에서 반전해야 합니다.

## 안전 주의사항

이 패키지는 `/cmd_vel`을 직접 발행합니다. Nav2, 도킹, 기존 teleop 등 다른
`/cmd_vel` 발행기와 동시에 사용하지 마십시오. 여러 제어원을 함께 써야 한다면
각 출력 토픽을 분리한 뒤 velocity mux를 사용해야 합니다.

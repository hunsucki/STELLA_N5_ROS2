# gimbal_camera_capture

SIYI A8 mini 짐벌 카메라 두 대에서 ROS 2 토픽 명령을 받을 때마다 최신
프레임을 한 장씩 병렬 캡처하고, 각 카메라의 yaw/pitch 회전, 디지털 줌과
중앙 복귀를 ROS 2 토픽으로 제어하는 독립 패키지다.

촬영 서비스 노드는 `stella_bringup/launch/robot.launch.py`에 등록되어 있다.
`robot_launch_param.yaml`의 `launch_gimbal_camera_capture: true`가 기본값이므로
로봇 bringup과 함께 시작해 촬영 요청을 기다린다. 대기 중에는 RTSP 스트림을
열지 않으며 실제 촬영 요청을 처리할 때만 두 카메라에 접속한다. 수동 짐벌
제어용 `control_node`는 별도로 실행한다.

## 1. 현재 구성 요약

2026-07-26 기준으로 이 라즈베리파이에 적용하고 실제 촬영까지 확인한 구성은
다음과 같다.

```text
공유기/ROS 2 네트워크
        │ Wi-Fi: wlan0 (현재 192.168.0.15/24)
        │ ROS 2 Cyclone DDS는 이 인터페이스만 사용
        ▼
라즈베리파이
        ├── eth0 (192.168.144.10/32)
        │     └── Left 카메라  (192.168.144.25:8554)
        │
        └── USB-LAN: enx00e04c3628a8 (192.168.144.11/32)
              └── Right 카메라 (192.168.144.26:8554)
```

스위칭 허브는 더 이상 사용하지 않는다. 카메라마다 전용 이더넷 인터페이스가
하나씩 있으므로 두 링크는 서로 다른 물리 네트워크이며, 카메라 두 대가 같은
MAC 주소를 사용하더라도 충돌하지 않는다.

| 구분 | Left | Right |
|---|---|---|
| ROS 결과 키/저장 폴더 | `left` | `right` |
| 카메라 주소 | `192.168.144.25` | `192.168.144.26` |
| 카메라 RTSP 포트 | `8554` | `8554` |
| 카메라 실제 제어 UDP 포트 | `37260` | `37260` |
| 라즈베리파이 인터페이스 | `eth0` | `enx00e04c3628a8` |
| 라즈베리파이 링크 주소 | `192.168.144.10/32` | `192.168.144.11/32` |
| Wi-Fi RTSP 접근 포트 | `8554` | `8555` |
| Wi-Fi 제어 접근 포트 | `37260` | `37261` |

패키지는 라즈베리파이 안에서 실행되므로 NAT 포트를 거치지 않고 카메라의
실제 주소인 `.25:8554/.25:37260`, `.26:8554/.26:37260`에 직접 접속한다.
실제 펌웨어 조회 결과 두 카메라 모두 UDP `37260`에 응답했고 Right의
`.26:37261`에는 응답하지 않았다. `37261`은 Wi-Fi/외부에서 Right를 구분하기
위한 라즈베리파이 입력 포트일 뿐이다.

## 2. 왜 카메라를 두 네트워크로 분리했는가

일반 스위칭 허브는 이더넷 프레임의 출발지 MAC 주소를 보고 해당 MAC이 어느
포트에 있는지 학습한다. 두 카메라의 IP 주소가 다르더라도 MAC 주소가 같으면
스위치는 같은 MAC이 두 포트 사이를 이동하는 것으로 판단한다. 그 결과 MAC
테이블이 계속 바뀌어 패킷이 잘못된 포트로 전달될 수 있다.

이 문제는 IP 계층보다 아래인 데이터 링크 계층에서 발생하므로 IP 주소만
`.25`, `.26`으로 다르게 설정해도 해결되지 않는다. 현재는 Left를 내장 LAN,
Right를 USB-LAN에 직결하여 두 카메라를 서로 다른 L2 구간으로 분리했다.

두 인터페이스가 모두 `192.168.144.x` 주소를 사용하기 때문에 라즈베리파이는
일반적인 `/24` 연결 경로 대신 다음 `/32` 호스트 경로를 사용한다.

```text
192.168.144.25 dev eth0
192.168.144.26 dev enx00e04c3628a8
```

이렇게 해야 어느 카메라 패킷을 어느 케이블로 보내야 하는지가 항상
명확하다.

## 3. ROS 2 인터페이스와 동작

### drive_manager 자동 운용 서비스

자동 순회에서는 다음 `inspection_interfaces` 서비스를 사용한다.

| 서비스 | 역할 |
|---|---|
| `/camera/capture_run/start` | mission/map/활성 구역 스냅샷으로 run 시작 |
| `/camera/capture_pair` | request/구역/AMCL pose와 좌우 사진 한 쌍 저장 |
| `/camera/capture_run/finish` | metadata 확정 후 `READY` 생성 |
| `/camera/capture_run/abort` | metadata를 `aborted`로 확정하고 `READY` 없이 종료 |

정상 호출 순서는 `start -> capture 0회 이상 -> finish`다. 모든 서비스 응답은
필요한 파일과 `metadata.yaml`의 원자적 저장이 끝난 뒤 반환된다. 같은
`mission_id`, `request_id`, finish 또는 abort 요청이 재전송되면 디스크에 저장된
기존 결과를 반환하고 새 run이나 사진을 만들지 않는다. 노드 재시작 후 재요청도
같은 규칙을 적용한다.

상세 필드와 drive_manager 상태 머신 계약은 같은 저장소의
`../inspection_interfaces/CAMERA_SERVICE_INTEGRATION.md`를 따른다. 메시지와
서비스 정의는 형제 패키지인 `inspection_interfaces`에 분리되어 있다.

### 수동 시험용 legacy 촬영 명령

- 구독 토픽: `/camera/capture`
- 메시지 형식: `std_msgs/msg/Bool`
- `data: true`: 두 카메라 촬영 시작
- `data: false`: 무시

한 번의 촬영이 진행 중일 때 새로운 `true` 명령이 들어오면 중복 촬영을
시작하지 않고 요청을 거절한 결과를 발행한다.

### 수동 시험용 legacy run 시작과 종료

- 시작 토픽: `/camera/run/start` (`std_msgs/msg/Empty`)
- 종료 토픽: `/camera/run/finish` (`std_msgs/msg/Empty`)
- 결과 토픽: `/camera/run/result` (`std_msgs/msg/String`, JSON)

`start`는 해당 날짜의 다음 `run_N` 폴더를 만들고, `finish`는
`metadata.yaml`을 `completed` 상태로 갱신한 뒤 `READY` 파일을 만든다.
`/camera/run/result`의 `finish`, `success: true`, `ready: true` 결과에 포함된
`directory`가 Jetson으로 전송할 단위다.

시작 토픽 없이 legacy 촬영 명령이 먼저 들어오면 새 run을 자동 생성한다.
drive_manager는 이 토픽들을 사용하지 않고 위 네 서비스를 사용해야 한다.
서비스가 관리하는 mission run에는 legacy 촬영/종료 토픽을 사용할 수 없다.

### 수동 촬영 결과

- 발행 토픽: `/camera/capture/result`
- 메시지 형식: `std_msgs/msg/String`
- 내용: JSON 문자열

성공 결과 예시는 다음과 같다.

```json
{
  "directory": "/home/user/capture/20260726/run_1",
  "errors": {},
  "files": {
    "left": "/home/user/capture/20260726/run_1/left/154408_1.jpg",
    "right": "/home/user/capture/20260726/run_1/right/154408_1.jpg"
  },
  "metadata_file": "/home/user/capture/20260726/run_1/metadata.yaml",
  "run_id": "run_1",
  "success": true,
  "timestamp": "2026-07-26T15:44:08.000000+09:00"
}
```

한 카메라만 성공할 수도 있다. 이 경우 성공한 사진은 그대로 저장되고,
`success`는 `false`, 실패한 카메라의 원인은 `errors`에 기록된다.

### 캡처 방식

두 카메라는 각각 별도의 OpenCV `VideoCapture` 객체와 작업 스레드를 사용한다.
각 촬영 명령에서 두 RTSP 스트림을 병렬로 열고, 기본값으로 카메라마다 최대
5번 프레임을 읽어 가장 최근에 받은 유효 프레임을 저장한 뒤 연결을 해제한다.

이는 두 촬영을 거의 같은 시점에 시작하는 소프트웨어 동시 캡처다. 두 카메라의
센서 셔터나 RTSP 프레임 타임스탬프를 맞추는 하드웨어 동기화는 아니다.

이미지는 먼저 같은 폴더의 `.tmp.jpg`에 기록한 후 최종 이름으로 교체하므로,
저장 도중 중단되어 불완전한 파일이 최종 JPEG 이름으로 남을 가능성을 줄였다.

### 짐벌 제어 토픽

제어 노드는 카메라별로 다음 토픽을 구독한다.

| 토픽 | 형식 | 의미 |
|---|---|---|
| `/gimbal/left/move` | `std_msgs/msg/String` | Left 한 단계 상/하/좌/우 이동 |
| `/gimbal/right/move` | `std_msgs/msg/String` | Right 한 단계 상/하/좌/우 이동 |
| `/gimbal/left/cmd_vel` | `geometry_msgs/msg/Twist` | Left yaw/pitch 속도 |
| `/gimbal/right/cmd_vel` | `geometry_msgs/msg/Twist` | Right yaw/pitch 속도 |
| `/gimbal/left/zoom` | `std_msgs/msg/Float32` | Left 줌 방향 |
| `/gimbal/right/zoom` | `std_msgs/msg/Float32` | Right 줌 방향 |
| `/gimbal/left/center` | `std_msgs/msg/Empty` | Left 중앙 복귀 |
| `/gimbal/right/center` | `std_msgs/msg/Empty` | Right 중앙 복귀 |
| `/gimbal/control/result` | `std_msgs/msg/String` | UDP 전송 결과 JSON |

일반적인 조작에는 `move` 토픽을 사용한다. 메시지로 `up`, `down`, `left`,
`right`, `stop` 또는 `상`, `하`, `좌`, `우`, `정지`를 한 번 발행하면 된다.
기본 설정에서는 SIYI 속도 40으로 0.15초 동안 움직인 후 자동 정지하므로,
명령 한 번에 카메라가 살짝 움직인다.

`cmd_vel`은 조이스틱처럼 연속 속도 제어가 필요할 때 사용하는 고급 인터페이스다.
사용하는 필드는 다음 두 개이며 다른 필드는 무시한다.

- `angular.z`: yaw 명령, `-1.0 ~ 1.0`
- `angular.y`: pitch 명령, `-1.0 ~ 1.0`
- 범위를 벗어난 값은 `-1.0` 또는 `1.0`으로 제한
- SIYI SDK의 `-100 ~ 100` 회전 속도로 변환

줌 토픽은 값의 부호만 사용한다.

- 양수: 확대
- 음수: 축소
- `0`: 줌 정지

SIYI 회전과 줌 명령은 시작 후 정지 명령을 받아야 멈추는 방식이다. `move`는
`step_duration_sec` 후 정지하고, 연속 `cmd_vel`과 줌은 마지막 명령 후 기본
0.5초가 지나면 정지한다. 오래 움직이려면 `cmd_vel` 또는 `zoom`을 0.5초보다
짧은 간격으로 계속 발행해야 한다.

`/gimbal/control/result`의 `success: true`는 UDP 패킷을 운영체제에 정상
전송했다는 의미다. 현재 회전 명령은 SIYI 응답을 요구하지 않는 형식이므로,
카메라가 실제로 움직였다는 피드백이나 현재 각도를 의미하지는 않는다.

## 4. 저장 위치와 파일명

기본 저장 위치는 **`~/capture`** 다. 과거 오타 경로인 `~/capcture`의 기존
사진은 자동으로 이동하거나 삭제하지 않으며, 새 촬영만 올바른 경로에 저장한다.

```text
~/capture/
└── 20260726/
    ├── run_1/
    │   ├── metadata.yaml
    │   ├── READY
    │   ├── left/
    │   │   └── 153633_1.jpg
    │   └── right/
    │       └── 153633_1.jpg
    └── run_2/
        ├── metadata.yaml
        ├── left/
        └── right/
```

- 날짜 폴더: 로컬 시간 기준 `YYYYMMDD`
- 주행 폴더: 날짜별 `run_1`, `run_2`, ...
- 카메라 폴더: `left`, `right`
- 파일명: `HHMMSS_번호.jpg`
- 한 번의 촬영으로 생성된 Left/Right 사진은 같은 파일명을 사용
- 같은 초에 다시 촬영하거나 한쪽에 같은 이름의 기존 파일이 있으면
  `_2`, `_3` 순서로 증가
- 다음 초에는 다시 `_1`부터 시작
- `metadata.yaml`: 시작/종료 시각, run 상태, 촬영별 성공 여부와 상대 파일 경로
- `READY`: run 종료 처리가 끝났으며 폴더를 전송해도 된다는 표시
- 기존 `run_YYYYMMDD_N` 구조의 사진은 삭제하거나 이동하지 않음
- 오래된 사진을 자동 삭제하는 기능은 없으므로 디스크 용량은 별도로 관리해야 함

`metadata.yaml`은 schema version 2로 mission/map/가변 구역 스냅샷과 각 요청의
`request_id`, `zone_id`, 요청·촬영 시각, 전체 AMCL pose/covariance, 좌우 결과를
기록한다. 파일 경로는 `left/153633_1.jpg` 같은 상대경로이므로 run 폴더 전체를
Jetson으로 이동한 후에도 수정할 필요가 없다. 비정상 종료된 run은
`status: running`이고 `READY`가 없으며, abort된 run은 `status: aborted`이고
역시 `READY`가 없다. Jetson은 `READY`가 있는 run만 추론 대상으로 사용한다.

구역은 1개 이상이면 되고 최대 개수 제한은 없다. 이름에는 영문, 숫자, `_`,
`-`만 사용할 수 있으며 start 요청의 이름·좌표만 metadata에 기록한다.
`CapturePair.zone_id`는 해당 start 요청에 포함됐던 이름 중 하나여야 한다.

파일 날짜와 시간이 중요하므로 다음 명령으로 시스템 시간과 시간대를 확인한다.

```bash
timedatectl
```

## 5. 패키지 위치와 주요 파일

패키지 경로:

```text
~/colcon_ws/src/STELLA_N5_ROS2/gimbal_camera_capture
```

주요 파일은 다음과 같다.

| 파일 | 역할 |
|---|---|
| `gimbal_camera_capture/capture_node.py` | 네 서비스, legacy 토픽, 병렬 RTSP 캡처 |
| `gimbal_camera_capture/control_node.py` | Left/Right 제어 토픽, 자동 정지 watchdog |
| `gimbal_camera_capture/siyi_protocol.py` | SIYI 프레임, CRC16, UDP 전송 구현 |
| `gimbal_camera_capture/storage.py` | 날짜/run/카메라 폴더, 메타데이터와 파일명 관리 |
| `../inspection_interfaces/msg`, `../inspection_interfaces/srv` | 양쪽이 공유하는 메시지와 네 서비스 타입 |
| `launch/camera_capture.launch.py` | 캡처 노드 단독 launch |
| `launch/gimbal_control.launch.py` | 제어 노드 단독 launch |
| `test/test_storage.py` | 저장 경로와 중복 방지 로직 테스트 |
| `test/test_service_contract.py` | start/capture/finish/abort 서비스 계약 테스트 |
| `test/test_siyi_protocol.py` | 공식 패킷 CRC와 UDP 전송 테스트 |
| `../inspection_interfaces/CAMERA_SERVICE_INTEGRATION.md` | drive_manager 호출 순서와 필드 계약 |
| `README.md` | 현재 구성과 운용 문서 |

## 6. 빌드와 테스트

현재 환경은 Ubuntu 24.04와 ROS 2 Jazzy를 사용한다. 주요 런타임 의존성은
`rclpy`, `std_msgs`, `geometry_msgs`, `inspection_interfaces`,
`python3-opencv`, `python3-yaml`, ROS 2 launch 패키지다.

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to gimbal_camera_capture --symlink-install
source install/setup.bash
```

패키지 테스트:

```bash
cd ~/colcon_ws
colcon test --packages-select gimbal_camera_capture \
  --event-handlers console_direct+
colcon test-result \
  --test-result-base build/gimbal_camera_capture --verbose
```

현재 저장 경로, SIYI 공식 패킷/CRC16, 로컬 UDP 전송과 코드 표준 검사를
포함한 20개 테스트가 모두 통과했다. 실제 카메라 두 대에서도 같은 파일명의
1280x720 JPEG 저장과 양쪽 UDP `37260` 펌웨어 응답을 확인했다.

## 7. 실행, 촬영 및 짐벌 제어

이 패키지는 bringup과 별도로 직접 실행한다. 캡처와 제어는 서로 독립된
노드이므로 필요한 launch만 실행하거나 두 launch를 각각 실행한다.

### 사진 캡처 노드

```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch gimbal_camera_capture camera_capture.launch.py
```

서비스와 타입 확인:

```bash
ros2 service list -t | grep -E 'capture_pair|capture_run'
ros2 interface show inspection_interfaces/srv/CapturePair
```

다른 터미널에서 촬영 명령을 한 번 발행한다.

```bash
source ~/colcon_ws/install/setup.bash

# 주행 시작: 새 run_N 생성
ros2 topic pub --once /camera/run/start std_msgs/msg/Empty '{}'

# 주행 중 필요한 횟수만큼 촬영
ros2 topic pub --once /camera/capture std_msgs/msg/Bool '{data: true}'

# 마지막 capture/result 확인 후 주행 종료
ros2 topic pub --once /camera/run/finish std_msgs/msg/Empty '{}'
```

결과 토픽과 생성 파일 확인:

```bash
source ~/colcon_ws/install/setup.bash
ros2 topic echo /camera/capture/result
ros2 topic echo /camera/run/result
find ~/capture -maxdepth 4 -type f | sort
```

노드가 실행 중인지 확인:

```bash
ros2 node list | grep gimbal_camera_capture
ros2 topic info /camera/capture --verbose
```

종료는 launch를 실행한 터미널에서 `Ctrl+C`를 누른다.

### 짐벌 제어 노드

```bash
cd ~/colcon_ws
source install/setup.bash
ros2 launch gimbal_camera_capture gimbal_control.launch.py
```

처음 시험할 때는 주변에 카메라가 부딪힐 물체가 없는지 확인한다. 다음 명령은
각 카메라를 해당 방향으로 한 단계만 움직이고 자동 정지한다.

```bash
# Left 카메라
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: 'up'}"
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: 'down'}"
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: 'left'}"
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: 'right'}"

# 한글 명령도 동일하게 지원
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: '상'}"
```

Right 카메라도 토픽 이름만 바꾼다.

```bash
ros2 topic pub --once /gimbal/right/move std_msgs/msg/String "{data: 'up'}"
ros2 topic pub --once /gimbal/right/move std_msgs/msg/String "{data: 'left'}"
```

명시적인 즉시 정지:

```bash
ros2 topic pub --once /gimbal/left/move std_msgs/msg/String "{data: 'stop'}"
ros2 topic pub --once /gimbal/right/move std_msgs/msg/String "{data: 'stop'}"
```

`step_speed`와 `step_duration_sec`를 바꾸면 한 번에 움직이는 정도를 조절할 수
있다. 더 작게 움직이려면 속도나 시간을 낮춘다.

```bash
ros2 launch gimbal_camera_capture gimbal_control.launch.py \
  step_speed:=25 step_duration_sec:=0.10
```

연속 속도 제어가 필요할 때만 `cmd_vel`을 반복 발행한다. 아래 명령은 Left에
yaw `-0.15`, pitch `0.10`을 10 Hz로 보낸다. `Ctrl+C` 후 0.5초 안에
자동 정지한다.

```bash
ros2 topic pub -r 10 /gimbal/left/cmd_vel geometry_msgs/msg/Twist \
  '{angular: {y: 0.10, z: -0.15}}'
```

중앙 복귀:

```bash
ros2 topic pub --once /gimbal/left/center std_msgs/msg/Empty '{}'
ros2 topic pub --once /gimbal/right/center std_msgs/msg/Empty '{}'
```

줌 확대/축소/정지:

```bash
# Left 확대: Ctrl+C 후 0.5초 안에 자동 정지
ros2 topic pub -r 5 /gimbal/left/zoom std_msgs/msg/Float32 '{data: 1.0}'

# Right 축소
ros2 topic pub -r 5 /gimbal/right/zoom std_msgs/msg/Float32 '{data: -1.0}'

# 명시적인 줌 정지
ros2 topic pub --once /gimbal/left/zoom std_msgs/msg/Float32 '{data: 0.0}'
ros2 topic pub --once /gimbal/right/zoom std_msgs/msg/Float32 '{data: 0.0}'
```

제어 패킷 전송 결과 확인:

```bash
ros2 topic echo /gimbal/control/result
```

## 8. Launch 파라미터

### 캡처 노드 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `trigger_topic` | `/camera/capture` | 촬영 명령 토픽 |
| `result_topic` | `/camera/capture/result` | JSON 결과 토픽 |
| `run_start_topic` | `/camera/run/start` | 새 주행 시작 토픽 |
| `run_finish_topic` | `/camera/run/finish` | 현재 주행 종료 토픽 |
| `run_result_topic` | `/camera/run/result` | 주행 시작/종료 JSON 결과 토픽 |
| `capture_run_start_service` | `/camera/capture_run/start` | mission run 시작 서비스 |
| `capture_pair_service` | `/camera/capture_pair` | 동기 좌우 촬영 서비스 |
| `capture_run_finish_service` | `/camera/capture_run/finish` | run 완료/READY 서비스 |
| `capture_run_abort_service` | `/camera/capture_run/abort` | run 중단 서비스 |
| `output_directory` | `~/capture` | 사진 저장 최상위 경로 |
| `camera_1_url` | `rtsp://192.168.144.25:8554/main.264` | Left RTSP 주소 |
| `camera_2_url` | `rtsp://192.168.144.26:8554/main.264` | Right RTSP 주소 |
| `open_timeout_ms` | `5000` | 스트림 열기 제한 시간(ms) |
| `read_timeout_ms` | `5000` | 프레임 읽기 제한 시간(ms) |
| `frame_read_attempts` | `5` | 최신 프레임 확보를 위한 읽기 횟수 |
| `jpeg_quality` | `95` | JPEG 품질(0~100) |

drive_manager의 `capture_service_timeout_sec`는 정상 촬영 시간뿐 아니라 카메라
장애 시 RTSP 제한 시간도 포함해야 한다. 현재 카메라 노드의 최악 지연은
`open_timeout_ms + frame_read_attempts * read_timeout_ms`에 근접할 수 있으므로,
현재 기본값의 이론상 상한은 약 30초다. 최초 연동 시 drive_manager timeout을
35초 이상으로 두는 것이 안전하다. 5초 설정을 유지하려면 현장 측정으로 응답
시간을 검증하고 `open_timeout_ms`와 `read_timeout_ms`를 함께 낮춰야 한다.

촬영 실패를 metadata에 기록하되 주행은 계속하려면 `drive_manager`가 설치된
컴퓨터에서 다음 정책을 사용한다. 이 로봇 워크스페이스에는 `drive_manager`가
없으므로 해당 값은 카메라 서버가 아니라 drive_manager 설정에서 변경해야 한다.

```yaml
capture_failure_stops_mission: false
capture_service_timeout_sec: 35.0
capture_finish_wait_timeout_sec: 40.0
```

예를 들어 저장 경로와 토픽을 바꾸려면 다음과 같이 실행한다.

```bash
ros2 launch gimbal_camera_capture camera_capture.launch.py \
  trigger_topic:=/my/capture_command \
  output_directory:=~/capture \
  jpeg_quality:=90
```

파라미터가 적용되었는지 실행 중에 확인할 수도 있다.

```bash
ros2 param list /gimbal_camera_capture
ros2 param get /gimbal_camera_capture camera_1_url
ros2 param get /gimbal_camera_capture camera_2_url
```

### 제어 노드 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `left_ip` | `192.168.144.25` | Left 제어 IP |
| `left_port` | `37260` | Left 실제 SIYI UDP 포트 |
| `left_bind_address` | `192.168.144.10` | Left 전용 로컬 출발지 IP |
| `left_yaw_direction` | `1` | Left yaw 방향 보정(`1`/`-1`) |
| `left_pitch_direction` | `1` | Left pitch 방향 보정(`1`/`-1`) |
| `right_ip` | `192.168.144.26` | Right 제어 IP |
| `right_port` | `37260` | Right 실제 SIYI UDP 포트 |
| `right_bind_address` | `192.168.144.11` | Right 전용 로컬 출발지 IP |
| `right_yaw_direction` | `1` | Right yaw 방향 보정(`1`/`-1`) |
| `right_pitch_direction` | `1` | Right pitch 방향 보정(`1`/`-1`) |
| `command_timeout_sec` | `0.5` | 명령 중단 후 자동 정지 시간 |
| `step_duration_sec` | `0.15` | `move` 한 번의 이동 시간(초) |
| `step_speed` | `40` | `move` 이동 속도(1~100) |
| `result_topic` | `/gimbal/control/result` | 제어 결과 JSON 토픽 |

카메라 설치 방향 때문에 움직임이 반대라면 해당 축의 direction만 `-1`로
실행한다.

```bash
ros2 launch gimbal_camera_capture gimbal_control.launch.py \
  right_yaw_direction:=-1 \
  right_pitch_direction:=-1
```

현재 값 확인:

```bash
ros2 param list /gimbal_control
ros2 param get /gimbal_control command_timeout_sec
ros2 param get /gimbal_control right_port
```

## 9. 카메라 네트워크 설정

현재 기기의 재현 가능한 설정 스크립트는 다음 위치에 있다.

```text
/home/user/configure-camera-router.sh
```

카메라와 USB-LAN을 모두 연결한 뒤 실행한다.

```bash
/home/user/configure-camera-router.sh
```

스크립트 자체가 필요할 때 `sudo`를 요청한다. 이 스크립트는 다음 작업을 한다.

1. `eth0`과 USB-LAN 장치/MAC이 예상과 같은지 확인
2. `/etc/netplan/99-camera-eth0.yaml`에 두 `/32` 주소와 호스트 경로 설정
3. 일반 DHCP 프로필인 `Wired connection 1`이 있으면 자동 연결 비활성화
4. `/etc/sysctl.d/90-camera-router.conf`에 IPv4 forwarding 영구 설정
5. Wi-Fi에서 카메라로 전달하는 iptables DNAT, FORWARD, MASQUERADE 설정
6. `iptables-persistent`를 이용해 재부팅 후에도 방화벽 규칙 유지
7. 마지막에 주소, 경로, forwarding, NAT, Cyclone DDS 인터페이스 출력

USB-LAN 어댑터를 교체하면 장치명과 MAC 주소가 달라질 수 있다. 현재 스크립트는
안전을 위해 아래 값을 정확히 확인하고, 다르면 설정을 중단한다.

```text
인터페이스: enx00e04c3628a8
MAC 주소:   00:e0:4c:36:28:a8
```

어댑터 교체 시에는 실제 값을 `ip link`로 확인한 뒤 스크립트의 `CAMERA2_IF`,
`CAMERA2_MAC`과 Netplan의 `macaddress`를 함께 수정해야 한다.

### 네트워크 상태 확인

```bash
ip -4 -br address
ip -4 route get 192.168.144.25
ip -4 route get 192.168.144.26
```

정상이라면 핵심 출력은 다음과 같아야 한다.

```text
eth0             UP  192.168.144.10/32
wlan0            UP  192.168.0.15/24
enx00e04c3628a8  UP  192.168.144.11/32

192.168.144.25 dev eth0 src 192.168.144.10
192.168.144.26 dev enx00e04c3628a8 src 192.168.144.11
```

`wlan0` 주소는 공유기의 DHCP에 따라 바뀔 수 있다. 공유기 포트포워딩을
사용한다면 라즈베리파이에 DHCP 예약을 설정하는 것이 좋다.

RTSP 스트림 자체를 짧게 확인하려면 다음 명령을 사용할 수 있다.

```bash
timeout 15 ffprobe -v error -rtsp_transport tcp \
  -show_entries stream=codec_name,width,height \
  rtsp://192.168.144.25:8554/main.264

timeout 15 ffprobe -v error -rtsp_transport tcp \
  -show_entries stream=codec_name,width,height \
  rtsp://192.168.144.26:8554/main.264
```

## 10. Wi-Fi/외부 네트워크 포트포워딩

ROS 캡처 노드를 라즈베리파이에서만 사용할 경우 포트포워딩은 필요 없다.
다른 컴퓨터에서 두 RTSP 스트림이나 제어 포트에 접근해야 할 때만 필요하다.

라즈베리파이의 iptables 매핑은 다음과 같다.

| Wi-Fi로 들어오는 포트 | 카메라 목적지 | 용도 |
|---|---|---|
| TCP+UDP `8554` | `192.168.144.25:8554` | Left RTSP |
| TCP+UDP `8555` | `192.168.144.26:8554` | Right RTSP |
| UDP `37260` | `192.168.144.25:37260` | Left 제어 |
| UDP `37261` | `192.168.144.26:37260` | Right 제어 |

상위 공유기에서는 선택한 외부 포트를 라즈베리파이의 현재 Wi-Fi 주소와 위의
내부 포트로 전달한다. 외부 포트를 `16054/16055`로 쓰고 싶다면 예를 들어
`외부 16054 → Pi 8554`, `외부 16055 → Pi 8555`로 설정하고, 외부 클라이언트는
`16054/16055`를 사용해야 한다.

공인 인터넷에 RTSP와 제어 포트를 직접 노출하면 인증되지 않은 접근 위험이
있다. 가능한 경우 VPN을 사용하고, 포트포워딩이 꼭 필요하면 공유기와
방화벽에서 허용할 출발지 IP를 제한한다.

기존 규칙이 Right 입력 `37261`을 `.26:37261`로 전달하고 있다면 카메라가
응답하지 않는다. 수정된 `/home/user/configure-camera-router.sh`를 다시
실행해야 `37261 → .26:37260` 규칙이 실제 iptables에도 반영된다. 로컬 ROS
제어 노드는 `.26:37260`에 직접 접속하므로 이 외부 NAT 규칙과 무관하다.

## 11. Cyclone DDS는 wlan0만 사용

카메라용 Ethernet이 추가되면 Cyclone DDS가 카메라 인터페이스를 ROS 2
통신용으로 선택하거나 멀티캐스트를 내보낼 수 있다. 이를 막기 위해 현재
`/etc/cyclonedds.xml`은 `wlan0`만 명시한다.

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="wlan0" multicast="true"/>
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

`~/.bashrc`에는 다음 환경 변수가 설정되어 있다.

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
```

새 터미널에서 적용 상태를 확인한다.

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION \
  ROS_AUTOMATIC_DISCOVERY_RANGE CYCLONEDDS_URI
grep -F 'NetworkInterface name="wlan0"' /etc/cyclonedds.xml
```

설정을 변경한 직후 기존 ROS 2 daemon이 이전 환경을 유지하는 것 같으면 새
터미널에서 다음 명령을 실행한다.

```bash
ros2 daemon stop
ros2 daemon start
```

다른 ROS 2 컴퓨터와 토픽을 주고받으려면 최소한 `ROS_DOMAIN_ID`가 같고,
공유기/Wi-Fi가 ROS 2 discovery 트래픽을 차단하지 않아야 한다. `eth0`과
`enx00e04c3628a8`은 카메라 전용이며 DDS 통신에 사용하지 않는다.

## 12. 자주 발생한 오류와 확인 순서

### `Error: unknown connection 'Wired connection 1'`

`Wired connection 1`은 인터페이스 이름이 아니라 NetworkManager의 연결
프로필 이름이다. 기기마다 이름이 다르거나 프로필이 없을 수 있어 발생한다.
현재 구성은 Netplan에서 `eth0`과 USB 어댑터 MAC을 직접 지정하므로 예전
`nmcli connection modify "Wired connection 1" ...` 명령을 다시 실행할
필요가 없다.

프로필 이름이 필요할 때만 다음으로 실제 이름을 확인한다.

```bash
nmcli connection show
nmcli device status
```

### `RTNETLINK answers: Network is unreachable`

대상 카메라의 `/32` 경로가 없거나, USB-LAN 장치가 연결되지 않았거나,
인터페이스 설정이 적용되지 않은 경우다.

```bash
ip -br link
ip -4 route get 192.168.144.25
ip -4 route get 192.168.144.26
/home/user/configure-camera-router.sh
```

두 카메라 케이블과 USB-LAN을 연결한 상태에서 다시 확인한다.

### `NetworkManager is not running` 또는 networkd 관련 오류

예전 설정 스크립트가 실제 renderer와 다른 서비스를 직접 재시작하면서 발생한
오류다. 현재 스크립트는 `netplan apply` 후 NetworkManager가 준비될 때까지
확인한다. 현재 스크립트를 사용하고, 그래도 실패하면 다음을 확인한다.

```bash
systemctl status NetworkManager --no-pager
networkctl status 2>/dev/null || true
sudo netplan generate
```

### `Temporary failure resolving ...`

APT 저장소의 DNS 이름을 찾지 못한 것으로, 카메라 Ethernet 링크 오류와는
별개다. `wlan0`의 인터넷 연결, 기본 경로와 DNS를 확인한다.

```bash
ip route show default
resolvectl status
ping -c 1 192.168.0.1
```

이미 `iptables-persistent`와 패키지 의존성이 설치되어 있다면 카메라 캡처
자체는 인터넷 없이도 동작한다. 새 패키지를 설치할 때는 인터넷/DNS가 필요하다.

### 유선 인터페이스가 `DOWN` 또는 `NO-CARRIER`

카메라 전원, 랜 케이블 또는 USB-LAN이 연결되지 않으면 발생한다. 스위칭 허브를
사용하지 않는 현재 구성에서는 Left 카메라가 `eth0`, Right 카메라가
USB-LAN에 각각 직결되어 있어야 한다.

```bash
ip -br link show eth0
ip -br link show enx00e04c3628a8
```

### RTSP 연결 또는 프레임 읽기 실패

다음 순서로 확인한다.

1. `ip route get` 결과가 각각 올바른 인터페이스인지 확인
2. 두 인터페이스가 `UP`이고 carrier가 있는지 확인
3. `ffprobe`로 Left와 Right를 각각 확인
4. 두 `ffprobe`를 동시에 실행하여 동시 스트리밍 확인
5. 노드의 `/camera/capture/result`에서 `errors` 확인
6. 필요하면 `open_timeout_ms`, `read_timeout_ms`, `frame_read_attempts` 증가

FFmpeg가 드물게 HEVC 참조 프레임 관련 경고를 출력할 수 있다. JPEG 두 장이
정상 저장되고 결과가 성공이라면 일시적인 디코더 경고일 수 있다. 프레임 실패가
반복되면 케이블, 전원, 패킷 손실과 RTSP 스트림 상태를 점검한다.

### 제어 토픽은 보이지만 카메라가 움직이지 않음

1. 제어 노드가 실행 중인지 `ros2 node list | grep gimbal_control`로 확인
2. `/gimbal/control/result`에 UDP 전송 오류가 있는지 확인
3. Left는 `.25:37260`, Right는 `.26:37260`인지 확인
4. `.25`와 `.26`의 `/32` 경로가 각각 올바른 인터페이스인지 확인
5. `move` 메시지가 지원 값인지, 또는 `cmd_vel`이 5~10 Hz인지 확인
6. 설치 방향이 반대면 yaw/pitch direction 파라미터를 `-1`로 변경

`move`는 원래 한 번 발행하도록 설계되었고 기본 0.15초 후 정지한다.
`cmd_vel`을 `--once`로 한 번만 보내도 watchdog 때문에 약 0.5초 후 자동
정지한다. 장시간 움직이려면 `cmd_vel` 반복 발행이 필요하다.

### 패키지를 찾을 수 없음

빌드 후 현재 터미널에서 workspace를 source하지 않은 경우가 대부분이다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 pkg prefix gimbal_camera_capture
```

## 13. 운용 체크리스트

촬영 전 최소 확인 사항은 다음과 같다.

- Left 카메라가 `eth0`, Right 카메라가 USB-LAN에 연결되어 있는가
- `.25` 경로가 `eth0`, `.26` 경로가 `enx00e04c3628a8`로 나가는가
- `CYCLONEDDS_URI`가 `/etc/cyclonedds.xml`을 가리키는가
- Cyclone DDS 설정이 `wlan0`만 지정하는가
- `~/capture`에 충분한 디스크 공간이 있는가
- 시스템 날짜, 시간과 `Asia/Seoul` 시간대가 올바른가
- 필요한 독립 launch가 실행 중이고 캡처/제어 구독자가 보이는가
- 처음 움직이기 전에 짐벌 주변에 충돌할 물체나 케이블이 없는가

현재 구조에서 카메라 Ethernet은 영상 전용, Wi-Fi는 ROS 2와 상위 네트워크
전용이다. 이 역할을 유지하면 동일 MAC 카메라 문제와 Cyclone DDS의 잘못된
인터페이스 선택 문제를 함께 피할 수 있다.

## 14. SIYI 프로토콜 참고 자료

제어 구현은 SIYI 공식 SDK 프레임 형식과 A8 mini 명령을 기준으로 한다.

- [A8 mini 공식 다운로드 페이지](https://www.siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/download/)
- [A8 mini User Manual v1.5](https://siyi.biz/siyi_file/A8%20mini/A8%20mini%20User%20Manual%20v1.5.pdf)
- [SIYI Gimbal Camera External SDK Protocol](https://siyi.biz/siyi_file/A8%20mini/SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log%20V0.1.1.pdf)

현재 구현에서 사용하는 명령 ID는 회전 `0x07`, 중앙 복귀 `0x08`, 수동 줌
`0x05`이며, CRC는 초기값 0의 CRC16-XMODEM 다.

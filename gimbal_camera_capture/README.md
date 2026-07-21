# gimbal_camera_capture

ROS 2 토픽 명령을 받으면 SIYI A8 mini 두 대의 RTSP 스트림에서 한 장씩
캡처하여 저장하는 독립 패키지다. `stella_bringup`에는 등록하지 않는다.

## 기본 인터페이스

- 명령 구독: `/camera/capture` (`std_msgs/msg/Bool`)
  - `data: true`: 촬영 시작
  - `data: false`: 무시
- 결과 발행: `/camera/capture/result` (`std_msgs/msg/String`)
  - JSON 형식으로 성공 여부, 저장 파일, 오류를 발행
- 기본 저장 위치: `~/capcture`
- 카메라 1: `rtsp://192.168.144.25:8554/main.264`
- 카메라 2: `rtsp://192.168.144.26:8554/main.264`

같은 날짜와 같은 시간대에 촬영한 사진은 같은 폴더에 저장된다. 예를 들어
그날 첫 촬영 시간대는 `run_20260721_1`, 다음 촬영 시간대는
`run_20260721_2`가 된다. 노드를 재시작해도 숨김 마커 파일을 확인하여 같은
시간대 폴더를 다시 사용한다.

## 빌드와 실행

```bash
cd ~/colcon_ws
colcon build --packages-select gimbal_camera_capture --symlink-install
source install/setup.bash
ros2 launch gimbal_camera_capture camera_capture.launch.py
```

다른 터미널에서 촬영 명령을 한 번 발행한다.

```bash
source ~/colcon_ws/install/setup.bash
ros2 topic pub --once /camera/capture std_msgs/msg/Bool '{data: true}'
```

촬영 결과 확인:

```bash
ros2 topic echo /camera/capture/result
find ~/capcture -maxdepth 2 -type f
```

토픽이나 저장 위치를 바꾸려면 launch 인자를 사용한다.

```bash
ros2 launch gimbal_camera_capture camera_capture.launch.py \
  trigger_topic:=/my/capture_command \
  output_directory:=~/capture
```

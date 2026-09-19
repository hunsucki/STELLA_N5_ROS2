# 도킹 정확도 개선 검토 — 2026-09-07

> 후속 수정(2026-09-09): 설치된 `image_proc`의 종료 crash가 재현되어 보정 노드를 `docking/image_rectifier`로 교체했다. 도킹 서버도 실행별 namespace로 분리했다. 아래는 9월 7일 검토 기록이며, 최신 변경·검증은 [즉시 실패 분석](FAILURE_ANALYSIS.md)에 정리했다.

이번 수정은 위치 추정의 시각·영상 조건을 맞추고, 정지 완료를 너무 일찍 판정하는 문제를 해결한다. 실기 도킹 오차가 몇 mm 줄었는지는 아직 측정하지 않았다. 기존 LiDAR 후면 정렬·후진 제어의 게인, 최종 간격 20 ± 5 mm, 이동 한도는 유지했다.

## 확인한 문제와 반영한 수정

| 문제 | 정확도에 미치는 영향 | 반영한 수정 |
|---|---|---|
| bridge가 TF의 원래 시각 대신 `now - 0.1 s`를 사용 | 다른 시각의 로봇 pose로 관측을 odom에 변환하고, 태그 소실 후에도 캐시를 새 검출처럼 발행 | TF header 보존; 0.5초보다 오래되거나 미래·중복·역순인 TF 발행 거부 |
| AprilTag `image_rect`에 원본 `image_raw`를 연결 | 렌즈 왜곡이 있을 때 보정 영상용 카메라 행렬과 실제 픽셀 좌표가 불일치 | `image_proc::RectifyNode`를 같은 container에 추가하고 `/apriltag/image_rect`로 연결 |
| 회전·정밀 보정의 안정 횟수를 루프마다 증가 | `spin_once`가 다른 콜백 때문에 즉시 반환하면 한 odom만으로도 여러 번 안정하다고 판단 | 고유 timestamp의 odom만 sequence 증가; 서로 다른 odom에서 완료 확인 |
| 정밀 보정은 위치·yaw만 맞으면 성공 | 목표 범위를 지나며 감속 중인 순간에도 회전 단계로 전환 가능 | 선속도 0.005 m/s 이하와 기존 정지 각속도 조건도 함께 확인 |
| odom·IMU·wheel yaw는 수신 시각만 검사 | 전송 지연·중복 메시지가 실제보다 최신으로 판단되고, 역순 IMU가 적분 기준을 되돌릴 수 있음 | header 시각과 수신 경과 시간 모두 검사; 중복·역순 데이터가 수신 시각을 갱신하지 않음 |
| LiDAR header는 수신 콜백에서만 검사 | 수신할 때 이미 오래된 scan이 이후 읽을 때 최대 수신 유효 시간만큼 더 사용될 수 있음 | snapshot을 읽을 때도 header 유효 시간 검사 |
| 정밀 목표의 잘못된 quaternion/odom frame을 그대로 해석 | 영 quaternion을 yaw=0으로 해석하거나 다른 기준점의 pose를 로봇 중심으로 사용 | 목표 quaternion 검증·정규화; odom의 frame, pose, 속도 검증 |

`/odom`의 실제 child frame은 `base_footprint`다. 저장소의 세 URDF 모두 `base_footprint → base_link`가 z 방향 0.071 m의 고정 변환이므로 평면상의 x/y/yaw는 같다. 코드는 이 등가성을 TF로 확인해 수용한다. x/y 오프셋 또는 회전이 있는 다른 child frame은 거부한다. `fixed_frame` 역시 odom header frame과 일치해야 한다.

AprilTag는 보정 영상과 `CameraInfo.P`를 사용하고, TF header에 영상 시각을 넣는다고 명시한다. [AprilTag ROS 공식 설명](https://github.com/christianrauch/apriltag_ros#topics)

Nav2 Jazzy의 `SimpleChargingDock`은 외부 pose header로 검출 timeout을 검사하고, **같은 header 시각**으로 fixed frame 변환을 수행한다. 따라서 bridge의 시각 변경은 변환 결과와 소실 감지 모두에 영향을 준다. [Nav2 Jazzy 구현](https://api.nav2.org/nav2-jazzy/html/simple__charging__dock_8cpp_source.html)

`RectifyNode`는 `CameraInfo`로 영상을 보정하고 원본 header를 유지한다. 왜곡 계수가 모두 0이면 원본을 그대로 전달한다. [image_proc Jazzy 구현](https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp)

## 설정과 호환성

- `config/docking.yaml`의 `apriltag_to_pose_bridge` 블록에서 `transform_max_age_sec: 0.50`, `transform_future_tolerance_sec: 0.05`를 조절한다. ManagedStack이 bridge에도 같은 params file을 전달한다.
- 기존 `pose_stamp_delay_sec`는 기본 0이며, 값이 지정돼도 경고 후 무시한다. 관측 시각을 인위적으로 변경하는 동작은 재활성화하지 않는다.
- `tag_refinement_stationary_linear_speed: 0.005`는 정밀 보정의 정지 선속도 한도다. 정지 각속도는 기존 `imu_stationary_yaw_rate`를 사용한다.
- 새 `motion_sensor_future_tolerance_sec`의 기본값은 0.05초다. odom 0.50초, IMU 0.25초, wheel yaw 0.50초의 기존 age 설정은 이제 관측 header에도 적용한다.
- 태그 앞 pose 대기·검증은 기존 `tag_front_pose_max_age_sec`(0.30초), 정밀 목표 획득은 `tag_refinement_target_max_age_sec`(1.5초)를 관측 시각과 수신 시각에 모두 적용한다. 정밀 목표는 획득 후 odom 좌표계에 고정한다.
- 센서 시계가 뒤로 리셋되면 역순 메시지가 거부된다. 시계를 일치시키고 도킹 노드를 새로 시작해야 한다.
- 새 런타임 의존성은 `image_proc`다. 현재 Jazzy 환경에는 설치되어 있다.

실제 영상 처리 지연이 한도를 넘는다면 먼저 카메라·호스트 시계와 처리 속도를 확인한다. 시각을 다시 붙여 지연을 숨기면 fixed-frame 변환 오차가 재발한다.

## 검증

- 변경 전: 기존 테스트 61개 통과.
- 변경 후: **91개 테스트 통과, 오류·실패·건너뛰기 0개**. 패키지 회귀 테스트에 TF 소실·시각 보존, 중복/지연 센서 입력, odom frame, 정밀 보정 실제 루프의 정지 조건을 추가했다.
- `colcon build --packages-select docking --symlink-install`로 패키지 빌드 확인.
- `test/check_apriltag_pipeline.py`에서 별도 DDS domain 217로 합성 영상과 CameraInfo를 공급했다. 실제 RectifyNode와 AprilTagNode를 로드해 보정 영상 6장, 원래 촬영 시각을 유지한 bridge pose 5개를 확인했다. 영상 공급 중단 후 캐시 pose를 재발행하지 않았다.
- 합성 영상 검증은 처리 경로·QoS·시각 전달을 확인한다. 실물 카메라의 보정 오차, 태그 자세 정확도, 실제 CPU 부하와 도킹 성공률을 측정한 결과는 아니다.

재검증 명령:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/colcon_ws
colcon build --packages-select docking --symlink-install
colcon test --packages-select docking --event-handlers console_direct+
colcon test-result --test-result-base build/docking --verbose
source install/setup.bash
cd src/STELLA_N5_ROS2/docking
python3 test/check_apriltag_pipeline.py
```

## 정확도를 더 높일 다음 실측 과제

1. **카메라·태그·장착 TF 보정.** 실제 태그 검출 경계의 한 변이 설정값 154 mm와 일치하는지 확인하고, 카메라와 차체 사이 x/y/yaw를 실측한다. 태그 크기 오차 1 mm는 약 0.65%의 비율 오차이므로 이상적인 핀홀 모델에서 0.95 m 거리 추정에 약 6.2 mm 영향을 줄 수 있다. 이는 모델 계산이며 실제 측정값은 아니다.
2. **태그 yaw에 의해 움직이는 가상 목표점 확인.** 목표점은 태그에서 0.95 m 떨어져 있으므로 태그 yaw 편향 1°만으로 `0.95 × sin(1°) ≈ 16.6 mm`의 횡방향 목표 편향이 생길 수 있다. 더 큰 태그/복수 태그 보드 또는 정지 관측의 안정된 자세 추정이 후보지만, 실제 관측 분산과 설치 위치를 확인한 후 적용해야 한다.
3. **횡방향 오차를 별도로 측정.** 후면의 직선 하나는 법선 방향을 알려주지만 그 평면을 따라 어디가 도크 중앙인지는 알려주지 못한다. 현재 태그 보정 허용 횡오차는 25 mm이고, 후진 중에는 두 평행 가이드 레일이 모두 검출될 때 중앙 추종을 적용한다. 한쪽 레일만 보이면 중앙 오프셋을 만들지 않으므로 태그 정렬 정확도도 계속 중요하다.
4. **후진 경로의 yaw 오차 누적 확인.** 0.5 m를 2° 틀어진 방향으로 후진하면 단순 기하학적으로 약 17.5 mm의 추가 횡변위가 가능하다. 회전 후·후진 중·최종의 yaw와 횡위치를 따로 기록해 태그 오차와 구동 오차를 구분한다.
5. **정지 거리와 검출 속도를 측정한 후 허용값 조정.** 최종 LiDAR 간격 20 ± 5 mm와 최소 후진 속도 0.015 m/s는 그대로다. 센서 지연·거리 잡음·차체 제동 거리를 측정한 뒤 정지 여유를 조절한다. `detector.decimate`를 2에서 1로 낮추는 실험도 영상 처리 지연을 함께 비교해야 한다.

반복 시험에서는 같은 시작 위치·조명·바닥 조건으로 변경 전후 각각 최소 20회씩, 최종 횡오차(mm), yaw 오차(°), 후방 간격(mm), 충전 성공 여부, 단계별 실패 횟수를 기록한다. odom 자체를 최종 위치의 정답으로 쓰지 말고 스테이션 기준 외부 실측으로 확인한다. 현재 `development_test_mode: true`이므로 종료 코드 0은 충전 성공률 측정값이 아니다.

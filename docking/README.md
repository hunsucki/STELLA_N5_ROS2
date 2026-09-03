# STELLA N5 AprilTag 회전 후진 도킹

이 문서는 `docking` 패키지의 **현재 작업 트리(2026-09-02)** 를 기준으로 작성한 설계·운용 문서다. 현재 도킹 절차가 어떤 센서와 기준으로 움직이는지, 초기 Git 버전에서 무엇이 달라졌는지, 반복 시험 중 어떤 실패가 있었고 어떻게 방어했는지, 설정을 어디서 되돌리거나 조정해야 하는지를 한곳에 정리한다.

> 주의: 이 패키지는 실제 로봇에 `cmd_vel`을 발행한다. 시험 전 로봇 주변, 180도 회전 반경, 도킹 스테이션 내부를 비우고 비상 정지 수단을 준비한다. 특히 Nav2 접근 컨트롤러의 `use_collision_detection`은 현재 `false`다.

## 1. 현재 목표와 전체 동작

실행 명령은 다음과 같다.

```bash
ros2 run docking dock_turn_backup
```

현재 노드는 아래 순서로 움직인다.

```text
센서·TF·도킹 서버 준비
        ↓
AprilTag 감지 및 Nav2 DockRobot 전방 접근
        ↓
선택 기능: 필터링된 태그 목표로 저속 정밀 위치·각도 보정
        ↓
선택 기능: 태그 전방 위치 검증 (현재 꺼짐)
        ↓
선택 기능: 회전 전 시간 기반 저속 직진 (현재 꺼짐)
        ↓
시작 odom yaw + 180°의 절대 yaw 목표로 제자리 회전
        ↓
후방 LiDAR RANSAC으로 도킹 스테이션 후면 평면과 수직 정렬
        ↓
후방 LiDAR 거리로 감속하며 후진
  └─ 멀리서는 후면 평면 RANSAC으로 매우 약한 각도 보정
  └─ 후방 간격 10 cm 이하에서는 각도 보정을 끄고 직선 후진
        ↓
개발 모드: 기하학적 도킹 위치 도달 시 성공
실사용 모드: 충전기 접촉 및 충전 전류까지 확인 후 성공
```

핵심은 센서별 역할을 분리한 것이다.

| 단계 | 주 기준 | 보조 기준 | 현재 역할 |
|---|---|---|---|
| 태그 접근 | AprilTag TF, Nav2 `/dock_pose` | odom/TF | 도킹 스테이션 앞의 가상 목표점으로 이동 |
| 태그 정밀 보정 | 정지 직전의 필터링된 `/dock_pose` | odom | 목표를 고정한 뒤 작은 위치·yaw 오차만 저속 보정 |
| 180도 회전 | `/odom`의 절대 yaw | `/imu/data` 또는 odom 각속도 | 시작 yaw에 정확히 180도를 더한 목표로 회전하고 정지까지 확인 |
| 회전 후 정렬 | 후방 `/scan_2`의 지배 평면 | odom, IMU 각속도 | 스테이션 후면 패널의 법선과 로봇 후방 축 정렬 |
| 후진 | 후방 LiDAR의 차체 기준 간격 | LiDAR RANSAC, IMU 적분, 휠 yaw | 거리 기반 감속·정지와 약한 방향 보정 |
| 최종 성공 | 개발 모드에서는 LiDAR 거리 | 실사용 모드에서는 충전 상태 | 단순 시간 또는 odom 이동 거리만으로 성공시키지 않음 |

`/imu/data`의 orientation을 별도 절대 yaw 목표로 사용하지는 않는다. 180도 목표 자체는 `/odom` quaternion에서 읽은 시작 yaw로 계산한다. IMU는 회전이 실제로 멈췄는지 확인하고, 후진 중 짧은 구간의 yaw 변화를 추적하는 데 사용한다. 현재 wheel odometry가 IMU를 융합한다면 그 결과는 이미 `/odom` yaw에 간접 반영된다.

## 2. 초기 Git 버전과 비교한 핵심 변경 사항

이 절은 현재 `git diff`의 기준인 저장소 `HEAD`와 현재 작업 트리를 비교한다. 아직 커밋되지 않은 변경도 포함한다.

### 2.1 설정 변경

| 항목 | 기존 | 현재 | 이유 |
|---|---:|---:|---|
| `external_detection_translation_x` | `-0.80 m` | `-0.95 m` | 태그 정밀 보정 후 회전 위치가 스테이션에 너무 가까워지는 문제를 줄이기 위해 전방 목표점을 약 15 cm 더 멀리 배치 |
| `dock_turn_backup` 전용 YAML 블록 | 없음 | 추가됨 | 정밀 보정, RANSAC 재획득, 선택 직진, 최종 간격을 명시적으로 관리 |
| `backup_target_rear_clearance` | 코드 기본 `0.010 m` | YAML에서 `0.020 m` | 물리적으로 성공한 약 2.3 cm 위치를 소프트웨어 성공 범위에 포함 |
| `backup_clearance_tolerance` | `0.005 m` | `0.005 m` | 최종 허용 범위를 1.5~2.5 cm로 유지 |
| 회전 전 직진 | 기능 없음 | 파라미터화 후 현재 `false` | 임의의 시간 직진이 태그 보정 결과를 망가뜨리거나 회전 여유를 줄이지 않도록 기본 비활성화 |

`staging_x_offset: -0.80`은 그대로지만, 실행 코드가 `navigate_to_staging_pose: false`로 DockRobot goal을 보내므로 현재 절차에서는 Nav2 사전 staging 이동에 사용되지 않는다. 실제 태그 기반 접근 목표에 직접 영향을 주는 값은 `external_detection_translation_x: -0.95`다.

### 2.2 태그 접근 이후 정밀 보정 추가

기존에는 Nav2 DockRobot action이 성공하면 바로 180도 회전했다. 현재는 그 사이에 선택 가능한 정밀 보정 단계가 추가됐다.

- Nav2가 필터링해 발행하는 `/dock_pose`를 받는다.
- DockRobot action이 끝나는 순간의 최신 목표를 `odom` 좌표계의 고정 목표로 사용한다.
- 이후 태그 관측이 조금 흔들려도 움직이는 목표를 계속 추종하지 않는다.
- 로봇 기준 오차로 속도 명령을 만들되, 성공 여부는 고정 목표 좌표축 기준 종·횡·yaw 오차로 판단한다.
- 최대 이동 18 cm, 최대 yaw 변화 30도, 최대 초기 오차를 두어 엉뚱한 목표를 추종하지 않는다.
- `/dock_pose`의 실제 QoS에 맞춰 `RELIABLE + VOLATILE`로 구독한다. 이전의 `TRANSIENT_LOCAL` 요청은 Jazzy 환경의 발행자와 호환되지 않아 메시지를 받지 못할 수 있었다.
- DockRobot이 이미 15 cm 성공 반경 안에서 즉시 끝나는 경우에도 콜백이 도착할 수 있도록 최대 1초 기다린다.

이 기능은 `use_tag_pose_refinement: true`일 때만 실행한다. 결과가 나쁘면 `false`로 바꾸면 기존의 성공하던 순서로 바로 돌아간다.

### 2.3 180도 회전 제어 변경

기존 회전은 매 주기 odom yaw 변화의 **절댓값을 누적**하고, 누적량이 약 180도에 도달하면 일정한 각속도에서 바로 정지했다. 이 방식은 미끄러짐·노이즈·제동 지연을 모두 회전량으로 더할 수 있고 감속 구간이 없어 실제 회전이 더 들어갈 수 있었다.

현재 방식은 다음과 같다.

1. 회전 직전 절대 odom yaw를 `start_yaw`로 읽는다.
2. `target_yaw = normalize(start_yaw + spin_yaw)`로 목표를 한 번 계산한다.
3. 매 주기 현재 odom yaw에서 목표까지 남은 각도를 다시 계산한다.
4. 목표 40도 전부터 최대 `0.15 rad/s`에서 최소 `0.025 rad/s`까지 감속한다.
5. 목표 오차가 1도 이내이고 정지 각속도가 0.5 deg/s 이하인 상태가 5주기 연속이어야 끝낸다.
6. 정확히 180도일 때 양·음 회전 방향이 수치 wrap에 따라 바뀌지 않도록 `spin_yaw` 부호를 유지한다.

따라서 시작 odom yaw가 89도라면 목표는 정규화된 `89 + 180 = -91도`가 된다. 시작값이 0도일 필요가 없으며, “현재 각도에서 180도”를 수행한다.

정지 판정의 각속도는 신선한 `/imu/data.angular_velocity.z`를 우선 사용한다. IMU가 잠시 끊기면 `/odom.twist.twist.angular.z`를 사용한다. 즉 IMU가 odom 절대 yaw를 대체하지는 않지만, 관성 때문에 더 돌아가는 중인데 회전 완료로 판정하는 문제를 줄인다.

### 2.4 전방 LiDAR 가정에서 후방 LiDAR 계약으로 변경

현재 도킹 전용 LiDAR는 다음 계약을 사용한다.

- 토픽: `/scan_2`
- 프레임: `base_scan2`
- TF 방향: `base_link <- base_scan2`
- 로봇 기준 후방 중심각: `pi rad = 180°`
- 스캔 로컬 각도를 직접 해석하지 않고 TF로 모든 점과 광선 방향을 `base_link`로 투영
- 최근 수신 0.30초 이내, header 0.50초 이내, timestamp가 단조 증가하는 고유 스캔만 사용
- LaserScan 특성에 맞춘 `BEST_EFFORT + VOLATILE` QoS

이렇게 바꿔 후방 장착 LiDAR의 로컬 0도가 어느 방향인지에 기대지 않는다. 설정에 사용하는 각도는 모두 `base_link` 기준이다.

과거 호환 파라미터 `backup_lidar_sector_center`와 `lidar_align_sector_center`는 기본값이 `NaN`이다. 여기에 유한한 숫자를 넣으면 예전 scan-frame 규약을 사용한 것으로 보고 안전하게 실패한다. 새 파라미터인 `*_sector_center_base`를 사용해야 한다.

### 2.5 회전 후 RANSAC 평면 정렬 강화

기존 RANSAC은 첫 유효 평면 추정값부터 즉시 회전 명령에 반영했다. 회전 직후 왜곡된 스캔, 짧은 프레임 모서리, 좌우 가이드가 지배 직선으로 선택되면 큰 오조향이 발생할 수 있었다.

현재는 다음 방어가 추가됐다.

- 로봇 후방 180°를 중심으로 좌우 30°씩, 총 60°만 사용한다.
- 기대하는 후면 패널 방향에서 15° 이상 벗어난 직선 후보는 RANSAC 단계에서 제외한다.
- 단순 inlier 수가 아니라 `inlier 수 × 직선 길이`를 기준으로 긴 패널을 우선한다.
- 최소 20개 점, 12개 inlier, 길이 0.15 m 이상을 요구한다.
- 정지 상태에서 3개의 고유 스캔이 서로 3° 이내로 일치해야 처음 회전을 시작한다.
- 추적 중 예상 평면과 5° 이상 달라지면 즉시 실패하지 않고 정지 후 재획득한다.
- 차이가 12°를 넘으면 다른 구조물을 잡은 것으로 보고 추가 회전을 거부한다.
- 총 보정 회전은 18° 이내로 제한한다.
- 최종 오차 1° 이내이고 정지 각속도 조건을 만족한 스캔이 5회 연속이어야 완료한다.

여기서 “평면과 정렬”은 로봇이 패널과 나란히 달린다는 뜻이 아니다. 패널 직선은 로봇의 좌우축과 평행하고, 패널의 수직 벡터(법선)는 로봇 전후축과 일치하도록 맞춘다. 결과적으로 로봇은 후면 패널을 향해 **수직으로 후진**한다.

### 2.6 후진 제어 변경

기존에도 LiDAR 거리 기반 후진은 있었지만, 현재는 후방 장착 위치·차체 후단·측면 구조를 분리해서 처리하고 후진 중 약한 방향 보정을 추가했다.

- 완료 거리 영역: 후방 중심 `180° ±10°`.
- 보호 영역: 후방 `180° ±30°`, 로봇 반폭 0.22 m + 여유 0.02 m 안의 점.
- 기존 보호 부채꼴 `±75°`는 좌우 가이드 레일을 장애물처럼 포함해 진입 직후 멈추게 할 수 있어 `±30°`로 줄였다.
- LiDAR 장착 x와 차체 후단 기준 x를 구분한다.
  - `backup_rear_reference_x = -0.2295 m`
  - 실제 TF에서 LiDAR가 이 기준보다 앞에 있어야 한다.
- 15 cm보다 멀 때 최대 0.05 m/s, 가까워질수록 최소 0.015 m/s까지 감속한다.
- 최대 이동 0.60 m, 한 단계 최대 시간 45초를 넘으면 실패한다.
- 한 개의 짧은 반사점으로 성공시키지 않고, 최소 5개의 인접 빔과 최소 3° 각도 폭을 요구한다.
- 성공 범위가 3개의 서로 다른 스캔에서 연속 확인되어야 한다.
- 목표보다 0.5 cm 이상 지나친 군집을 검출하면 overrun으로 실패한다.

현재 YAML의 최종 조건은 다음과 같다.

```text
목표 차체 후방 간격 = 0.020 m
허용 오차          = 0.005 m
허용 완료 범위      = 0.015 ~ 0.025 m
필요 군집           = 인접 5점 이상, 각도 폭 3° 이상
시간 안정성         = 고유 스캔 3회 연속
```

로그의 `rear_clearance`는 LiDAR 센서 원점까지의 raw range가 아니라, TF 투영 후 `backup_rear_reference_x`로 보정한 **차체 후단과 물체 사이의 종방향 간격**이다. 예를 들어 실제 TF의 LiDAR x가 약 `-0.166 m`라면 2.5 cm 완료 상한에 대응하는 센서 raw range는 약 8.85 cm다.

### 2.7 후진 중 약한 RANSAC 각도 보정

회전 직후 RANSAC 정렬을 끝내도 바닥 마찰 차이와 휠 편차로 후진 중 조금씩 비뚤어질 수 있어, 다음 조건의 보수적인 보정이 추가됐다.

- 후면 평면 법선 오차를 매 스캔 다시 계산한다.
- 3회 연속 품질을 통과한 뒤에만 제어에 사용한다.
- 저역 통과 필터 계수는 0.15다.
- 직전 필터값에서 2.5° 이상 튄 값은 버린다.
- 오차 1° 이내에서는 각속도 명령을 내지 않는다.
- 최대 각속도는 `0.004 rad/s`, 최대 변화율은 `0.010 rad/s²`다.
- 회전하려고 후진을 멈추지 않는다.
- RANSAC이 끊기거나 품질 조건을 통과하지 못하면 이전 angular 명령을 유지하지 않고 즉시 `angular.z = 0`으로 직진한다.
- 차체 후방 보호 간격이 10 cm 이하가 되면 보정을 완전히 끄고 직선 후진한다.
- 유효한 평면이 없을 때 IMU 적분 또는 휠 yaw drift가 5°를 넘으면 안전 실패한다.

RANSAC 품질 조건은 오차 5° 이내, inlier 비율 70% 이상, 직선 길이 0.15 m 이상이다. 유효한 평면 오차가 1° 이내로 돌아오면 IMU/휠의 상대 기준도 다시 잡아 의도적으로 수행한 LiDAR 보정이 drift 한도에 누적되지 않게 한다.

### 2.8 좌우 가이드 레일 인식 상태

알루미늄 좌우 프로파일을 각각 직선으로 맞춰 중앙을 구하는 실험 코드와 단위 테스트는 남아 있다. 하지만 현재 운영 경로에서는 다음 이유로 **꺼져 있다**.

- `use_lidar_guide_centering` 기본값은 `false`다.
- 메인 노드가 guide estimator를 후진 제어기에 연결하지 않으므로, 플래그만 `true`로 바꾸어도 현재는 중앙 제어가 활성화되지 않는다.
- 반사가 강한 프로파일은 입사각에 따라 점이 끊기고, 한쪽 레일만 보이거나 후면 패널 모서리가 섞여 중심점이 갑자기 바뀔 수 있었다.
- 좌우 위치 오차를 차동구동 로봇이 후진 중 yaw만으로 바로 고치면 S자 흔들림이나 프레임 충돌 위험이 커진다.

따라서 현재 실사용 제어는 **후면 평면 각도 정렬만 유지**하고 좌우 가이드 중앙 추종은 하지 않는다. 이 기능을 다시 시험하려면 단순히 YAML 플래그만 바꾸지 말고, estimator 연결·실측 rail 간격·반사 누락률·안전한 진입 궤적을 함께 검증해야 한다.

### 2.9 프로세스와 안전 처리

현재 실행 파일은 필요한 하위 스택을 직접 시작하고 종료한다.

- AprilTag composable container 시작
- `apriltag_bridge` 시작
- `opennav_docking` 서버 시작
- lifecycle 상태를 확인하고 configure/activate
- 종료 시 로봇 정지 명령을 여러 번 발행
- 하위 프로세스에 `SIGINT → SIGTERM → SIGKILL` 순서로 정리
- `/tmp/stella_dock_turn_backup.lock` 단일 실행 lock으로 중복 실행 차단
- 부모 `ros2 run` 프로세스가 사라지면 자식에도 종료 신호를 받도록 Linux parent-death signal 설정
- 전체 절차 100초 timeout

## 3. 구성 파일과 코드 책임

| 파일 | 책임 |
|---|---|
| `config/docking.yaml` | Nav2 docking server 설정과 현재 시험에서 덮어쓰는 핵심 `dock_turn_backup` 값 |
| `config/tags_36h11.yaml` | AprilTag family, 실제 크기 0.154 m, detector와 QoS 설정 |
| `launch/apriltag_36h11.launch.py` | RealSense color image/camera info를 AprilTag component에 연결 |
| `docking/apriltag_bridge.py` | `base_link <- tag36h11:0` TF를 `detected_dock_pose` PoseStamped로 10 Hz 발행 |
| `docking/dock_turn_backup.py` | 전체 상태 순서, DockRobot action, 태그 정밀 보정, timeout과 exit code |
| `docking/motion.py` | odom 회전, IMU 정지 확인, 선택 직진, LiDAR/odom 후진, 약한 방향 제어 |
| `docking/docking_lidar.py` | `/scan_2` 구독, 시간·frame 검증, TF 해결, base_link 투영 |
| `docking/lidar_geometry.py` | 각도 정규화, 투영, 후단 간격, 연속 빔 군집, 고유 스캔 안정성 계산 |
| `docking/lidar_alignment.py` | 후면 패널 RANSAC, PCA 직선, 평면 획득·추적·재획득, 실험용 가이드 추정 |
| `docking/charging.py` | 충전기 접촉, 충전 명령, 배터리 양의 전류 안정성 확인 |
| `docking/lifecycle.py` | docking server lifecycle configure/activate |
| `docking/stack_manager.py` | AprilTag·bridge·docking server 프로세스 시작과 정리 |
| `docking/safety.py` | 단일 실행 lock, 종료 코드, 부모 종료 감지 |
| `test/` | 후방 LiDAR 계약, 기하, 회전, RANSAC, 태그 보정, 안전 처리 회귀 테스트 |

`package.xml`에는 충전 제어의 `Bool` 메시지를 위한 `std_msgs` 의존성과 YAML 설정 회귀 테스트를 위한 `python3-yaml` test dependency가 포함되어 있다.

## 4. 토픽, 액션, TF 계약

### 4.1 입력

| 이름 | 타입 | 용도 |
|---|---|---|
| `/camera/camera/color/image_raw` | Image | AprilTag 영상 |
| `/camera/camera/color/camera_info` | CameraInfo | PnP pose 추정용 카메라 보정값 |
| `/odom` | Odometry | 위치, 절대 회전 목표, odom 각속도 fallback |
| `/imu/data` | Imu | 회전 정지 판정, 후진 중 gyro 적분 |
| `/wheel_odometry/yaw_diagnostics` | Vector3Stamped | `vector.x` encoder-only yaw; IMU 융합 odom의 사후 보정과 분리된 안전 기준 |
| `/scan_2` | LaserScan | 후면 패널 각도와 후방 간격 |
| `/dock_pose` | PoseStamped | Nav2가 필터링한 태그 기반 고정 정밀 목표 |
| `/sk120/available` | BatteryState | 충전기 접촉/사용 가능 상태 |
| `/battery_state` | BatteryState | 실제 충전 전류 확인 |

### 4.2 출력 및 action

| 이름 | 타입 | 용도 |
|---|---|---|
| `detected_dock_pose` | PoseStamped | bridge가 만든 외부 도킹 감지 pose |
| `/cmd_vel` | Twist | 회전·정밀 보정·후진 속도 명령 |
| `/dock_robot` | DockRobot action | Nav2 태그 접근 |
| `/sk120/cmd_output` | Bool | 충전 시작/취소 명령 |

### 4.3 필수 TF

- `odom -> base_link`
- 카메라 체인에서 `base_link -> ... -> camera` 변환
- AprilTag 검출 결과 `tag36h11:0`
- `base_link <- base_scan2`

LiDAR TF가 없거나 frame이 `base_scan2`가 아니면 fail-closed로 움직이지 않는다. 시작 로그의 다음 줄에서 실제 장착값을 반드시 확인한다.

```text
Docking LiDAR ready: base_link <- base_scan2, xyz=(...), yaw=...deg
```

후방 장착이라면 yaw는 대략 180° 부근이어야 한다. x, y, yaw가 URDF의 실제 장착과 다르면 모든 sector와 차체 후단 간격 계산이 잘못된다.

## 5. 현재 핵심 파라미터

실제로 자주 바꿔야 하는 값은 `config/docking.yaml`에 명시되어 있다. 나머지는 각 Python 파일의 `declare_parameters()` 기본값을 사용한다.

### 5.1 Nav2 태그 접근

| 파라미터 | 현재값 | 의미와 주의점 |
|---|---:|---|
| `external_detection_translation_x` | `-0.95` | 태그 TF로부터 생성하는 도킹 목표의 x 오프셋. 더 음수로 가면 현재 장착/축 기준에서는 회전 위치를 더 멀리 두는 방향으로 사용 중 |
| `filter_coef` | `0.1` | 외부 pose 필터 계수. 작을수록 일반적으로 더 부드럽지만 반응이 느려질 수 있음 |
| `docking_threshold` | `0.15` | Nav2 DockRobot의 위치 성공 반경. 정밀 보정 tolerance가 아님 |
| `v_linear_max/min` | `0.10 / 0.05 m/s` | Nav2 접근 속도 범위 |
| `v_angular_max` | `0.15 rad/s` | Nav2 접근 최대 각속도 |
| `use_collision_detection` | `false` | 접근 중 Nav2 collision detection 비활성. 시험 공간을 물리적으로 확보해야 함 |
| `navigate_to_staging_pose` | `false` | 저장된 staging pose로 먼저 가지 않고 보이는 dock를 바로 접근 |

Nav2 로그의 `Made contact with dock` 또는 `Robot is charging!`은 현재 plugin의 `use_battery_status: false`, `use_stall_detection: false` 조건에서는 **물리 충전 확인이 아니라 기하학적 성공 상태**일 수 있다. 전체 노드의 최종 충전 검증과 구분해야 한다.

### 5.2 태그 정밀 보정

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `use_tag_pose_refinement` | `true` | Nav2 접근 뒤 정밀 보정 사용 |
| `tag_refinement_target_pose_topic` | `/dock_pose` | 필터링된 목표 pose |
| `tag_refinement_target_wait_timeout_sec` | `1.0 s` | action 직후 queued pose 대기 |
| `tag_refinement_target_max_age_sec` | `1.5 s` | 허용 목표 수신 age |
| `tag_refinement_timeout_sec` | `18.0 s` | 정밀 보정 최대 시간 |
| 종방향 tolerance | `0.040 m` | 고정 dock 축의 앞뒤 허용 오차 |
| 횡방향 tolerance | `0.025 m` | 고정 dock 축의 좌우 허용 오차 |
| yaw tolerance | `2°` | 최종 방향 허용 오차 |
| `tag_refinement_stable_cycles` | `5` | 모든 tolerance 동시 만족 연속 주기 |
| 최대 선속도 | `0.025 m/s` | 보정 이동 속도 제한 |
| 최대 각속도 | `0.08 rad/s` | 보정 회전 속도 제한 |
| 최대 초기 오차 | `0.18 m / 0.10 m / 25°` | 종/횡/yaw 안전 진입 범위 |
| 최대 누적 이동 | `0.18 m` | 정밀 단계 폭주 방지 |
| 최대 yaw excursion | `30°` | 정밀 단계 폭주 방지 |
| `tag_refinement_abort_on_failure` | `true` | 보정 실패 시 전체 도킹 중단 |

정밀 기능만 즉시 끄는 실행 예시는 다음과 같다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_tag_pose_refinement:=false
```

보정 실패가 있어도 기존 순서를 계속 시험하려면 다음처럼 실행할 수 있다. 안전 판단을 우회할 수 있으므로 원인 로그를 확인한 시험에서만 사용한다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p tag_refinement_abort_on_failure:=false
```

### 5.3 회전 전 선택 직진

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `use_pre_spin_forward` | `false` | 현재 완전히 꺼짐 |
| `pre_spin_forward_duration_sec` | `1.0 s` | 켰을 때 직진 시간 |
| `pre_spin_forward_speed` | `0.03 m/s` | 직진 속도 |
| `pre_spin_forward_max_distance` | `0.05 m` | odom 이동 안전 상한 |

예전의 약 1초 직진을 재현하려면 정밀 보정과 독립적으로 이 플래그만 켠다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_pre_spin_forward:=true
```

태그 목표를 더 정확히 맞추기 위해 이 직진 시간을 늘리는 방식은 권장하지 않는다. 태그/odom 기반 오차를 보지 않는 open-loop 동작이고, 회전 반경을 줄일 수 있다.

### 5.4 odom 180도 회전

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `spin_yaw` | `pi rad` | 시작 절대 odom yaw에 더할 회전량; 음수면 반대 방향 |
| `spin_angular_speed` | `0.15 rad/s` | 최대 속도 |
| `spin_min_angular_speed` | `0.025 rad/s` | 감속 구간 최소 속도 |
| `spin_slowdown_angle` | `40°` | 남은 오차가 이 값보다 작으면 선형 감속 |
| `spin_tolerance` | `1°` | 목표 yaw 허용 오차 |
| `spin_stable_cycles` | `5` | yaw와 정지 조건을 연속 만족할 주기 |
| `imu_stationary_yaw_rate` | `0.5°/s` | 멈춤 판정 한계 |

`spin_yaw`를 180도보다 작게 보정값처럼 조절하는 것보다, odom/TF/휠 스케일을 먼저 확인하고 회전 후 LiDAR 정렬로 물리 스테이션 오차를 제거하는 구성이 현재 설계 의도다.

### 5.5 정지 상태 LiDAR 평면 정렬

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `use_lidar_alignment` | `true` 기본 | 회전 후 RANSAC 정렬 사용 |
| `lidar_align_sector_center_base` | `180°` | base_link 기준 후방 |
| `lidar_align_sector_width` | `60°` | 실제 영역은 180° ±30° |
| range | `0.15~2.0 m` | RANSAC 입력 범위 |
| 최소 점/inlier | `20 / 12` | 평면 품질 |
| RANSAC 반복 | `100` | 후보 탐색 횟수 |
| inlier 거리 | `0.035 m` | 직선 허용 잔차 |
| 최소 직선 길이 | `0.15 m` | 짧은 프레임 모서리 배제 |
| 후보 최대 오차 | `15°` | 기대 후면 방향과 다른 후보 배제 |
| 최종 tolerance | `1°` | 정렬 완료 각도 |
| 최대/최소 각속도 | `0.06 / 0.012 rad/s` | 정렬 제어 제한 |
| 획득 안정성 | `3 scans, 3°` | 회전 시작 전 합의 조건 |
| soft/hard 추적 잔차 | `5° / 12°` | 재획득 또는 안전 중단 경계 |
| 최대 보정 회전 | `18°` | 오인식 폭주 방지 |
| timeout | `12 s` | 전체 평면 정렬 제한 |

### 5.6 LiDAR 후진과 방향 보정

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `use_lidar_backup` | `true` | 거리 기반 후진. `false`면 odom 0.50 m 후진으로 fallback |
| `backup_lidar_sector_width` | `20°` | 완료 거리 영역: 180° ±10° |
| `backup_lidar_safety_sector_width` | `60°` | 보호 영역: 180° ±30° |
| `backup_target_rear_clearance` | `0.020 m` | 차체 후단 목표 간격 |
| `backup_clearance_tolerance` | `0.005 m` | 성공/overrun 허용 오차 |
| `backup_speed` | `0.05 m/s` | 먼 거리 최대 후진 속도 |
| `backup_min_speed` | `0.015 m/s` | 근거리 최소 속도 |
| `backup_slowdown_clearance` | `0.15 m` | 감속 시작 보호 간격 |
| `backup_max_travel` | `0.60 m` | 이동 안전 상한 |
| `backup_blocked_timeout_sec` | `1.0 s` | 가까워 정지는 했지만 완료 군집이 안 생길 때 실패까지 대기 |
| `use_lidar_heading_during_backup` | `true` | 후진 중 약한 후면 평면 보정 |
| filter coefficient | `0.15` | 법선 오차 저역 통과 필터 |
| heading tolerance | `1°` | 이 안에서는 angular 0 |
| max heading error | `5°` | 제어에 사용할 수 있는 RANSAC 오차 상한 |
| max jump | `2.5°` | 단일 측정 급변 거부 |
| min inlier ratio | `0.70` | 후진 중 RANSAC 품질 |
| stable cycles | `3` | 보정 활성화 전 품질 연속 확인 |
| max angular speed | `0.004 rad/s` | 흔들림 방지용 매우 작은 보정 |
| max angular accel | `0.010 rad/s²` | 명령 급변 제한 |
| disable clearance | `0.10 m` | 이보다 가까우면 보정을 끄고 직진 |
| `use_lidar_guide_centering` | `false` | 좌우 레일 중앙 추종 비활성 |

최종 목표 거리를 바꿀 때는 `backup_target_rear_clearance`를 수정한다. 값을 크게 하면 더 일찍 멈추고, 작게 하면 더 깊이 들어간다. 허용 오차를 너무 작게 하면 물리적으로 도킹했는데도 센서 분해능·노이즈 때문에 완료 군집이 만들어지지 않아 `blocked` 실패가 날 수 있다. 목표와 tolerance를 바꾼 뒤에는 LiDAR 최소 측정거리와 장착 x를 고려한 `completion_range` 검증 로그도 확인해야 한다.

### 5.7 개발 모드와 충전 검증

`development_test_mode`의 코드 기본값은 현재 `true`다.

- `true`: LiDAR 후진 성공 후 충전 접촉을 확인하지 않고 exit 0.
- `false`: `/sk120/available` 접촉, `/sk120/cmd_output=true`, `/battery_state`의 `current >= 0.05 A`가 3초 유지되어야 exit 0.

실제 무인 운용에서는 다음처럼 충전 검증을 켜야 한다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p development_test_mode:=false
```

## 6. 빌드와 실행

소스의 YAML을 수정해도 `ros2 run`은 `get_package_share_directory('docking')`가 가리키는 **설치 공간의 설정 파일**을 자동으로 읽는다. 수정 후에는 빌드하고 올바른 workspace를 source해야 한다.

```bash
cd ~/colcon_ws
colcon build --symlink-install --packages-select docking
source install/setup.bash
ros2 pkg prefix docking
ros2 run docking dock_turn_backup
```

`ros2 pkg prefix docking` 결과가 방금 빌드한 `~/colcon_ws/install/docking`이 아니면 다른 overlay의 구버전을 실행하고 있는 것이다.

YAML 기본값 대신 일회성 override를 함께 줄 수 있다.

```bash
ros2 run docking dock_turn_backup --ros-args \
  -p use_tag_pose_refinement:=false \
  -p use_pre_spin_forward:=false \
  -p backup_target_rear_clearance:=0.020
```

동일 노드를 두 번 실행하면 lock 때문에 두 번째 실행은 exit code 2로 거부된다. 이전 프로세스가 실제로 종료됐는지 먼저 확인한다.

## 7. 실행 전 점검

### 7.1 토픽과 주기

```bash
ros2 topic hz /odom
ros2 topic hz /imu/data
ros2 topic hz /wheel_odometry/yaw_diagnostics
ros2 topic hz /scan_2
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/color/camera_info
```

카메라 image와 camera_info가 모두 있어도 timestamp가 맞지 않으면 AprilTag 로그에 synchronized pair 부족 경고가 날 수 있다. 이 경우 태그 접근이 끊기거나 흔들릴 수 있으므로 카메라 드라이버의 timestamp와 QoS를 먼저 확인한다.

### 7.2 TF와 LiDAR 방향

```bash
ros2 run tf2_ros tf2_echo base_link base_scan2
ros2 run tf2_ros tf2_echo base_link tag36h11:0
ros2 run tf2_ros tf2_echo odom base_link
```

로봇을 움직이지 않은 상태에서 `base_link <- base_scan2` 값이 매번 일정해야 한다. tag TF는 태그가 보일 때 연속적으로 나와야 한다.

### 7.3 실제 실행 파라미터 확인

노드가 실행 중일 때 다른 터미널에서 확인한다.

```bash
ros2 param get /dock_turn_backup use_tag_pose_refinement
ros2 param get /dock_turn_backup use_pre_spin_forward
ros2 param get /dock_turn_backup backup_target_rear_clearance
ros2 param get /dock_turn_backup backup_clearance_tolerance
ros2 param get /dock_turn_backup lidar_align_sector_width
```

Nav2 서버 값도 별도 노드에서 확인한다.

```bash
ros2 param get /docking_server simple_charging_dock.external_detection_translation_x
ros2 param get /docking_server simple_charging_dock.docking_threshold
```

## 8. 로그 읽는 법과 실패 원인

### 8.1 단계 식별용 정상 로그

| 로그 일부 | 의미 |
|---|---|
| `Docking LiDAR ready` | `/scan_2`, frame, TF 검증 완료 |
| `Docking step complete` | Nav2의 15 cm 반경 기준 접근 완료; 물리 충전 의미 아님 |
| `Starting bounded tag pose refinement` | 추가 정밀 보정 시작 |
| `Tag pose refinement complete` | 고정 태그 목표 tolerance 만족 |
| `Optional pre-spin forward step is disabled` | 현재 의도대로 임의 직진 생략 |
| `Spinning to an absolute odom yaw target` | 시작 yaw와 180도 목표 계산 완료 |
| `Spin step complete ... target_error=` | odom 각도와 정지 조건 만족 |
| `LiDAR rear plane acquired` | 정지 상태의 일관된 후면 패널 3회 획득 |
| `LiDAR plane alignment complete` | 패널 법선 기준 정렬 완료 |
| `LiDAR backup:` | 후진 거리·각도·RANSAC 상태 |
| `Dock-turn-backup sequence complete` | 거리 군집 3회 연속 만족 |

### 8.2 반복 시험에서 실제로 문제가 됐던 원인

1. **Nav2 성공 반경과 정밀 위치의 혼동**  
   `docking_threshold=0.15`는 접근 action을 안정적으로 끝내기 위한 반경이다. 이를 몇 cm로 무리하게 줄이면 태그 노이즈 때문에 앞뒤 왕복과 timeout이 생겼다. 현재는 Nav2 반경을 유지하고 별도 저속 정밀 단계에서 오차를 줄인다.

2. **태그 목표를 너무 가까이 둔 회전 충돌**  
   정밀 보정이 약 0.8 m 목표를 정확히 만들자, 예전의 느슨한 Nav2 정지 때보다 오히려 스테이션에 가까워져 180도 회전 중 프레임과 접촉했다. 현재 `external_detection_translation_x=-0.95`로 회전 공간을 약 15 cm 늘렸다.

3. **회전량 누적 방식의 overshoot**  
   매 샘플 yaw 변화의 절댓값 누적과 일정 속도 제어는 노이즈와 제동을 처리하지 못했다. 절대 odom 목표, 40도 감속, 1도 tolerance, IMU 정지 확인으로 변경했다.

4. **회전 직후 첫 LiDAR 평면 오인식**  
   실제 실패 로그에서 첫 평면이 13.54°, 다음이 6.22°처럼 크게 바뀌었다. 현재는 정지 상태의 일관된 3개 스캔을 획득한 후에만 움직인다.

5. **추적 잔차가 5°를 근소하게 넘은 즉시 실패**  
   5.14° 변화처럼 작은 초과도 이전에는 종료됐다. 현재 5~12°는 정지·재획득하고, 12° 초과만 위험한 표면 전환으로 중단한다.

6. **넓은 후방 안전 sector가 좌우 가이드 레일을 장애물로 판단**  
   스테이션에 진입하면 레일은 정상적으로 로봇 옆에 가까워진다. 과거 ±75° 보호 영역은 이를 포함해 진입 직후 정지시켰다. 현재 차체 통과 폭 안의 ±30° 보호 영역으로 제한했다.

7. **단일 근거리 반사점으로 정지한 뒤 성공하지 못함**  
   `protective_clearance`가 작으면 충돌 방지를 위해 즉시 선속도를 0으로 만든다. 그러나 그 점이 중앙 완료 영역의 연속 5빔·3°·3스캔 조건을 만족하지 않으면 성공할 수 없다. 1초 지속되면 `LiDAR backup remains blocked outside the completion condition`으로 실패한다. 사진상 장애물이 없어 보여도 레일 모서리, 케이블, 차체 반사, 잘못된 TF가 원인이 될 수 있다.

8. **물리 성공 위치와 코드 목표가 2 mm 차이**  
   목표 1.0 cm, tolerance 0.5 cm일 때 실제 반복 위치 약 1.7 cm는 상한 1.5 cm 밖이었다. 현재 목표를 1.5 cm로 바꿔 1.0~2.0 cm를 성공 범위로 삼는다.

9. **후진 중 너무 큰 각도 보정**  
   LiDAR 법선이 튈 때 이전 명령을 유지하거나 크게 조향하면 스테이션 안에서 곡선 궤적이 생겼다. 현재 최대 0.004 rad/s, 1° deadband, LPF, jump reject를 적용하고 10 cm 이내에서는 직선만 사용한다.

10. **태그 정밀 목표 QoS 불일치**  
    `/dock_pose` 발행자는 VOLATILE인데 구독자가 TRANSIENT_LOCAL이면 연결이 호환되지 않아 `No fresh filtered Nav2 dock pose`가 발생할 수 있었다. 현재 VOLATILE로 수정했다.

11. **action 즉시 완료와 목표 콜백 경쟁**  
    이미 성공 반경 안에서 재실행하면 action 결과가 `/dock_pose` 콜백보다 먼저 처리될 수 있었다. 최대 1초 bounded wait를 추가했다.

12. **정밀 보정 성공 오차를 회전하는 base 좌표로 판정**  
    제자리 yaw 보정만 했는데도 좌우 위치 오차가 변한 것처럼 보이는 문제가 있었다. 성공 판정은 고정 target 축에서 하고, 속도 계산만 현재 base 축에서 한다.

13. **정리 시 AprilTag container `exit code -11`**  
    도킹 단계가 먼저 실패하거나 Ctrl-C가 들어온 뒤 component container가 cleanup 중 segfault로 끝나는 로그가 있었다. 이는 대개 앞선 도킹 실패의 원인이 아니라 종료 과정에서 생기는 2차 오류다. 첫 번째 `[ERROR] [dock_turn_backup]` 줄을 우선 분석한다. 반복적으로 정상 종료에서도 발생하면 `apriltag_ros` component/container 조합을 별도 조사해야 한다.

### 8.3 대표 오류별 조치

| 오류 | 직접 의미 | 우선 확인 |
|---|---|---|
| `No fresh filtered Nav2 dock pose` | 정밀 목표가 없거나 1.5초보다 오래됨 | `/dock_pose` hz/QoS, docking server가 같은 설정으로 실행됐는지 |
| `Refusing an unexpectedly large tag refinement` | 초기 종/횡/yaw가 안전 범위 밖 | 태그 TF 축, `external_detection_*`, 실제 시작 위치. 한도부터 늘리지 말 것 |
| `Tag pose refinement timed out` | 18초 내 tolerance 안정화 실패 | 오차 로그가 수렴하는지, odom 지연, 너무 작은 tolerance |
| `Spin step failed or timed out` | 절대 목표 또는 정지 연속 조건 미달 | `/odom` yaw, `/imu/data` age/rate, 모터 최소 속도 |
| `rear panel RANSAC is invalid` | 점·inlier·길이·방향 조건 미달 | `/scan_2` 시각화, 180° ±30° 안에 패널이 있는지, TF yaw |
| `Rejecting one inconsistent ... reacquiring` | soft 추적 jump | 로봇은 정지하며 정상 재획득 시 계속 진행. 반복되면 반사/sector 문제 |
| `tracking changed too far` | 12° 이상 다른 구조로 전환 | 후면 패널 가시성, 프레임/레일 오인식, 회전 오차 |
| `alignment exceeded ... 18deg` | LiDAR가 과도한 추가 회전을 요구 | odom 180도 결과와 LiDAR TF를 먼저 수정 |
| `Wheel-only yaw diagnostics ... unavailable` | 독립 휠 yaw 안전 기준 없음 | `/wheel_odometry/yaw_diagnostics` 노드와 hz |
| `Docking LiDAR failed during backup` | scan stale/frame/time 오류 | `/scan_2`, sensor timestamp, `base_scan2` |
| `lost its dock plane and exceeded ... drift` | RANSAC 없이 상대 yaw가 5° 초과 | 바퀴 미끄러짐, IMU/휠 데이터, 패널 가시성 |
| `remains blocked outside completion` | 안전상 정지했지만 성공 군집 불충족 | 중앙·보호 clearance 차이, 레일/반사점, 완료 sector |
| `target was overrun` | 1.0 cm보다 가까운 유효 군집 | 즉시 물리 간격 확인, 목표를 더 작게 바꾸지 말 것 |
| `[ros2run]: Process exited with failure 4` | 도킹 단계 실패 | 이 줄 위의 최초 `dock_turn_backup` ERROR가 실제 원인 |

## 9. 튜닝 원칙과 권장 순서

여러 파라미터를 한 번에 바꾸면 어떤 변화가 성공률을 올렸는지 알 수 없다. 다음 순서로 하나씩 조정한다.

1. **센서·TF 고정**  
   `/scan_2`와 `base_scan2`, LiDAR x/y/yaw, tag 크기 0.154 m, 카메라 TF부터 확인한다.

2. **회전 공간 확보**  
   태그 앞에서 정확도보다 충돌 여유를 먼저 확보한다. `external_detection_translation_x`를 한 번에 2~5 cm만 바꾸고 실제 회전 궤적을 측정한다.

3. **Nav2 접근 단독 확인**  
   필요하면 `use_tag_pose_refinement=false`, `use_lidar_alignment=false`로 접근 종료 위치 분포를 여러 번 기록한다. `docking_threshold`를 지나치게 줄여 정밀도를 얻으려 하지 않는다.

4. **정밀 태그 보정 활성화**  
   시작 오차와 종료 오차 로그를 비교한다. 목표 위치가 잘못됐으면 controller gain보다 `external_detection_*`와 TF를 먼저 수정한다.

5. **odom 180도 회전 확인**  
   `start`, `target`, `final`, `target_error`, `stationary_rate`를 기록한다. final error가 작지만 실제 로봇이 틀리면 odom 자체나 base/IMU 축 문제다.

6. **정지 RANSAC 정렬 확인**  
   inlier 비율과 line length가 반복해서 안정적인지 본다. sector를 넓히면 점은 늘지만 레일과 주변 벽을 잡을 가능성도 함께 늘어난다.

7. **후진 방향 보정 없이 거리 확인**  
   필요하면 `use_lidar_heading_during_backup=false`로 거리 정지만 검증한다. 이때 RANSAC 각도 보정은 꺼지지만 LiDAR 최종 거리 정지는 유지된다.

8. **약한 방향 보정 활성화**  
   현재 보수적 기본값에서 시작한다. 최대 각속도를 먼저 올리지 말고 filter, jump, stable 상태가 로그에서 안정적인지 확인한다.

9. **최종 성공 간격 조정**  
   최소 10회 반복한 실제 접촉 위치 분포를 보고 `backup_target_rear_clearance`와 tolerance를 조정한다. 하드웨어 충전 성공 범위를 포함하되 overrun 여유를 남긴다.

10. **마지막에 충전 검증 전환**  
    기하학적 반복 성공이 확보된 뒤 `development_test_mode=false`로 충전 접촉과 전류까지 통합한다.

현재 안정 버전으로 즉시 돌아가는 가장 중요한 스위치는 다음 두 개다.

```yaml
dock_turn_backup:
  ros__parameters:
    use_tag_pose_refinement: false
    use_pre_spin_forward: false
```

첫 번째는 새 정밀 보정만 제거하고, 두 번째는 임의 직진을 제거한다. odom 회전, 정지 RANSAC 정렬, LiDAR 거리 후진은 그대로 유지된다.

## 10. 테스트

패키지 단위 테스트는 다음을 검증한다.

- 후방 LiDAR 토픽·frame·sector 기본 계약
- LaserScan timestamp, freshness, QoS, TF 투영
- LiDAR 장착점과 차체 후단 기준의 완료 range 계산
- 인접 빔 군집 및 고유 스캔 안정성
- odom 절대 180도 목표와 감속 명령
- IMU timestamp 기반 yaw 적분과 정지 판정
- 후진 중 LPF, jump reject, deadband, angular rate limit
- 긴 후면 패널이 짧고 조밀한 프레임 모서리보다 선택되는지
- 회전 중 동일 평면의 예상 법선 변화 보상
- 실제 실패 로그의 13.54°→6.22° 획득 jump와 5.14° tracking jump 회귀
- 태그 정밀 보정의 고정 목표축 오차와 안전 envelope
- 실험용 좌우 가이드 양쪽 검출/한쪽 검출 거부
- 단일 실행 lock과 종료 코드

빌드 후 전체 패키지 테스트:

```bash
cd ~/colcon_ws
colcon test --packages-select docking --event-handlers console_direct+
colcon test-result --verbose
```

Python 테스트만 빠르게 실행:

```bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/docking
python3 -m pytest test -q
```

단위 테스트 통과는 실제 카메라 동기, TF 보정, 바닥 마찰, 스테이션 반사 특성을 보장하지 않는다. 실제 로봇에서는 단계별 로그와 물리 비상 정지를 함께 사용해야 한다.

## 11. 종료 코드

| 코드 | 의미 |
|---:|---|
| `0` | 현재 모드의 성공 조건 만족 |
| `1` | 처리되지 않은 내부 오류 또는 cleanup 오류 |
| `2` | 잘못된 요청/파라미터 또는 중복 실행 |
| `3` | 센서, TF, action server, base 상태 준비 실패 |
| `4` | 태그 접근·정밀 보정·회전·RANSAC·후진 중 도킹 실패 |
| `5` | 전체 100초 timeout |
| `6` | 실사용 모드에서 충전 확인 실패 |
| `129` | SIGHUP |
| `130` | SIGINT/Ctrl-C |
| `143` | SIGTERM |

개발 모드의 exit 0은 “지정된 LiDAR 후방 간격에 도달했다”는 뜻이지 충전 전류가 확인됐다는 뜻이 아니다.

## 12. 현재 상태 요약

현재 기본 구성은 다음 선택을 한다.

- Nav2의 넓은 15 cm 성공 반경은 유지한다.
- 태그 기반 고정 목표 정밀 보정은 켠다.
- 목표 위치 없이 수행하던 1초 직진은 끈다.
- 회전 위치는 태그 기준 기존보다 약 15 cm 멀리 둔다.
- 현재 odom yaw에서 정확히 180도 떨어진 절대 목표로 감속 회전한다.
- IMU는 절대 목표 대신 정지 확인과 상대 drift 보조에 쓴다.
- 후방 180° ±30°의 후면 패널 RANSAC으로 회전 후 정렬한다.
- 좌우 가이드 레일 중앙 추종은 끈다.
- 후진 중에는 멀리서만 최대 0.004 rad/s의 약한 평면 보정을 사용한다.
- 10 cm 이내에서는 angular 보정을 끄고 직선 후진한다.
- 차체 후단 간격 1.0~2.0 cm의 신뢰 가능한 군집을 3회 확인하면 성공한다.
- 기본은 개발 시험 모드이므로 실제 충전 확인은 생략한다.

이 구성에서 문제가 생기면 안전 한도를 무작정 넓히기보다, 로그에서 **태그 접근 → 정밀 보정 → odom 회전 → RANSAC 획득/추적 → 후진 거리 군집 → 충전 확인** 중 최초로 실패한 단계를 찾아 그 단계의 센서와 파라미터만 조정해야 한다.

# STELLA N5 AprilTag 회전 후진 도킹

이 문서는 `docking` 패키지의 **2026-09-18 현재 작업 트리**를 기준으로 작성한 설계·운용 문서다. 2026-09-17 현장 실행에서 AprilTag 접근, 정밀 위치 보정, 180° 회전, 후면 평면 정렬, LiDAR 후진과 최종 정지까지 전체 순서의 실물 성공을 확인했다. 2026-09-18의 엔코더 heading guard와 정렬 outlier 합의 수정은 최신 실패 로그를 재현한 제어 루프 시험까지 통과했으며 실물 재시험 전이다. 앞선 한 번의 성공은 반복 성공률 통계나 충전 전류 검증 결과를 뜻하지 않는다.

최신 18:11 실행에서는 첫 정지 재정렬 후 후진까지 성공했지만 두 번째 보정에서 실패했다. 정지 실측 50개 스캔으로 후면 패널과 꺾인 모서리가 기존 35 mm RANSAC 폭에 섞이는 것을 확인했다. 현재 허용 폭은 **10 mm**이며 PCA 직선에 맞춰 지지점을 다시 선정한다. 정렬이 완료되면 그 자세를 새 직진 drift 기준으로 사용해 의도적인 보정 회전을 이후 방향 이탈로 세지 않는다. 제자리 보정 중에는 임시 엔코더 guard로 전환해 후진을 재개하지 않는다.

정지 복구는 inlier 75%·길이 0.30 m 이상인 벽이 고유 스캔 5개에서 각도 폭 1° 이내일 때 새 평면을 확정한다. 1° 이내 정렬과 IMU·wheel odom 정지 확인을 마쳐야 후진을 재개한다. 실측 점군 재생과 제어 루프 모의시험을 수행했으며 상세 근거는 [실패 분석](FAILURE_ANALYSIS.md)에 기록했다.

2026-09-18 실행에서는 정렬과 후진 시작까지 정상이었지만, 2 cm 이동한 뒤 RANSAC 각도가 약 0.7°에서 3.4°로 이동 의존적으로 바뀌었다. IMU/odom에는 약 2.1° 바이어스가 있었고 엔코더 yaw는 0.13°로 직진을 나타냈지만, LiDAR-휠 잔차도 2.7~3.0°가 되어 기존 2° 합의 한계를 넘었다. 현재는 **LiDAR+엔코더 전용 yaw**와 **LiDAR+IMU 계열 yaw**의 정상 합의를 우선 사용하고, 두 그룹과 모순되더라도 LiDAR-휠 잔차가 4° 이내에서 3개 스캔 동안 안정되면 의심스러운 LiDAR 각도로 조향하지 않고 엔코더 heading guard로 직선 후진한다. 4°를 넘거나 휠 drift가 8°에 도달하면 기존처럼 정지한다.

같은 날 16:45 실행은 180° 회전을 목표 오차 0.13°로 끝냈지만 후진 전에 실패했다. 평면 정렬 중 정상 후보가 약 `-0.8~+1.6°`로 수렴하는 사이 한 스캔씩 `-4.1°` 후보가 섞였고, motion-compensated 잔차가 5°를 근소하게 넘을 때마다 획득 상태를 모두 초기화해 12초 제한을 소진했다. 현재는 5~12° 불일치 한 번은 정지 상태에서 버리고 기존 평면을 유지하며, **2개 고유 스캔에서 연속될 때만 재획득**한다. 12°를 넘는 전환은 즉시 실패하고 전체 정렬 제한은 18초다.

2026-09-15까지의 실기 로그에서는 20 Hz 제어 주기, 정밀 yaw 무응답 복구, 고유 odom 정지 판정, 엔코더 signed 32-bit 롤오버, 후진 평면 유실 시 정지·재획득, 동적 LiDAR 정렬 회전 예산을 보완했다. 태그 정밀 보정은 **목표점 방향 정렬 → 위치 이동 → 최종 yaw 정렬**의 세 단계로 동작한다. 영상 경로는 압축 입력 자동 선택, mono8 왜곡 보정, 최대 10 Hz reliable 출력과 실제 촬영 시각 보존을 사용한다.

자동 실행 서버는 기존 Nav2 서버와의 액션 충돌을 막기 위해 매 실행마다 `/stella_docking_<ID>/` 아래에서 동작한다. 액션, lifecycle, 검출 pose와 정밀 목표 pose를 함께 분리한다. 아래의 `/dock_robot`, `/dock_pose`, `/docking_server`는 논리 이름이며, 자동 실행 중 실제 조회 경로는 시작 로그의 전용 namespace를 사용해야 한다. 변경 근거와 실패 이력은 [후속 실패 분석](FAILURE_ANALYSIS.md), 정확도 검토는 [정확도 개선 검토](ACCURACY_REVIEW.md)에 정리돼 있다.

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
  └─ 멀리서는 후면 평면 RANSAC과 선택적 양쪽 레일 중심으로 각도 보정
  └─ 큰 평면 오차는 후진을 멈추고 제자리 정렬한 뒤 재개
  └─ LiDAR·휠 그룹과 LiDAR·IMU 그룹을 함께 비교해 센서 바이어스 판별
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
| `backup_target_rear_clearance` | 코드 기본 `0.010 m` | `0.0145 m` | 실물 성공 후 측정한 LiDAR-벽 거리 7.8 cm를 차체 후단 기준으로 변환 |
| `backup_clearance_tolerance` | `0.005 m` | `0.010 m` | LiDAR-벽 거리 7.8 cm ±1 cm를 허용 |
| 회전 전 직진 | 기능 없음 | 파라미터화 후 현재 `false` | 임의의 시간 직진이 태그 보정 결과를 망가뜨리거나 회전 여유를 줄이지 않도록 기본 비활성화 |

`staging_x_offset: -0.80`은 그대로지만, 실행 코드가 `navigate_to_staging_pose: false`로 DockRobot goal을 보내므로 현재 절차에서는 Nav2 사전 staging 이동에 사용되지 않는다. 실제 태그 기반 접근 목표에 직접 영향을 주는 값은 `external_detection_translation_x: -0.95`다.

### 2.2 태그 접근 이후 정밀 보정 추가

기존에는 Nav2 DockRobot action이 성공하면 바로 180도 회전했다. 현재는 그 사이에 선택 가능한 정밀 보정 단계가 추가됐다.

- Nav2가 필터링해 발행하는 `/dock_pose`를 받는다.
- DockRobot action이 끝나는 순간의 최신 목표를 `odom` 좌표계의 고정 목표로 사용한다.
- 이후 태그 관측이 조금 흔들려도 움직이는 목표를 계속 추종하지 않는다.
- 로봇 기준 오차로 속도 명령을 만들되, 성공 여부는 고정 목표 좌표축 기준 종·횡·yaw 오차로 판단한다.
- 목표점 방위가 8° 이상이면 전진하지 않고 먼저 제자리 정렬한다. 방위가 작아진 뒤 위치를 맞추고, 최종 태그 yaw는 위치 허용 범위에 들어온 뒤 별도로 맞춘다.
- 최대 이동 18 cm, 최대 yaw 변화 30도, 최대 초기 오차를 두어 엉뚱한 목표를 추종하지 않는다.
- 목표의 수신 시각과 관측 header 시각을 함께 확인한다. 위치·yaw 허용 범위에서 선속도 0.005 m/s 이하, 정지 각속도 조건을 만족한 **서로 다른 odom 5개**를 받아야 보정을 완료한다.
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
5. 목표 오차가 1도 이내이고 정지 각속도가 0.5 deg/s 이하인 상태가 고유 odom 5개에서 연속 확인되어야 끝낸다. 같은 odom을 여러 번 읽어도 횟수를 올리지 않는다.
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
- 추적 중 예상 평면과 5° 이상 달라진 스캔 하나는 정지 상태에서 버리고 기존 평면을 유지한다. 2개 스캔 연속이면 정지 후 재획득한다.
- 차이가 12°를 넘으면 다른 구조물을 잡은 것으로 보고 추가 회전을 거부한다.
- 기본 보정 회전은 18°다. 평면을 안전하게 재획득하면 이미 소비한 누적 회전과 새 오차, 3° 여유를 반영해 예산을 갱신하되 총 누적 회전은 30°에서 차단한다.
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
- 18 cm보다 멀 때 최대 0.035 m/s, 가까워질수록 최소 0.012 m/s까지 감속한다.
- 최대 이동 0.60 m, 한 단계 최대 시간 45초를 넘으면 실패한다.
- 한 개의 짧은 반사점으로 성공시키지 않고, 최소 5개의 인접 빔과 최소 3° 각도 폭을 요구한다.
- 성공 범위가 3개의 서로 다른 스캔에서 연속 확인되어야 한다.
- 목표보다 1 cm 이상 지나친 군집을 검출하면 overrun으로 실패한다.

현재 YAML의 최종 조건은 다음과 같다.

```text
목표 LiDAR-벽 간격  = 0.078 m
허용 오차          = 0.010 m
LiDAR 완료 범위     = 0.068 ~ 0.088 m
차체 후단 목표 간격 = 0.0145 m
차체 완료 범위      = 0.0045 ~ 0.0245 m
필요 군집           = 인접 5점 이상, 각도 폭 3° 이상
시간 안정성         = 고유 스캔 3회 연속
```

로그의 `rear_clearance`는 LiDAR 센서 원점까지의 raw range가 아니라, TF 투영 후 `backup_rear_reference_x`로 보정한 **차체 후단과 물체 사이의 종방향 간격**이다. 실제 TF의 LiDAR x `-0.166 m`와 차체 후단 x `-0.2295 m`의 차이는 6.35 cm다. 따라서 LiDAR-벽 목표 7.8 cm는 차체 후단 간격 1.45 cm이고, ±1 cm 허용 범위는 각각 6.8~8.8 cm와 0.45~2.45 cm로 대응한다.

### 2.7 후진 중 RANSAC 각도 보정과 센서 합의

회전 직후 RANSAC 정렬을 끝내도 바닥 마찰 차이와 휠 편차로 후진 중 조금씩 비뚤어질 수 있어, 다음 조건의 보수적인 보정이 추가됐다.

- 후면 평면 법선 오차를 매 스캔 다시 계산한다.
- 3회 연속 품질을 통과한 뒤에만 제어에 사용한다.
- 저역 통과 필터 계수는 0.15다.
- 직전 필터값에서 2.5° 이상 튄 값은 버린다.
- 오차 1° 이내에서는 각속도 명령을 내지 않는다.
- 최대 각속도는 `0.015 rad/s`, 최대 변화율은 `0.030 rad/s²`다.
- 평면 오차가 3° 이상이면 선속도를 0으로 만들고 제자리에서 정렬한다. 오차와 정지 조건이 고유 스캔 3회 연속 안정돼야 후진을 재개한다.
- RANSAC이 끊기거나 품질 조건을 통과하지 못하면 이전 명령을 유지하거나 직진하지 않고 즉시 정지해 최대 3초 동안 재획득한다.
- 두 센서 그룹과 모두 모순되는 평면도 LiDAR-휠 잔차가 4° 이내에서 3개 고유 스캔 동안 안정되면, LiDAR 각도를 조향에 쓰지 않고 엔코더 yaw로 시작 heading을 유지한다.
- 엔코더 heading guard 진입 전에는 정지하고, 4° 초과 불일치·오래된 휠 yaw·8° 휠 drift는 계속 실패 처리한다.
- 차체 후방 보호 간격이 10 cm 이하가 되면 보정을 완전히 끄고 직선 후진한다.
- 유효한 평면이 없을 때 IMU 적분 또는 휠 yaw drift가 8°를 넘으면 안전 실패한다.

RANSAC 품질 조건은 오차 8° 이내, inlier 비율 70% 이상, 직선 길이 0.15 m 이상이다. 후진 시작 및 확인된 제자리 정렬 완료 시 평면의 `plane_error + robot_yaw`를 IMU, odom, 엔코더 전용 yaw 각각에 대해 기록한다. odom은 같은 IMU로 보정되므로 IMU와 하나의 상관된 증거 그룹으로 취급하고, 엔코더 전용 yaw를 독립 그룹으로 사용한다. 한 그룹이 2° 이내에서 일치하면 LiDAR 제어를 유지한다. 두 그룹 모두와 모순되고 후보 오차가 3° 미만일 때는 안정된 LiDAR-휠 잔차 4°까지 엔코더 heading guard로 제한적으로 진행하며, 그 밖의 경우에만 `motion_inconsistent`로 정지·재획득한다.

### 2.8 좌우 가이드 레일 인식 상태

현재 YAML은 `use_lidar_guide_centering: true`이며 메인 노드가 guide estimator를 후진 제어기에 연결한다. 다만 다음 조건을 모두 만족할 때만 경로 보정에 사용한다.

- 후면 패널을 먼저 유효하게 추정해야 한다.
- 패널 정렬 좌표계에서 좌우 레일을 각각 독립된 직선으로 검출해야 한다.
- 두 레일 사이 간격이 `0.44~0.60 m` 범위여야 한다.
- 각 레일은 최소 8개 inlier와 0.15 m 길이를 만족해야 한다.
- 중앙 오프셋이 3개 스캔에서 안정적이고 후방 간격이 0.12 m보다 클 때만 적용한다.
- 오프셋 0.010 m 이내에서는 보정하지 않고 목표 heading은 최대 4°로 제한한다.

한쪽 레일만 보이거나 조건을 만족하지 못하면 `guide_center_offset=unavailable`로 남고 중앙 보정은 0이 된다. 2026-09-17 성공 직전 실패 로그에서도 guide는 unavailable이었으므로, 기본 후면 평면 정렬과 거리 후진은 guide 검출에 의존하지 않는다.

### 2.9 프로세스와 안전 처리

현재 실행 파일은 필요한 하위 스택을 직접 시작하고 종료한다.

- OpenCV 영상 보정 노드와 AprilTag 검출 노드를 별도 프로세스로 시작
- `apriltag_bridge` 시작
- `opennav_docking` 서버 시작
- lifecycle 상태를 확인하고 configure/activate
- 종료 시 로봇 정지 명령을 여러 번 발행
- 하위 프로세스에 `SIGINT → SIGTERM → SIGKILL` 순서로 정리
- `/tmp/stella_dock_turn_backup.lock` 단일 실행 lock으로 중복 실행 차단
- 부모 `ros2 run` 프로세스가 사라지면 자식에도 종료 신호를 받도록 Linux parent-death signal 설정
- 전체 절차 180초 timeout

## 3. 구성 파일과 코드 책임

| 파일 | 책임 |
|---|---|
| `config/docking.yaml` | Nav2 docking server 설정과 현재 시험에서 덮어쓰는 핵심 `dock_turn_backup` 값 |
| `config/tags_36h11.yaml` | AprilTag family, 실제 크기 0.154 m, detector와 QoS 설정 |
| `launch/apriltag_36h11.launch.py` | 영상 보정·태그 검출 프로세스를 시작하고 보정된 image/camera info를 연결 |
| `docking/image_rectifier.py` | 같은 시각의 image/camera info를 OpenCV로 보정; 보정 map 재사용 및 원본 header 유지 |
| `docking/apriltag_bridge.py` | 새롭고 유효한 `base_link <- tag36h11:0` TF만 실제 관측 시각을 유지해 최대 10 Hz로 발행 |
| `docking/dock_turn_backup.py` | 전체 상태 순서, DockRobot action, 태그 정밀 보정, timeout과 exit code |
| `docking/motion.py` | odom 회전, IMU 정지 확인, 선택 직진, LiDAR 거리 후진, 평면·IMU·odom·휠 yaw 합의와 방향 제어 |
| `docking/docking_lidar.py` | `/scan_2` 구독, 시간·frame 검증, TF 해결, base_link 투영 |
| `docking/lidar_geometry.py` | 각도 정규화, 투영, 후단 간격, 연속 빔 군집, 고유 스캔 안정성 계산 |
| `docking/lidar_alignment.py` | 후면 패널 RANSAC, PCA 직선, 평면 획득·추적·재획득, 양쪽 가이드 레일 중심 추정 |
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
| `/camera/camera/color/image_raw` | Image | 왜곡 보정에 입력하는 원본 영상 |
| `/apriltag/image_rect` | Image | AprilTag에 입력하는 왜곡 보정 영상 |
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
| `max_retries` | `1` | 일시적인 태그 유실 시 후퇴 후 자동 재접근 횟수 |
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
| `tag_refinement_recovery_max_angular_speed` | `0.15 rad/s` | fresh gyro·휠이 모두 정지한 무응답 상태에서만 제한적으로 사용 |
| `tag_refinement_max_initial_distance` | `0.18 m` | Nav2의 0.15m 원형 성공 영역과 관측 지연 여유를 포함한 진입 검사 |
| `tag_refinement_translation_heading_limit` | `8°` | 이 방위 오차에서 병진 속도를 0으로 만들고 목표점 방향부터 정렬 |
| `tag_refinement_timeout_sec` | `45.0 s` | 위치 보정 및 최종 방향·정지 안정화 최대 시간 |
| 종방향 tolerance | `0.040 m` | 고정 dock 축의 앞뒤 허용 오차 |
| 횡방향 tolerance | `0.025 m` | 고정 dock 축의 좌우 허용 오차 |
| yaw tolerance | `2°` | 최종 방향 허용 오차 |
| `tag_refinement_stable_cycles` | `5` | 모든 tolerance 동시 만족 연속 주기 |
| 최대 선속도 | `0.025 m/s` | 보정 이동 속도 제한 |
| 최대 각속도 | `0.08 rad/s` | 보정 회전 속도 제한 |
| 최대 초기 오차 | `거리 0.18 m / 계획 yaw excursion 30°` | 목표점 선회와 최종 yaw를 포함해 실제 계획이 운행 범위 안인지 검사 |
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
| inlier 거리 | `0.010 m` | 모서리 혼합을 줄이는 직선 허용 잔차; PCA 후 지지점 재선정 |
| 최소 직선 길이 | `0.15 m` | 짧은 프레임 모서리 배제 |
| 후보 최대 오차 | `15°` | 기대 후면 방향과 다른 후보 배제 |
| 최종 tolerance | `1°` | 정렬 완료 각도 |
| 최대/최소 각속도 | `0.06 / 0.012 rad/s` | 정렬 제어 제한 |
| 획득 안정성 | `3 scans, 3°` | 회전 시작 전 합의 조건 |
| soft/hard 추적 잔차 | `5° / 12°` | 연속 outlier 판정 또는 즉시 안전 중단 경계 |
| soft outlier 합의 | `2 scans` | 한 번 튄 RANSAC 후보는 버리고 2회 연속일 때만 재획득 |
| 기본 보정 회전 | `18°` | 정상적인 단일 평면 정렬 예산 |
| 재획득 회전 여유 | `3°` | 누적 회전과 새 평면 오차에 더하는 정지·감속 여유 |
| hard 최대 누적 회전 | `30°` | 반복 재획득 또는 오인식 폭주 방지 |
| timeout | `18 s` | outlier 정지·완료 5스캔을 포함한 전체 평면 정렬 제한 |

### 5.6 LiDAR 후진과 방향 보정

| 파라미터 | 현재값 | 의미 |
|---|---:|---|
| `use_lidar_backup` | `true` | 거리 기반 후진. `false`면 odom 0.50 m 후진으로 fallback |
| `backup_lidar_sector_width` | `20°` | 완료 거리 영역: 180° ±10° |
| `backup_lidar_safety_sector_width` | `60°` | 보호 영역: 180° ±30° |
| `backup_target_rear_clearance` | `0.0145 m` | LiDAR-벽 0.078 m에 대응하는 차체 후단 목표 간격 |
| `backup_clearance_tolerance` | `0.010 m` | LiDAR-벽 목표의 ±1 cm 성공/overrun 허용 오차 |
| `backup_speed` | `0.035 m/s` | 먼 거리 최대 후진 속도 |
| `backup_min_speed` | `0.012 m/s` | 근거리 최소 속도 |
| `backup_slowdown_clearance` | `0.18 m` | 감속 시작 보호 간격 |
| `backup_max_travel` | `0.60 m` | 이동 안전 상한 |
| `backup_blocked_timeout_sec` | `1.0 s` | 가까워 정지는 했지만 완료 군집이 안 생길 때 실패까지 대기 |
| `use_lidar_heading_during_backup` | `true` | 후진 중 약한 후면 평면 보정 |
| `backup_heading_kp` | `0.35` | 후면 평면 각도 오차 비례 이득 |
| `backup_lidar_heading_filter_coef` | `0.15` | 법선 오차 저역 통과 필터 |
| `backup_heading_tolerance` | `1°` | 이 안에서는 angular 0 |
| `backup_heading_pause_error` | `3°` | 이 이상이면 후진을 멈추고 제자리 정렬 |
| `backup_heading_resume_stable_cycles` | `3` | 오차·정지 조건 만족 후 후진 재개에 필요한 고유 스캔 수 |
| `backup_lidar_heading_max_error` | `8°` | 정지 후 재획득·제어에 사용할 수 있는 RANSAC 오차 상한 |
| `backup_lidar_motion_residual` | `2°` | 고정된 평면의 `오차 + 로봇 yaw` 불변량 허용 편차 |
| `backup_lidar_wheel_guard_residual` | `4°` | 안정된 RANSAC 바이어스에서 LiDAR 조향을 버리고 엔코더 heading만 유지할 수 있는 상한 |
| `backup_lidar_heading_max_jump` | `2.5°` | 단일 측정 급변 거부 |
| `backup_lidar_heading_min_inlier_ratio` | `0.70` | 후진 중 RANSAC 품질 |
| `backup_lidar_heading_stable_cycles` | `3` | 보정 활성화 전 품질 연속 확인 |
| `backup_heading_max_angular_speed` | `0.015 rad/s` | 후진 편향을 이길 수 있는 저속 보정 상한 |
| `backup_heading_max_angular_accel` | `0.030 rad/s²` | 명령 급변 제한 |
| `backup_lidar_heading_disable_clearance` | `0.10 m` | 이보다 가까우면 보정을 끄고 직진 |
| `backup_plane_reacquire_timeout_sec` | `3.0 s` | 평면이 없거나 모순될 때 정지 재획득 제한 |
| `backup_max_yaw_drift` | `8°` | 마지막으로 정렬이 확인된 자세 대비 유효 평면 부재 시 drift 한도 |
| `use_lidar_guide_centering` | `true` | 두 평행 레일이 모두 검출될 때 개구부 중앙 추종 |
| `backup_guide_center_tolerance` / `backup_guide_center_kp` | `0.010 m / 1.0` | 중앙 오프셋 deadband와 목표 heading 이득 |
| `backup_guide_center_max_heading` | `4°` | 레일 중앙 추종이 요청할 수 있는 최대 heading |

후진을 시작할 때 정지 상태의 후면 평면으로 기준 불변량을 한 번만 고정한다. odom yaw는 같은 IMU로 보정되므로 IMU와 독립된 두 번째 증거로 세지 않는다. 평면은 `LiDAR + 엔코더 전용 yaw` 또는 `LiDAR + IMU 계열 yaw` 중 하나와 일치하면 유지한다. 이 판정은 직진 중 IMU 바이어스로 인한 오정지를 피하면서, 실제 차체 미끄러짐처럼 휠 yaw가 놓친 회전도 LiDAR와 IMU가 함께 관측하면 받아들인다. 후면 후보 오차가 3° 미만이고 두 그룹과 모순되는 작은 RANSAC 바이어스에서는 3개 스캔을 확인한 뒤 LiDAR 각속도 명령을 차단하고 휠 heading만 유지한다. 정상 합의된 후면 평면 오차가 3° 이상일 때만 후진을 멈춘 상태에서 천천히 평행 정렬한다.

최종 목표 거리를 바꿀 때는 `backup_target_rear_clearance`를 수정한다. 값을 크게 하면 더 일찍 멈추고, 작게 하면 더 깊이 들어간다. 허용 오차를 너무 작게 하면 물리적으로 도킹했는데도 센서 분해능·노이즈 때문에 완료 군집이 만들어지지 않아 `blocked` 실패가 날 수 있다. 목표와 tolerance를 바꾼 뒤에는 LiDAR 최소 측정거리와 장착 x를 고려한 `completion_range` 검증 로그도 확인해야 한다.

### 5.7 개발 모드와 충전 검증

`development_test_mode`의 코드 기본값은 현재 `true`다.

- `true`: LiDAR 후진 성공 후 충전 접촉을 확인하지 않고 exit 0.
- `false`: `/sk120/available` 접촉, `/sk120/cmd_output=true`, `/battery_state`의 `current >= 0.05 A`가 3초 유지되어야 exit 0.
- `total_timeout_sec`: 현재 YAML 값은 `180.0 s`이며 전체 절차에 적용된다.

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
  -p backup_target_rear_clearance:=0.0145 \
  -p backup_clearance_tolerance:=0.010
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
SERVER_NODE=$(ros2 node list | grep '/stella_docking_.*/docking_server$' | head -n1)
ros2 param get "$SERVER_NODE" \
  simple_charging_dock.external_detection_translation_x
ros2 param get "$SERVER_NODE" \
  simple_charging_dock.docking_threshold
```

`SERVER_NODE`가 비어 있으면 전용 docking server가 아직 시작되지 않았거나 이미 종료된 상태다. 실행 시작 로그의 `Using isolated docking server ...`에서도 같은 전체 노드 이름을 확인할 수 있다.

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
   5.14° 변화처럼 작은 초과도 이전에는 종료됐다. 이후에는 재획득하도록 바꿨지만, 2026-09-18 16:45 실행에서 서로 떨어진 단일 `5.23~5.73°` outlier가 매번 정상 추적 상태까지 지워 timeout을 만들었다. 현재 5~12° 한 번은 정지·무시하고 2회 연속일 때 재획득하며, 12° 초과만 위험한 표면 전환으로 즉시 중단한다.

6. **넓은 후방 안전 sector가 좌우 가이드 레일을 장애물로 판단**  
   스테이션에 진입하면 레일은 정상적으로 로봇 옆에 가까워진다. 과거 ±75° 보호 영역은 이를 포함해 진입 직후 정지시켰다. 현재 차체 통과 폭 안의 ±30° 보호 영역으로 제한했다.

7. **단일 근거리 반사점으로 정지한 뒤 성공하지 못함**  
   `protective_clearance`가 작으면 충돌 방지를 위해 즉시 선속도를 0으로 만든다. 그러나 그 점이 중앙 완료 영역의 연속 5빔·3°·3스캔 조건을 만족하지 않으면 성공할 수 없다. 1초 지속되면 `LiDAR backup remains blocked outside the completion condition`으로 실패한다. 사진상 장애물이 없어 보여도 레일 모서리, 케이블, 차체 반사, 잘못된 TF가 원인이 될 수 있다.

8. **물리 성공 위치와 코드 목표가 어긋남**
   초기 차체 후단 목표 1.0 cm, tolerance 0.5 cm는 실제 도킹 위치를 충분히 포함하지 못했다. 실물 전체 도킹 성공 후 40개 정지 스캔에서 LiDAR-벽 거리가 약 7.8 cm로 측정됐다. 현재는 장착 오프셋 6.35 cm를 반영해 차체 후단 목표를 1.45 cm로 두며, 요청한 ±1 cm 범위를 적용한다.

9. **후진 중 각도 오차를 주행하면서 보정**
   LiDAR 법선이 크게 틀어진 상태에서 계속 후진하면 스테이션 안에서 곡선 궤적이 생겼다. 현재 3° 이상은 선속도를 0으로 만들고 제자리 정렬하며, 작은 오차만 최대 0.015 rad/s로 보정한다. 1° deadband, LPF, jump reject를 적용하고 10 cm 이내에서는 직선만 사용한다.

10. **태그 정밀 목표 QoS 불일치**  
    `/dock_pose` 발행자는 VOLATILE인데 구독자가 TRANSIENT_LOCAL이면 연결이 호환되지 않아 `No fresh filtered Nav2 dock pose`가 발생할 수 있었다. 현재 VOLATILE로 수정했다.

11. **action 즉시 완료와 목표 콜백 경쟁**  
    이미 성공 반경 안에서 재실행하면 action 결과가 `/dock_pose` 콜백보다 먼저 처리될 수 있었다. 최대 1초 bounded wait를 추가했다.

12. **정밀 보정 성공 오차를 회전하는 base 좌표로 판정**  
    제자리 yaw 보정만 했는데도 좌우 위치 오차가 변한 것처럼 보이는 문제가 있었다. 성공 판정은 고정 target 축에서 하고, 속도 계산만 현재 base 축에서 한다.

13. **정리 시 AprilTag container `exit code -11`**  
    도킹 실패 뒤 발생한 별도 종료 오류다. 2026-09-09 합성 영상 시험에서도 설치된 `image_proc` 노드의 종료 crash를 재현했다. 현재는 이미지 전송 플러그인을 사용하지 않는 `docking/image_rectifier`와 독립 AprilTag 프로세스로 교체했다. 통합 검증은 자식 프로세스가 정상 종료하는지도 검사한다.

14. **정상 직진 후진을 `motion_inconsistent`로 오판**
    2026-09-17 로그에서 휠 yaw는 `-0.05°`, LiDAR 기준 변화는 약 `+0.65°`로 직진에 합의했지만 raw IMU와 IMU 보정 odom 잔차가 각각 `+2.17°`, `+2.09°`가 되어 멈췄다. odom을 IMU와 독립된 증거로 센 것이 원인이었다. 현재는 IMU/odom을 하나의 상관 그룹으로 묶고 엔코더 전용 yaw를 독립 그룹으로 비교한다. 이 판정 수정 후 실물 전체 도킹 성공을 확인했다.

15. **이동에 따라 RANSAC 각도가 바뀌어 두 센서 그룹과 모두 불일치**
    2026-09-18 로그에서 정지 평면은 약 0.7°였지만 2 cm 후진 뒤 후보가 3.4°로 바뀌었다. IMU/odom 바이어스 약 2.1°와 합쳐 관성 잔차는 약 5°, 직진을 나타낸 휠과의 잔차도 2.7~3.0°가 되어 기존 2° 한계를 넘었다. 같은 후보가 정지 후에도 반복됐으므로 재획득이 불가능했다. 현재는 4° 이내의 안정된 불일치만 엔코더 heading guard로 처리하며 이때 LiDAR 각도는 조향에 사용하지 않는다.

16. **단일 RANSAC outlier가 평면 추적을 반복 초기화**
    2026-09-18 16:45 로그에서 180° 회전은 목표 오차 0.13°로 정상 완료됐다. 평면 정렬도 마지막에 `1.42 → 1.33 → 0.97 → 0.18 → 0.55°`로 수렴했지만, 앞서 한 스캔씩 나타난 `-4.1°` 후보가 motion-compensated 5° soft limit를 근소하게 넘어 획득 3스캔과 완료 5스캔 상태를 다섯 번 초기화했다. 현재는 단일 soft outlier를 정지·무시하고 두 번 연속일 때만 재획득하며 정렬 제한을 18초로 늘렸다.

### 8.3 대표 오류별 조치

| 오류 | 직접 의미 | 우선 확인 |
|---|---|---|
| `No fresh filtered Nav2 dock pose` | 정밀 목표가 없거나 1.5초보다 오래됨 | `/dock_pose` hz/QoS, docking server가 같은 설정으로 실행됐는지 |
| `Refusing an unexpectedly large tag refinement` | 초기 평면 거리 또는 yaw가 안전 범위 밖 | 로그의 `distance`, 태그 TF 축, `external_detection_*`, 실제 시작 위치 확인 |
| `Tag pose refinement timed out` | 45초 내 tolerance 안정화 실패 | 오차 로그가 수렴하는지, odom 지연, 너무 작은 tolerance |
| `Spin step failed or timed out` | 절대 목표 또는 정지 연속 조건 미달 | `/odom` yaw, `/imu/data` age/rate, 모터 최소 속도 |
| `rear panel RANSAC is invalid` | 점·inlier·길이·방향 조건 미달 | `/scan_2` 시각화, 180° ±30° 안에 패널이 있는지, TF yaw |
| `Ignoring isolated inconsistent ... retaining` | 단일 soft 추적 outlier | 로봇은 정지하고 기존 평면을 유지하며 다음 고유 스캔을 확인 |
| `Consecutive inconsistent ... reacquiring` | soft 추적 outlier 2회 연속 | 로봇은 정지한 채 평면 3스캔을 다시 획득 |
| `Rejected backup plane inconsistent with robot motion` | 평면이 IMU 계열과 휠 yaw 그룹 모두에 모순 | `imu/odom/wheel_plane_residual`, RANSAC 대상, 센서 freshness 확인 |
| `continuing with encoder-only heading guard` | 평면 잔차가 2~4°지만 휠 yaw가 안정돼 LiDAR 조향 없이 직선 후진 | `wheel_residual ≤ 4°`, `wheel_drift < 8°`, 3개 안정 스캔 확인 |
| `Backup translation paused for heading correction` | 유효한 평면 오차가 3° 이상 | 정지 상태에서 자동 정렬하며 `settled` 로그 후 재개 |
| `tracking changed too far` | 12° 이상 다른 구조로 전환 | 후면 패널 가시성, 프레임/레일 오인식, 회전 오차 |
| `alignment exceeded ... rotation budget` | 동적 예산 또는 30° hard limit 초과 | odom 180도 결과와 LiDAR TF를 먼저 수정 |
| `Wheel-only yaw diagnostics ... unavailable` | 독립 휠 yaw 안전 기준 없음 | `/wheel_odometry/yaw_diagnostics` 노드와 hz |
| `Docking LiDAR failed during backup` | scan stale/frame/time 오류 | `/scan_2`, sensor timestamp, `base_scan2` |
| `lost its dock plane and exceeded ... drift` | RANSAC 없이 상대 yaw가 8° 초과 | 바퀴 미끄러짐, IMU/휠 데이터, 패널 가시성 |
| `remains blocked outside completion` | 안전상 정지했지만 성공 군집 불충족 | 중앙·보호 clearance 차이, 레일/반사점, 완료 sector |
| `target was overrun` | 차체 후단 0.45 cm, 즉 LiDAR-벽 6.8 cm보다 가까운 유효 군집 | 즉시 물리 간격 확인, 목표를 더 작게 바꾸지 말 것 |
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

현재 실물 성공 구성은 정밀 태그 보정을 켜고 시간 기반 직진을 끈 상태다.

```yaml
dock_turn_backup:
  ros__parameters:
    use_tag_pose_refinement: true
    use_pre_spin_forward: false
    use_lidar_alignment: true
    use_lidar_backup: true
    use_lidar_heading_during_backup: true
    use_lidar_guide_centering: true
```

문제 분리를 위해 개별 기능을 끌 수는 있지만 위 값을 한꺼번에 변경하면 2026-09-17 실물 성공 구성과 달라진다. 진단 시에는 한 기능만 바꾸고 해당 단계 로그를 비교한다.

## 10. 테스트

패키지 단위 테스트는 다음을 검증한다.

- 후방 LiDAR 토픽·frame·sector 기본 계약
- LaserScan timestamp, freshness, QoS, TF 투영
- LiDAR 장착점과 차체 후단 기준의 완료 range 계산
- 인접 빔 군집 및 고유 스캔 안정성
- odom 절대 180도 목표와 감속 명령
- IMU timestamp 기반 yaw 적분과 정지 판정
- 후진 중 LPF, jump reject, deadband, angular rate limit
- 큰 후진 heading 오차의 정지 보정과 안정화 후 재개
- IMU/odom 상관 바이어스가 있어도 LiDAR와 휠 yaw가 합의하면 계속 후진하는 실제 제어 루프
- 휠 미끄러짐 때 LiDAR와 관성 그룹이 합의한 평면을 유지하는 센서 중재
- 2026-09-18의 3.4° RANSAC 이동 바이어스에서 잠시 정지 후 엔코더 heading guard로 완료하는 실제 후진 루프와 4°/8° 차단 경계
- 긴 후면 패널이 짧고 조밀한 프레임 모서리보다 선택되는지
- 회전 중 동일 평면의 예상 법선 변화 보상
- 실제 실패 로그의 13.54°→6.22° 획득 jump와 5.14° tracking jump 회귀
- 태그 정밀 보정의 고정 목표축 오차와 안전 envelope
- TF 캐시 중복 발행 방지, 촬영 시각 보존, 잘못된 시간·좌표계·quaternion 거부
- `base_footprint`와 `base_link`의 평면상 일치 여부를 TF로 확인
- 정밀 보정 실제 루프의 오래된 목표 거부 및 고유 정지 odom 확인
- 좌우 가이드 양쪽 검출과 한쪽 검출 거부
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

2026-09-18 18:11 실패 분석에서는 정지 실측 점군 및 두 차례 정렬 후 재획득 시험을 추가했다. `docking` 테스트 **245개 통과, 오류 0, 실패 0**를 확인했다. 패키지 빌드와 설치 코드 확인도 완료했다. 실물 도킹 재시험은 아직 수행하지 않았다.

합성 영상으로 왜곡 보정 → AprilTag 검출 → bridge까지 확인:

```bash
source ~/colcon_ws/install/setup.bash
cd ~/colcon_ws/src/STELLA_N5_ROS2/docking
python3 test/check_apriltag_pipeline.py
```

이 스크립트는 별도 DDS domain 217에서 합성 카메라와 인식 노드만 실행한다. 실제 로봇 속도 명령을 발행하지 않으며, 원래 촬영 시각 유지와 영상 공급 중단 후 과거 태그 재발행 방지도 확인한다.

기본 제어 스택은 2026-09-17 실물 전체 도킹에 성공했다. 이후 반영한 7.8 cm 거리 계약과 2026-09-18 엔코더 heading guard는 로그 기반 실제 제어 루프와 단위·통합 시험을 통과했지만 아직 실물 전체 순서로 재검증하지 않았다. 실제 로봇에서는 단계별 로그와 물리 비상 정지를 계속 사용한다.

## 11. 종료 코드

| 코드 | 의미 |
|---:|---|
| `0` | 현재 모드의 성공 조건 만족 |
| `1` | 처리되지 않은 내부 오류 또는 cleanup 오류 |
| `2` | 잘못된 요청/파라미터 또는 중복 실행 |
| `3` | 센서, TF, action server, base 상태 준비 실패 |
| `4` | 태그 접근·정밀 보정·회전·RANSAC·후진 중 도킹 실패 |
| `5` | 전체 180초 timeout |
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
- 양쪽 가이드 레일이 품질 조건을 만족할 때만 중앙 추종을 추가하고, 검출되지 않아도 후면 평면 제어로 계속 진행한다.
- 후진 중 작은 오차는 최대 0.015 rad/s로 보정하고, 3° 이상이면 정지 상태에서 평행 정렬한다.
- LiDAR+휠 yaw 또는 LiDAR+IMU 계열 중 하나가 일치하면 정상 평면을 유지한다.
- 두 그룹과 모두 모순되는 4° 이내의 안정된 RANSAC 바이어스에서는 LiDAR 조향을 차단하고 엔코더 heading guard로 직선 후진한다.
- 10 cm 이내에서는 angular 보정을 끄고 직선 후진한다.
- LiDAR-벽 간격 6.8~8.8 cm, 즉 차체 후단 간격 0.45~2.45 cm의 신뢰 가능한 군집을 3회 확인하면 성공한다.
- 기본은 개발 시험 모드이므로 실제 충전 확인은 생략한다.

이 구성에서 문제가 생기면 안전 한도를 무작정 넓히기보다, 로그에서 **태그 접근 → 정밀 보정 → odom 회전 → RANSAC 획득/추적 → 후진 거리 군집 → 충전 확인** 중 최초로 실패한 단계를 찾아 그 단계의 센서와 파라미터만 조정해야 한다.

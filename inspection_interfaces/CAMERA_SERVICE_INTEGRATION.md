# 카메라 촬영 서비스 연동 명세

이 문서는 `drive_manager`와 로봇의 `gimbal_camera_capture`가 한 번의 순회와 그 안의
사진들을 손실 없이 연결하기 위한 ROS 2 계약입니다. 공용 타입은 이 워크스페이스의
`inspection_interfaces` 패키지에 정의되어 있습니다.

## 1. 서비스 목록과 호출 순서

| 서비스 | 타입 | 호출 시점 |
| --- | --- | --- |
| `/camera/capture_run/start` | `inspection_interfaces/srv/StartCaptureRun` | START 미션의 AMCL/Nav2 준비가 끝난 직후 |
| `/camera/capture_pair` | `inspection_interfaces/srv/CapturePair` | 활성 구역 진입 또는 마지막 촬영 위치에서 설정 거리만큼 이동한 뒤 정지했을 때 |
| `/camera/capture_run/finish` | `inspection_interfaces/srv/FinishCaptureRun` | 복귀 goal 도착 및 마지막 capture 응답 완료 후 |
| `/camera/capture_run/abort` | `inspection_interfaces/srv/AbortCaptureRun` | STOP, ESTOP, HOME 전환, 주행/촬영 오류 발생 시 |

정상 순서는 반드시 `start -> capture 0회 이상 -> finish`입니다. `drive_manager`는
capture 요청을 한 번에 하나만 전송하며 `CapturePair` 응답을 받은 뒤에만 다음 요청을
허용합니다. finish 전에도 진행 중인 요청이 없어질 때까지 기다립니다.

기존 Empty/Bool 토픽은 수동 시험용 legacy 인터페이스로 남길 수 있지만 자동 순회는
위 서비스를 사용해야 합니다. 서비스와 legacy 토픽의 이름이 겹치지 않도록 서비스는
`capture_run`과 `capture_pair` 이름을 사용합니다.

## 2. 공용 인터페이스 설치

로봇 워크스페이스에도 `inspection_interfaces` 디렉터리를 그대로 복사해야 합니다.

```bash
cp -a /path/to/inspection_interfaces \
  /home/user/colcon_ws/src/inspection_interfaces

cd /home/user/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select inspection_interfaces
source install/setup.bash
ros2 interface show inspection_interfaces/srv/CapturePair
```

`gimbal_camera_capture/package.xml`에는 다음 실행 의존성을 추가합니다.

```xml
<exec_depend>inspection_interfaces</exec_depend>
```

Python 코드에서는 다음 타입을 사용합니다.

```python
from inspection_interfaces.srv import (
    AbortCaptureRun,
    CapturePair,
    FinishCaptureRun,
    StartCaptureRun,
)
```

## 3. StartCaptureRun 서버 동작

요청에는 `mission_id`, 실제 지도 식별자, map 프레임, 구역 revision, 그리고 해당
run에서 사용할 활성 구역 좌표 스냅샷이 들어옵니다. 구역 개수는 고정되어 있지 않으며
구역 좌표는 정규화된
`min_x/min_y/max_x/max_y`이며 단위는 meter입니다.

`drive_manager` YAML의 `capture_zone_names`에 나열된 구역만 start 요청에 포함합니다.
촬영 기능을 사용하지 않을 때는 `capture_enabled: false`로 설정하므로 카메라 서버가 받는
start 요청에는 항상 활성 구역이 1개 이상 있습니다.

서버는 다음 순서로 처리해야 합니다.

1. 활성 run이 없는지 확인합니다.
2. `mission_id`가 비어 있지 않고 활성 구역이 1개 이상인지 검사합니다.
3. 각 구역 ID가 비어 있지 않고 중복되지 않으며 좌표가 유한하고 면적이 0보다 큰지
   검사합니다. 특정 이름이나 최대 개수로 제한하지 않습니다.
4. 기존 저장 로직으로 날짜 디렉터리와 다음 `run_N`을 할당합니다.
5. 상태가 `running`인 `metadata.yaml`을 생성하고 요청의 지도·구역 스냅샷을 기록합니다.
6. 메모리에 `mission_id -> run_id` 활성 상태를 저장합니다.
7. 성공 응답에 할당된 `run_id`를 반환합니다.

같은 `mission_id`로 start가 재호출되면 새 run을 만들지 말고 기존 `run_id`를 성공으로
반환하는 idempotent 처리를 권장합니다. 다른 mission의 run이 활성 상태라면 실패해야
합니다.

응답 성공 조건은 `success=true`, 비어 있지 않은 `run_id`입니다.

### 로봇 카메라의 동적 구역 검증

`gimbal_camera_capture/capture_node.py`에 `A~E`, `len(zones) == 5` 또는
`set(zone_ids) == {"A", "B", "C", "D", "E"}` 같은 고정 검사가 있다면 제거해야 합니다.
대신 start callback에서 아래와 같은 규칙으로 검사합니다.

```python
import math
import re


ZONE_ID_PATTERN = re.compile(r"[A-Za-z0-9_-]+")


def validate_zones(zones):
    if not zones:
        return False, "at least one capture zone is required"

    seen = set()
    for zone in zones:
        zone_id = zone.id.strip()
        if not zone_id or ZONE_ID_PATTERN.fullmatch(zone_id) is None:
            return False, f"invalid zone id: {zone.id!r}"
        if zone_id in seen:
            return False, f"duplicate zone id: {zone_id}"

        coordinates = (zone.min_x, zone.min_y, zone.max_x, zone.max_y)
        if not all(math.isfinite(value) for value in coordinates):
            return False, f"non-finite coordinates: {zone_id}"
        if zone.min_x >= zone.max_x or zone.min_y >= zone.max_y:
            return False, f"zone area must be positive: {zone_id}"
        seen.add(zone_id)

    return True, ""
```

검증 성공 후에는 `zone_id -> 좌표` 전체를 활성 run 상태와 `metadata.yaml`에 그대로
저장합니다. `CapturePair` callback은 더 이상 A~E를 검사하지 않고 다음처럼 start 때
저장한 ID 집합만 검사해야 합니다.

```python
if request.zone_id not in self.active_zone_ids:
    return failure_response(f"unknown zone_id: {request.zone_id}")
```

`storage.py`가 구역을 반복문으로 YAML에 기록하고 있다면 저장 구조 변경은 필요 없습니다.
고정된 `A`, `B`, `C`, `D`, `E` 키를 직접 생성하고 있다면 요청으로 받은 구역만 기록하도록
바꿔야 합니다.

## 4. CapturePair 서버 동작

요청의 핵심 식별자는 `request_id`입니다. 예시는
`mission_123456_capture_000001`이며 한 mission 안에서 절대 중복되지 않습니다.

서버는 다음 순서로 처리해야 합니다.

1. `run_id`와 `mission_id`가 현재 활성 run과 일치하는지 확인합니다.
2. `zone_id`가 start 요청에서 받은 활성 구역 중 하나인지 확인합니다.
3. 같은 `request_id`가 이미 처리됐다면 재촬영하지 말고 저장된 기존 결과를 반환합니다.
4. 촬영 중 상태를 설정하고 Left/Right 한 쌍을 촬영합니다.
5. 두 파일을 run 디렉터리 아래 상대경로로 저장합니다.
6. 실제 촬영 시각 `captured_at`과 요청 문맥을 `metadata.yaml`에 원자적으로 기록합니다.
7. metadata 저장까지 끝난 후에만 서비스 응답을 반환합니다.

`requested_at`은 drive_manager가 명령을 만든 시각이고 `robot_pose.header.stamp`는 사용한
AMCL pose의 시각입니다. `captured_at`은 카메라가 실제 이미지를 획득한 시각으로 카메라
서버가 기록해야 합니다. 서로 다른 PC라면 chrony/NTP로 시계를 동기화해야 하지만,
사진 연결의 기본 키는 시간이 아니라 `request_id`입니다.

응답은 요청의 `run_id`와 `request_id`를 그대로 echo해야 합니다. `left_file`과
`right_file`은 `left/153633_1.jpg`처럼 run 디렉터리 기준 상대경로입니다. 한쪽만
실패하면 성공한 파일 경로와 실패한 쪽의 error를 모두 기록하되 `success=false`로
응답합니다.

## 5. FinishCaptureRun 서버 동작

finish는 현재 run의 모든 파일과 metadata가 디스크에 기록된 뒤 run을 전송 가능한
상태로 바꾸는 commit 동작입니다.

1. `run_id`와 `mission_id` 일치를 검사합니다.
2. 촬영이 진행 중이면 `success=false`, `ready=false`로 거부합니다.
3. summary와 `finished_at`, `status: completed`를 metadata에 기록하고 fsync/원자 교체합니다.
4. metadata 기록이 성공한 뒤 마지막 단계로 `READY` 파일을 생성합니다.
5. `success=true`, `ready=true`, 절대 directory와 metadata 경로를 반환합니다.

동일 run의 finish 재호출은 기존 READY와 경로를 그대로 반환해야 합니다. READY는
반드시 metadata보다 나중에 생성해야 Jetson이 미완성 run을 가져가지 않습니다.

## 6. AbortCaptureRun 서버 동작

STOP, ESTOP, HOME 전환, Nav2 실패 또는 카메라 실패로 START 미션이 정상 완료되지 않으면
abort를 호출합니다.

1. `run_id`와 `mission_id`를 검사합니다.
2. `status: aborted`, `finished_at`, `abort_reason`을 metadata에 기록합니다.
3. `READY`가 있다면 제거하고 새 READY를 만들지 않습니다.
4. 활성 run 상태를 해제합니다.
5. 같은 abort 재호출은 이미 중단된 run 정보를 성공으로 반환합니다.

중단된 run의 사진을 보존할지는 현재 정책처럼 보존하는 것을 권장합니다. 추론 대상은
READY가 있는 run으로 제한합니다.

## 7. 카메라 노드 상태 머신

```text
IDLE --start 성공--> ACTIVE --capture 시작--> CAPTURING
  ^                    ^                         |
  |                    +---- capture 완료 -------+
  |
  +---- finish 성공(READY 생성) <--- ACTIVE
  +---- abort(READY 없음) <---------- ACTIVE
```

상태와 storage 쓰기는 하나의 lock으로 보호해야 합니다. 서비스 callback을 병렬로
실행한다면 `ReentrantCallbackGroup`과 `MultiThreadedExecutor`를 사용하되, 실제 촬영은
동시에 한 건만 허용해야 합니다. 더 단순하게 SingleThreadedExecutor를 사용해도 되지만
긴 촬영 중 다른 상태 조회가 막힐 수 있습니다.

서비스 등록 예시는 다음과 같습니다.

```python
self.start_service = self.create_service(
    StartCaptureRun,
    "/camera/capture_run/start",
    self.start_run_callback,
)
self.capture_service = self.create_service(
    CapturePair,
    "/camera/capture_pair",
    self.capture_pair_callback,
)
self.finish_service = self.create_service(
    FinishCaptureRun,
    "/camera/capture_run/finish",
    self.finish_run_callback,
)
self.abort_service = self.create_service(
    AbortCaptureRun,
    "/camera/capture_run/abort",
    self.abort_run_callback,
)
```

## 8. metadata.yaml schema version 2

run 디렉터리만 Jetson으로 옮겨도 구역과 위치를 복원할 수 있도록 다음 정보를 같은
metadata에 저장해야 합니다.

```yaml
schema_version: 2
run_id: run_1
mission_id: mission_1788422000000000000
date: '20260903'
status: completed
started_at: '2026-09-03T15:36:33+09:00'
finished_at: '2026-09-03T16:10:20+09:00'
map:
  id: map_0903
  frame_id: map
zone_config:
  revision: yaml_v1
  assignment_rule: robot_base_at_request
  zones:
    greenhouse_1: {min_x: -4.70, min_y: -4.66, max_x: -2.97, max_y: -1.94}
    north-bed: {min_x: 0.0, min_y: 0.0, max_x: 1.0, max_y: 1.0}
summary:
  capture_requests: 2
  successful_pairs: 2
  partial_pairs: 0
  failed_requests: 0
  image_files: 4
captures:
- capture_id: 1
  request_id: mission_1788422000000000000_capture_000001
  zone_id: greenhouse_1
  requested_at: '2026-09-03T15:36:33.100+09:00'
  captured_at: '2026-09-03T15:36:33.125+09:00'
  robot_pose:
    frame_id: map
    stamp: '2026-09-03T15:36:33.080+09:00'
    source: amcl
    x: -3.20
    y: -2.10
    yaw: 1.57
    covariance: [/* PoseWithCovariance의 36개 값 */]
  success: true
  files:
    left: left/153633_1.jpg
    right: right/153633_1.jpg
  errors: {}
```

`zone_id`는 현재 "촬영 요청 시점에 로봇 중심이 속한 구역"을 뜻합니다. 카메라가
좌우의 서로 다른 재배 구역을 바라본다면 추후 `files.left.observed_zone_id`와
`files.right.observed_zone_id`를 별도로 추가해야 합니다.

## 9. 추론 결과 계약

원본 촬영 metadata는 변경하지 말고 Jetson이 `inference.yaml`을 별도로 생성하는 것을
권장합니다. 각 검출은 `(run_id, capture_id, request_id, camera)`로 원본 사진과 연결하고
반드시 `zone_id`를 복사합니다. 모델 이름, 버전, weight hash, confidence threshold도
기록해야 재현할 수 있습니다.

같은 병변이 거리 간격 사진과 Left/Right에 반복될 수 있으므로 raw detection 개수를
병해 개체 수로 사용하면 안 됩니다. 초기 구역 요약은 `positive_capture_pairs`,
`total_successful_pairs`, `max_confidence` 중심으로 만들고, 개체 수가 필요할 때 위치
클러스터링이나 tracking을 추가합니다.

## 10. 필수 시험 항목

1. start 두 번 호출 시 run 디렉터리가 중복 생성되지 않는지
2. 같은 request_id 두 번 호출 시 사진이 중복 촬영되지 않는지
3. Left 또는 Right 실패가 response와 metadata 양쪽에 남는지
4. capture 처리 중 finish가 READY를 만들지 않는지
5. 마지막 capture 응답 후 finish가 READY를 만드는지
6. STOP/ESTOP/HOME abort run에 READY가 없는지
7. 노드 재시작 후 미완성 `status: running` run을 READY로 오인하지 않는지
8. run 디렉터리를 다른 PC로 복사해도 상대 이미지 경로가 유효한지
9. request의 zone/pose와 metadata의 값이 정확히 같은지
10. 설정한 활성 구역의 경계 좌표에서 drive_manager 판정 결과가 예상과 같은지

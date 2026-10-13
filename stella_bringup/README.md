# Bringup 지연 분석 및 안정화 (2026-10-06)

## 2026-10-12 DDS 반복 오류 출력 제한

`ddsi_udp_conn_write ... retcode -5`는 CycloneDDS의 송신 자원 부족 반환이다.
기존 UDP guard가 큐 포화의 `EAGAIN`을 `ENOBUFS`로 반환하는 경로에서도
발생한다. 블로킹 송신으로 복원하면 센서 처리가 다시 밀릴 수 있으므로
nonblocking 및 실패 반환은 유지한다. `dds_log_throttle.cpp`는 이 한 종류의
반복 출력을 프로세스당 30초에 한 번 자원 부족 요약 경고로 제한한다.
다른 UDP 오류, DDS 초기화 오류, 로그 심각도와 긴 메시지는 유지한다.
네트워크 인터페이스·발견 범위·QoS·watchdog은 변경하지 않았다.

설치된 실제 CycloneDDS 0.10.5 라이브러리에서 1,000개 자원 부족 로그가
요약 경고 1개로 줄고 별도의 `retcode -3` 오류는 즉시 출력됨을 확인했다.
동시 출력 4,000개 및 원본 출력 복원 검사, 기존 UDP 실패 반환·Python ROS
초기화 검사도 통과했다. 읽기 전용 센서 수집은 정상 수신했고 이동·충전
명령을 발행하지 않았다. 실제 통신 혼잡 자체가 해소됐다는 의미는 아니다.

새 도킹 실행과 그 자식 프로세스는 설치된 보호 라이브러리를 바로 사용한다.
기존 bringup은 종료하지 않았으며 **다음 bringup 재시작부터** 출력 제한이
적용된다. 전체 원본 로그가 필요하면 실행 환경에
`STELLA_DDS_LOG_THROTTLE=0`을 지정한다.
상세: `/home/user/docking_diagnostics/2026-10-12/user79_analysis.md`.

## 2026-10-08 원본 영상 수신 공백과 로컬 DDS 경로 보완

정지 상태 카메라 발행 함수 계측에서 영상 수신 공백 9.345초 동안 compressed
영상/CameraInfo 각각 276회 발행이 완료됐다. 영상 발행 함수는 그 구간 최대
5.94ms였고 `rcl_publish` 반환 오류는 없었다. 따라서 카메라가 태그를 못 보거나
촬영을 멈춘 현상만으로 설명되지 않는 DDS/UDP 전달 누락이다. 시스템 UDP
송신 버퍼 오류와 카메라 자체 송신 큐 포화를 함께 확인했다.

`dds_use_loopback: true`를 추가했고 다음 bringup 시작 시 설치된
`param/cyclonedds_local_control.xml`을 선택한다. lo/wlan0 각각의 DDS 송신
연결을 사용해 Wi-Fi 송신 큐와 Pi 내부 전달을 분리한다. wlan0와 외부 ROS
발견을 유지한다. 원래 설정으로 로컬 IP에 보내는 패킷도 Linux의 lo 경로를
쓸 수 있었으나 DDS의 송신 소켓/버퍼가 외부 통신과 공유됐다는 점이 차이다.
[사용 버전의 인터페이스별 송신 소켓 생성](https://github.com/eclipse-cyclonedds/cyclonedds/blob/0.10.5/src/core/ddsi/src/q_init.c#L1714)

기존 기본 `/etc/cyclonedds.xml` 또는 미지정 URI에서 domain 0/CycloneDDS에만
자동 적용한다. 사용자 지정 URI, 다른 RMW, 격리 domain/LOCALHOST 환경은 보존한다.
`STELLA_DDS_LOOPBACK=0`으로 bringup/SSH 도킹 모두 이전 DDS 설정을 유지한다.
bringup YAML `dds_use_loopback: false`는 bringup 측 자동 선택만 비활성화한다.
기존 UDP guard/센서 주기/QoS/모터 watchdog은 유지한다. `/etc`와 커널 버퍼
설정은 바꾸지 않았다. SSH 도킹도 같은 설치 프로필을 RMW 초기화 전에 선택한다.

90초 정지 시험 2회에서 원본 영상 최대 수신 공백은 0.088/0.106초였다.
기존 설정 수신기 호환도 확인했다. 비교 조건이 완전히 같지 않고 이후 기존
설정도 정상인 구간이 있어 간헐적 문제의 영구 해결을 확정하지 않는다.
50개 관련 검사와 docking/stella_bringup 빌드는 통과했다. 최종 배포 설정의
AprilTag 연속 측정은 기존 bringup의 15:59:43 KST SIGINT 종료로 중단됐다.
노드를 다시 켜 전체 센서/영상/PC navigation 부하를 확인해야 한다.
자세한 근거: `/home/user/docking_diagnostics/2026-10-08/camera_dds_root_cause.md`.

HOME_TO_DOCK 실패 시각은 `1791261091.718892778`, 모터 watchdog 정지 로그는
`1791261091.858177865`다. 이 정지 로그는 해당 navigation 실패보다 약 0.139초 늦다.
따라서 이 로그가 앞선 실패를 일으켰다는 근거는 없다. Jazzy FollowPath의 `102`는
`TF_ERROR`다. 실제 PC controller 로그는 이 환경에서 권한 때문에 읽지 못했으므로,
PC에서 전달된 102의 발생 위치는 controller 로그와 함께 확인해야 한다.

참고: [Nav2 Jazzy FollowPath.action](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_msgs/action/FollowPath.action)

## 확인한 지연 경로

실제 Raspberry Pi에서 기존 bringup을 정지 상태로 145초 관찰했다.

| 토픽 | 최대 수신 간격 | 최대 stamp age |
| --- | ---: | ---: |
| `/wheel/encoders` | 1.300초 | 1.278초 |
| `/imu/data` | 1.926초 | 1.918초 |
| `/odom` | 2.216초 | 1.548초 |
| `/scan` | 1.829초 | 1.829초 |
| `/scan_2` | 1.999초 | 1.999초 |

동시에 `/proc/<pid>/task/<tid>/wchan`을 기록했다. 모터 executor, IMU publisher,
wheel_odometry executor 및 DDS 전송 스레드에서 `sock_alloc_send_pskb`가 검출됐다.
즉, **네트워크 송신 큐의 대기가 로컬 센서 발행/처리까지 막는 경로**를 확인했다.
Pi에서 encoder 수신이 끊겼다는 사실만으로 USB encoder 입력 자체가 중단됐다고
단정할 수 없다. 당시 publisher도 네트워크 송신에 막힐 수 있기 때문이다.

설치된 DDS는 CycloneDDS 0.10.5다. 해당 Linux UDP transport는 blocking socket을
사용하고, `EAGAIN`을 받으면 시간 제한 없이 즉시 재시도한다. 단순히 UDP를
nonblocking으로 바꾸는 것만으로는 busy loop가 생길 수 있다.

참고: [CycloneDDS UDP transport](https://github.com/eclipse-cyclonedds/cyclonedds/blob/releases/0.10.x/src/core/ddsi/src/ddsi_udp.c),
[POSIX socket 오류 처리](https://github.com/eclipse-cyclonedds/cyclonedds/blob/releases/0.10.x/src/ddsrt/src/sockets/posix/socket.c)

측정 중 NetworkManager `LastScan`은 변하지 않았다. 반복되는 송신 지연이
Wi-Fi power saving, 드라이버, AP, 외부 DDS reader 또는 다른 부하 중 무엇 때문에
발생하는지는 아직 확정하지 않았다. 시스템 네트워크 설정은 변경하지 않았다.

모터 vendor archive를 분석한 결과, `ReadPacket`이 `Serial::available()`을 sleep 없이
반복 호출하며 timeout 인자를 사용하지 않는 것도 확인했다. 기존 모터 프로세스는
정지 상태에서도 CPU 한 코어의 약 79%를 사용했다.

## 적용한 변경

- 모터 port는 전용 serial worker만 사용한다. `/cmd_vel` callback은 최신 명령과
  수신 시각만 저장하며, worker가 명령 적용과 정지 watchdog을 처리한다.
- encoder query 사이에도 watchdog을 확인한다. DDS 발행과 `/rosout` 때문에 stop
  처리가 같이 막히지 않도록, worker는 ROS 토픽을 발행하지 않고 모터 노드는
  `/rosout`을 비활성화한다. 콘솔/파일 로그는 유지한다.
- `Serial::available()`은 데이터가 없으면 최대 2ms 동안 readable 상태를 기다린다.
  각 runtime transaction에는 steady-clock deadline과 serial timeout을 적용한다.
  초기 연결도 15초 deadline을 적용해 무한 대기를 방지한다.
- 불완전하거나 96바이트 한계를 넘는 ASCII 응답은 오류로 처리한다. vendor parser의
  100바이트 버퍼에 긴 응답이 복사되는 것도 막는다. 실패한 encoder 쌍은 발행하지 않는다.
- 시작/정상 종료 때 정지 명령을 보낸다. runtime 명령 실패 시 worker가 재시도한다.
- 완료된 encoder 쌍을 최신 값 하나로 전달하며 ROS executor가 10ms 간격으로 확인한다.
  새 query가 없으면 발행하지 않는다. 측정 stamp는 좌우 query 시작/끝 시각의 중간이다.
- IMU/encoder/odom publisher는 reliable 호환성을 유지하면서 history를 1로 줄인다.
  wheel_odometry의 입력은 sensor QoS, 동적 TF 출력은 reliable history 1을 사용한다.
  IMU publisher 주기는 ROS time 대신 wall time을 사용한다.
- Linux bringup 자식 프로세스에 `libstella_dds_udp_guard.so`를 preload한다. CycloneDDS의
  IPv4/IPv6 UDP `sendmsg`만 `MSG_DONTWAIT`로 보내 큐 포화 시 즉시 반환한다.
  여러 목적지에 짧은 timeout을 각각 적용해도 대기가 누적되므로 nonblocking을 사용한다.
  큐가 차 있으면 `ENOBUFS`로 반환해 무한 EAGAIN 재시도를 막는다. TCP와 다른
  라이브러리의 송신은 기존 동작을 유지한다. 시스템 DDS 라이브러리는 수정하지 않는다.

UDP 큐 포화 시 datagram을 버릴 수 있고 DDS가 `retcode -5`를 기록할 수 있다.
Reliable DDS는 이후 재전송할 수 있지만 실제 네트워크 단절을 해결하는 기능은 아니다.
새 timestamp로 오래된 센서 값을 반복 발행하거나 stale 검사를 느슨하게 만들지 않았다.

## Watchdog 설정

`/cmd_vel`은 속도 명령이며 마지막 명령을 받은 뒤 새로운 명령이 없을 때
그 속도가 자동으로 0이 된다는 보장이 없다. watchdog은 명령 공급 중단 시 0 속도를
전송해 계속 달리는 것을 방지한다. 기본값 `0.5초`를 유지한다. 정상 운전에서는
상위 제어기가 이보다 짧은 간격으로 명령을 보내야 한다.

`param/robot_launch_param.yaml`:

```yaml
cmd_vel_timeout_sec: 0.5
serial_transaction_timeout_ms: 100
guard_dds_udp_send: true
```

watchdog을 명시적으로 비활성화하려면 `cmd_vel_timeout_sec: 0.0`으로 설정한 뒤
bringup을 재시작한다. 단독 모터 launch에서는
`ros2 launch stella_md stella_md_launch.py cmd_vel_timeout_sec:=0.0`으로 설정할 수 있다.
모터 controller/USB 자체가 동작하지 않으면 host watchdog의 정지 명령도 전달될 수 없다.

DDS 보호 코드 없이 비교하려면 `guard_dds_udp_send: false`로 설정하고 재시작한다.
단독 driver launch에는 preload가 자동 적용되지 않는다.

## 빌드, 테스트, 측정

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stella_md stella_ahrs wheel_odometry stella_bringup --symlink-install
colcon test --packages-select stella_md stella_bringup --return-code-on-test-failure
source install/setup.bash
ros2 launch stella_bringup robot.launch.py
```

중복 bringup은 같은 serial 장치에 동시에 접근하므로 기존 launch가 종료된 것을 확인한
후 하나만 실행한다. 실측 검증은 모터를 움직이지 않는 정지 상태로 진행한다.

```bash
source ~/colcon_ws/install/setup.bash
LD_PRELOAD=~/colcon_ws/install/stella_bringup/lib/libstella_dds_udp_guard.so \
  ros2 run stella_bringup measure_stability.py --duration 145 --output /tmp/bringup_stability.json
```

측정 스크립트는 모터 명령을 발행하거나 serial port를 열지 않는다. 수신 간격/stamp age와
0.25초 이상 지연 시 driver thread 상태를 저장한다. NetworkManager scan 정보는
`python3-dbus`가 있을 때만 기록한다. `RMW_IMPLEMENTATION`과 `CYCLONEDDS_URI`는
실제 bringup과 동일하게 설정해야 한다.

자동 테스트는 silent serial endpoint의 timeout 및 다음 read 가능 여부, thread별 deadline
격리, 중첩 deadline, 불완전/긴 ACK 거부, UDP queue exhaustion 처리, 다른 caller/TCP
동작 보존, Python/rclpy 초기화 및 종료를 검증한다. 별도로 기존 encoder rollover 테스트도 실행한다.

실측 원본은 `~/docking_diagnostics/2026-10-06/`의 `stability_before.json`,
`stability_stationary.json`, `stability_bringup_stable.log`에서 확인할 수 있다.
주행 및 HOME_TO_DOCK 성공 여부는 이 정지 검증에 포함되지 않는다.

## 최종 정지 검증 결과

모터·IMU·카메라 초기화 후 동일하게 145초 측정했다.

| 토픽 | 변경 전 최대 수신 간격 | 변경 후 최대 수신 간격 |
| --- | ---: | ---: |
| `/wheel/encoders` | 1.300초 | 0.064초 |
| `/imu/data` | 1.926초 | 0.032초 |
| `/odom` | 2.216초 | 0.940초 |
| `/scan` | 1.829초 | 0.201초 |
| `/scan_2` | 1.999초 | 0.110초 |

모터 CPU 사용은 기존 약 79%에서 최종 5초 측정 평균 3.4%로 줄었다. encoder/IMU
실측 발행률은 각각 약 29.9Hz, 99.8Hz다. 새 encoder/IMU stamp를 가진 데이터만
발행한다. 0 속도 명령 1회를 보낸 검증에서 watchdog은 0.515초 후 정지 명령을
전송했고 검증 프로그램도 정상 종료했다. 이동 명령은 보내지 않았다.

**남은 문제:** `/odom` 수신에는 최대 0.940초 공백이 남았고 DDS queue exhaustion
로그도 계속 관찰됐다. startup 직후 encoder baseline reset이 1회 있었다. 센서 수집
및 모터 제어의 지연 확대 경로를 개선했지만 네트워크 문제 전체를 해결한 상태는 아니다.
PC SSH root 인증도 거부되어 PC controller 로그/설정은 직접 검사하지 못했다.
카메라 짐벌은 startup command 4회 실패를 보고했으며 카메라 연결/route 문제는 남아 있다.
실제 주행과 HOME_TO_DOCK 성공은 검증하지 않았다.

정지 검증 뒤 실행했던 bringup은 사용자 요청에 따라 정상 종료했다.
주행 테스트 준비를 위해 관련 패키지를 다시 빌드한 뒤 노드를 실행하지 않는 상태로 둔다.
재실행 시 기존 launch가 없는지 확인해 serial 장치에 중복 접근하지 않도록 한다.

기존 docking 작업 파일의 미커밋 변경은 보존했다.

## /cmd_vel 직접 주행 검증 (2026-10-06 추가)

사용자가 앞뒤 약 2m 공간을 준비한 뒤 실제 `/cmd_vel`로 짧은 왕복(0.08m/s),
긴 왕복(0.10m/s), 움직임 중 watchdog(0.06m/s)을 시험했고 모두 완료했다.
긴 왕복의 엔코더 기준 이동은 전진 0.817m, 후진 0.837m였다. 시험 전체의 최대
수신 간격은 encoder 0.080초, IMU 0.051초, odom 0.085초, odom TF 0.087초였다.
주행 중 명령 공급이 끊겨 발생한 예기치 않은 watchdog 정지는 없었다. 의도적으로
명령을 끊은 시험에서는 watchdog이 0.500초 후 작동했고 휠 정지는 마지막 명령
후 0.671초에 관측됐다. 검증 후 모터와 bringup을 모두 종료했다.

종전 정지 로그와 같은 PC/Nav2 부하를 유지한 A/B 비교는 아니다. 주행 시점에는
테스트 노드만 `/cmd_vel`을 발행했고 HOME_TO_DOCK 미션은 실행하지 않았다.
원본과 상세 결과는 `~/docking_diagnostics/2026-10-06/direct_motion_report.md` 및
`direct_motion_short.json`, `direct_motion_long.json`, `direct_motion_watchdog.json`을 참고한다.

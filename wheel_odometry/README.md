# wheel_odometry

STELLA N5의 timestamp가 있는 `/wheel/encoders`와 `/imu/data` orientation을
결합해 `/odom` 및 `odom -> base_footprint` TF를 발행한다.

yaw는 좌·우 휠 이동거리 차이로 먼저 예측하고, IMU의 절대 yaw로 누적
드리프트를 보정하는 complementary fusion을 사용한다. 기본 보정 시정수는
`0.5 s`, IMU에 의한 최대 보정 속도는 `1.0 rad/s`이다. `0.0 s`로 설정하면
기존처럼 매 샘플마다 IMU yaw를 그대로 적용한다.

IMU yaw는 timestamp 이력을 유지하고 엔코더 측정시각을 앞뒤로 감싸는 샘플이 있으면
최단 각도 방향으로 보간한다. 앞쪽 샘플만 있으면 timeout 이내의 최신 과거 샘플만
사용하며 미래 샘플 하나만으로 과거 엔코더를 보정하지 않는다.

`/wheel_odometry/yaw_diagnostics`의 `vector.x/y/z`에는 같은 엔코더 시각의
휠 적분 yaw, 시간 정렬된 IMU yaw, 융합 yaw가 각각 발행되어 rosbag으로 비교할 수 있다.

`/odom.twist.twist.angular.z`는 IMU heading 보정량을 제외한 휠 엔코더 차동 회전속도다.
따라서 IMU가 복구되거나 자기장 heading이 천천히 보정될 때 이를 실제 차체 회전으로
Nav2에 전달하지 않는다. `max_wheel_speed_m_s`보다 큰 좌우 휠 변화는 엔코더 reset,
rollover 또는 통신 오류로 보고 pose에 반영하지 않은 채 baseline을 재설정한다.

기본 기구 파라미터는 기존 `stella_md` 값을 유지한다.

- wheel radius: `0.0875 m`
- wheel separation: `0.36 m`
- wheel encoder source: motor `2000 count/rev` x gear ratio `51`
- IMU yaw: absolute orientation (`relative_imu_yaw: false`)

기본 bringup은 양쪽 엔코더를 `30 Hz`로 질의하고, AHRS 동기 전송 주기를
`5 ms`, ROS IMU 토픽의 최대 발행률을 `100 Hz`로 설정한다. AHRS 드라이버는
새 센서 타임스탬프가 생겼을 때만 발행하므로 같은 측정값을 반복 발행하지 않는다.
이 값들은 `stella_bringup/param/robot_launch_param.yaml`에서 변경할 수 있다.
같은 파일의 `motor_cpu_affinity: 2`, `imu_cpu_affinity: 3`은 Raspberry Pi 5의
서로 다른 코어에 두 시리얼 드라이버를 고정한다. affinity를 제한하지 않으려면
각 값을 `0-3`으로 설정한다.

`stella_md.enable_legacy_odom`과 `wheel_odometry.enabled`를 동시에 `true`로
설정하면 `/odom`과 TF가 중복되므로 반드시 한 쪽만 활성화한다.

```bash
ros2 param set /wheel_odometry enabled false
ros2 param set /stella_md_node enable_legacy_odom true
```

다시 새 odom으로 전환하려면 먼저 legacy odom을 끈다.

```bash
ros2 param set /stella_md_node enable_legacy_odom false
ros2 param set /wheel_odometry enabled true
```

노드 재활성화 시 encoder baseline은 현재 위치로 초기화하며 누적 pose는 유지한다.

#!/usr/bin/env python3
"""
통합 배터리 노드.

[ 도킹 감지 방식 ]
  poll_interval(기본 5s)마다 SK120 Modbus 통신을 시도한다.
  응답이 오면 → 도킹 감지 → /sk120/available: BatteryState(CHARGING) 발행
  연속 FAIL_THRESHOLD회 실패 → 도킹 해제 → 충전 자동 중지

[ 상태 흐름 ]
  IDLE ──(SK120 응답)──► AVAILABLE ──(/sk120/cmd_output: true)──► CHARGING
    ▲                        │                                        │
    └────(연속 N회 실패)──────┘◄──────────(연속 N회 실패 or cmd false)──┘

[ 전류 부호 관례 (ROS BatteryState) ]
  충전 중 : current > 0  (SK120 실측값)
  방전 중 : current < 0  (INA219 값 반전 – INA219는 방전=양수로 측정)

[ 램프업 중단 로직 ]
  실제 충전 전류가 확인된 전류만 _ramp_limit에 저장 → 재충전 시 그 전류까지만 올림
  실제 도킹 해제(SK120 전원 꺼짐)가 감지될 때만 _ramp_limit 초기화

[ 토픽 ]
  Subscribe
    /sk120/cmd_output  (Bool)   : 충전 시작(true) / 중지(false)

  Publish
    /battery_state     (BatteryState) : 배터리 상태 (항상)
    /sk120/available   (BatteryState) : SK120 연결 가능 여부
    /sk120/output_on   (Bool)         : 충전 출력 ON/OFF
    /sk120/current_set (Float32)      : 현재 설정 전류 [A]
    /sk120/current_out (Float32)      : SK120 실측 전류 [A]
    /sk120/voltage_out (Float32)      : SK120 실측 전압 [V]
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from smbus2 import SMBus
from std_msgs.msg import Bool, Float32

from .sk120_driver import SK120Driver


class StellaN5Monitor:
    def __init__(self, bus_num=1, addr=0x40):
        self.bus = SMBus(bus_num)
        self.addr = addr
        self.shunt_resistor = 0.01

        self.MAX_V = 25.2
        self.MIN_V = 17.5
        self.CAPACITY_AH = 13.0

    def read_voltage(self):
        raw = self.bus.read_word_data(self.addr, 0x02)
        raw = ((raw << 8) & 0xFF00) | (raw >> 8)
        return raw * 0.00125

    def read_current(self):
        raw = self.bus.read_word_data(self.addr, 0x01)
        if raw > 32767:
            raw -= 65536
        return (raw * 0.0000025) / self.shunt_resistor

    def get_soc(self, voltage):
        soc = (voltage - self.MIN_V) / (self.MAX_V - self.MIN_V) * 100
        return max(0.0, min(100.0, round(soc, 1)))

    def close(self):
        self.bus.close()


class BatteryNode(Node):

    FAIL_THRESHOLD = 3        # 연속 통신 실패 허용 횟수
    LOW_CURRENT_THRESHOLD = 0.05   # A – 이하면 "충전 전류 없음"으로 판단
    LOW_CURRENT_MAX_COUNT = 3      # 연속 N회 → 직전 정상 전류로 백오프
    GOOD_CURRENT_MIN_COUNT = 2     # 같은 설정 전류에서 연속 N회 정상 → 안전 전류로 확정

    def __init__(self):
        super().__init__('battery_node')

        # ── 파라미터 ────────────────────────────────────────────────────
        self.declare_parameter('port',            '/dev/SK120')
        self.declare_parameter('baudrate',        115200)
        self.declare_parameter('slave_id',        1)
        self.declare_parameter('voltage_set',     25.2)
        self.declare_parameter('start_current',   0.7)
        self.declare_parameter('target_current',  1.8)
        self.declare_parameter('ramp_step',       0.1)
        self.declare_parameter('ramp_interval',   5.0)
        self.declare_parameter('current_offset',  0.0)
        self.declare_parameter('status_interval', 2.0)
        self.declare_parameter('poll_interval',   5.0)

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._slave_id = self.get_parameter('slave_id').value
        self._v_set = self.get_parameter('voltage_set').value
        self._i_start = self.get_parameter('start_current').value
        self._i_target = self.get_parameter('target_current').value
        self._ramp_step = self.get_parameter('ramp_step').value
        self._ramp_interval = self.get_parameter('ramp_interval').value
        self._i_offset = self.get_parameter('current_offset').value
        status_interval = self.get_parameter('status_interval').value
        poll_interval = self.get_parameter('poll_interval').value

        self._i_start_raw = round(self._i_start + self._i_offset, 3)
        self._i_target_raw = round(self._i_target + self._i_offset, 3)

        # ── INA219 ───────────────────────────────────────────────────────
        self._monitor = StellaN5Monitor()

        # ── SK120 ────────────────────────────────────────────────────────
        self._sk120: SK120Driver | None = None

        # ── 내부 상태 ────────────────────────────────────────────────────
        self._sk120_ready = False
        self._charging = False
        self._ramp_active = False
        self._current_set = self._i_start_raw
        self._sk120_current = 0.0
        self._sk120_voltage = 0.0
        self._fail_count = 0

        # 램프업 중단 상한 (None = 제한 없음 / 값 있음 = 이번 세션 최대 전류)
        self._ramp_limit: float | None = None
        # 이번 충전에서 실제 적용할 목표 전류
        self._i_target_eff = self._i_target_raw
        # 실측 전류가 흐른 마지막 설정 전류
        self._last_good_current_set: float | None = None
        self._good_current_candidate: float | None = None
        self._good_current_count = 0

        # 충전 전류 감지 카운터
        self._low_current_count = 0

        # ── 퍼블리셔 ─────────────────────────────────────────────────────
        self._pub_battery = self.create_publisher(BatteryState, '/battery_state',     10)
        self._pub_available = self.create_publisher(BatteryState, '/sk120/available',   10)
        self._pub_on = self.create_publisher(Bool,         '/sk120/output_on',   10)
        self._pub_iset = self.create_publisher(Float32,      '/sk120/current_set', 10)
        self._pub_iout = self.create_publisher(Float32,      '/sk120/current_out', 10)
        self._pub_vout = self.create_publisher(Float32,      '/sk120/voltage_out', 10)

        # ── 서브스크라이버 ───────────────────────────────────────────────
        self.create_subscription(Bool, '/sk120/cmd_output', self._cb_cmd, 10)

        # ── 타이머 ───────────────────────────────────────────────────────
        self.create_timer(poll_interval,       self._cb_poll)
        self.create_timer(status_interval,     self._cb_status)
        self.create_timer(self._ramp_interval, self._cb_ramp)

        self.get_logger().info(
            f'배터리 노드 시작 | 폴링: {poll_interval}s | '
            f'충전: {self._i_start}A → {self._i_target}A'
        )

    # ──────────────────────────────────────────────────────────────────
    # SK120 폴링
    # ──────────────────────────────────────────────────────────────────
    def _cb_poll(self):
        if self._charging:
            return  # 충전 중에는 _cb_status가 담당

        ok = self._try_read_sk120()

        if ok:
            self._fail_count = 0
            if not self._sk120_ready:
                self._sk120_ready = True
                self._publish_sk120_available()
                self.get_logger().info(
                    '[충전 모듈 감지] 도킹 확인 -> /sk120/cmd_output: true 로 충전 시작 가능'
                )
        else:
            self._fail_count += 1
            if self._sk120_ready and self._fail_count >= self.FAIL_THRESHOLD:
                self.get_logger().warn(
                    f'[충전 모듈 응답 없음] {self.FAIL_THRESHOLD}회 연속 실패 → 도킹 해제'
                )
                self._on_undocked()

    def _try_read_sk120(self) -> bool:
        if self._sk120 is None:
            try:
                self._sk120 = SK120Driver(
                    self._port, baudrate=self._baudrate, slave_id=self._slave_id)
            except Exception:
                return False
        try:
            return self._sk120.get_status() is not None
        except Exception:
            return False

    def _on_undocked(self):
        """실제 도킹 해제 _ramp_limit 초기화 포함."""
        self._stop_charging(reset_ramp_limit=True)
        if self._sk120 is not None:
            try:
                self._sk120.close()
            except Exception:
                pass
            self._sk120 = None
        self._sk120_ready = False
        self._fail_count = 0
        self._low_current_count = 0
        self._publish_sk120_available()

    def _make_sk120_available_msg(self) -> BatteryState:
        """Nav2 docking이 읽을 수 있는 SK120 충전 가능 상태 메시지."""
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sk120'
        msg.voltage = 0.0
        msg.temperature = 0.0
        msg.current = 0.0
        msg.charge = 0.0
        msg.capacity = 0.0
        msg.design_capacity = 0.0
        msg.percentage = 1.0 if self._sk120_ready else 0.0
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self._sk120_ready
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = self._sk120_ready
        return msg

    def _publish_sk120_available(self):
        self._pub_available.publish(self._make_sk120_available_msg())

    # ──────────────────────────────────────────────────────────────────
    # 충전 시작/중지 명령
    # ──────────────────────────────────────────────────────────────────
    def _cb_cmd(self, msg: Bool):
        if msg.data:
            if not self._sk120_ready:
                self.get_logger().warn('[충전 명령 무시] SK120 미연결')
                return
            if not self._charging:
                self._start_charging()
        else:
            if self._charging:
                self._stop_charging(reset_ramp_limit=False)

    def _start_charging(self):
        # 이번 세션 목표 전류: 이전 중단 전류가 있으면 그 값까지만
        if self._ramp_limit is not None:
            self._i_target_eff = self._ramp_limit
            self.get_logger().info(
                f'[충전 시작] 이전 중단 전류 {round(self._ramp_limit - self._i_offset, 3)}A 적용'
            )
        else:
            self._i_target_eff = self._i_target_raw

        self._sk120.set_voltage(self._v_set)
        self._sk120.set_current(self._i_start_raw)
        self._sk120.set_output(True)
        self._current_set = self._i_start_raw
        self._charging = True
        self._ramp_active = True
        self._fail_count = 0
        self._low_current_count = 0
        self._last_good_current_set = None
        self._good_current_candidate = None
        self._good_current_count = 0
        self.get_logger().info(
            f'[충전 시작] 초기 {self._i_start}A -> 목표 '
            f'{round(self._i_target_eff - self._i_offset, 3)}A'
        )

    def _stop_charging(self, reset_ramp_limit: bool = False):
        if self._charging and self._sk120 is not None:
            try:
                self._sk120.set_output(False)
            except Exception:
                pass
        self._charging = False
        self._ramp_active = False
        self._sk120_current = 0.0
        self._sk120_voltage = 0.0
        self._low_current_count = 0
        self._last_good_current_set = None
        self._good_current_candidate = None
        self._good_current_count = 0
        if reset_ramp_limit:
            self._ramp_limit = None
        self.get_logger().info(
            f'[충전 중지] 다음 재충전 상한: '
            f'{round(self._ramp_limit - self._i_offset, 3) if self._ramp_limit else "없음(초기화)"}A'
        )

    # ──────────────────────────────────────────────────────────────────
    # 전류 램프업
    # ──────────────────────────────────────────────────────────────────
    def _cb_ramp(self):
        if not self._charging or not self._ramp_active or self._sk120 is None:
            return

        if self._current_set < self._i_target_eff:
            next_i = min(self._current_set + self._ramp_step, self._i_target_eff)
            if self._sk120.set_current(next_i):
                self._current_set = next_i
                real_i = round(self._current_set - self._i_offset, 3)
                self.get_logger().info(
                    f'[램프업] {real_i}A (SK120: {self._current_set}A)'
                )
                if self._current_set >= self._i_target_eff:
                    self._ramp_active = False
                    self.get_logger().info(
                        f'[램프업 완료] 목표 {round(self._i_target_eff - self._i_offset, 3)}A 도달'
                    )
            else:
                self._fail_count += 1
                if self._fail_count >= self.FAIL_THRESHOLD:
                    self.get_logger().error('[램프업] 통신 실패 반복 -> 충전 중지')
                    self._on_undocked()

    # ──────────────────────────────────────────────────────────────────
    # 상태 발행
    # ──────────────────────────────────────────────────────────────────
    def _cb_status(self):
        self._publish_sk120_available()

        # INA219 읽기 (항상)
        try:
            ina_voltage = self._monitor.read_voltage()
            ina_current = self._monitor.read_current()
            soc = self._monitor.get_soc(ina_voltage)
        except Exception as e:
            self.get_logger().error(f'INA219 읽기 오류: {e}')
            return

        # SK120 읽기 (충전 중일 때)
        if self._charging and self._sk120 is not None:
            sk_status = self._sk120.get_status()
            if sk_status is not None:
                self._sk120_current = sk_status.current_out
                self._sk120_voltage = sk_status.voltage_out
                self._fail_count = 0
                self._pub_iout.publish(Float32(data=sk_status.current_out))
                self._pub_vout.publish(Float32(data=sk_status.voltage_out))
                self._pub_iset.publish(Float32(data=sk_status.current_set))

                # ── 실제 충전 전류 감지 체크 ────────────────────────────
                if sk_status.current_out < self.LOW_CURRENT_THRESHOLD:
                    self._low_current_count += 1
                    if self._low_current_count >= self.LOW_CURRENT_MAX_COUNT:
                        self._handle_low_charge_current(sk_status.current_out)
                else:
                    self._low_current_count = 0
                    self._remember_good_current(sk_status.current_set)

            else:
                self._fail_count += 1
                self.get_logger().warn(
                    f'[SK120 읽기 실패] {self._fail_count}/{self.FAIL_THRESHOLD}회'
                )
                if self._fail_count >= self.FAIL_THRESHOLD:
                    self.get_logger().error('[SK120 연결 끊김] 충전 자동 중지')
                    self._on_undocked()
                    return

            self._pub_on.publish(Bool(data=self._charging))
        else:
            self._pub_on.publish(Bool(data=False))

        # ── /battery_state 구성 ──────────────────────────────────────
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'battery'
        msg.capacity = float(self._monitor.CAPACITY_AH)
        msg.design_capacity = float(self._monitor.CAPACITY_AH)
        msg.percentage = float(soc / 100.0)
        msg.present = True
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_DEAD
            if ina_voltage < self._monitor.MIN_V
            else BatteryState.POWER_SUPPLY_HEALTH_GOOD
        )

        if self._charging:
            # 전압: INA219와 SK120 평균 (더 안정적인 배터리 전압 추정)
            msg.voltage = float((ina_voltage + self._sk120_voltage) / 2.0)
            # 전류: SK120 실측값 (양수 = 충전)
            msg.current = float(self._sk120_current)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            # 전압: INA219 직접 측정값
            msg.voltage = float(ina_voltage)
            # 전류: INA219 반전 (배터리에 직결하지 않아 방전전류가 양수로 측정됨)
            msg.current = -float(ina_current)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        self._pub_battery.publish(msg)
        self.get_logger().debug(
            f'[배터리] {msg.voltage:.2f}V  {msg.current:+.3f}A  SoC={soc:.1f}%  '
            f'{"[충전중]" if self._charging else "[방전중]"}'
        )

    def _remember_good_current(self, current_set: float):
        """실제 전류가 흐른 설정값만 다음 램프업 상한으로 설정."""
        good_current = min(round(current_set, 3), round(self._current_set, 3))
        if good_current <= 0.0:
            return

        if (
            self._good_current_candidate is None
            or abs(good_current - self._good_current_candidate) > 0.001
        ):
            self._good_current_candidate = good_current
            self._good_current_count = 1
        else:
            self._good_current_count += 1

        if self._good_current_count >= self.GOOD_CURRENT_MIN_COUNT:
            self._last_good_current_set = good_current
            self._ramp_limit = good_current

    def _handle_low_charge_current(self, measured_current: float):
        """무선 커플링이 깨진 경우 직전 정상 전류까지만 충전하도록 백오프."""
        fallback = self._last_good_current_set
        if fallback is None:
            fallback = max(self._i_start_raw, self._current_set - self._ramp_step)
        fallback = round(min(fallback, self._current_set), 3)

        if fallback >= self._current_set - 0.001:
            self.get_logger().warn(
                f'[충전 전류 없음] {measured_current:.3f}A – 직전 정상 전류에서도 '
                '전류가 없어 충전 중지'
            )
            self._stop_charging(reset_ramp_limit=False)
            return

        self._ramp_limit = fallback
        self._i_target_eff = fallback
        self._ramp_active = True
        self._low_current_count = 0
        self._good_current_candidate = None
        self._good_current_count = 0

        real_fallback = round(fallback - self._i_offset, 3)
        real_current = round(self._current_set - self._i_offset, 3)
        real_restart = round(self._i_start_raw - self._i_offset, 3)
        self.get_logger().warn(
            f'[충전 전류 없음] {measured_current:.3f}A – {real_current}A에서 '
            f'커플링 불량 감지, {real_restart}A부터 재시작해 '
            f'{real_fallback}A까지만 램프업'
        )

        try:
            self._sk120.set_output(False)
            self._sk120.set_voltage(self._v_set)
            if self._sk120.set_current(self._i_start_raw):
                self._current_set = self._i_start_raw
            self._sk120.set_output(True)
        except Exception as e:
            self.get_logger().error(f'[충전 복구 실패] SK120 제어 오류: {e}')
            self._stop_charging(reset_ramp_limit=False)

    # ──────────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_charging(reset_ramp_limit=True)
        if self._sk120 is not None:
            try:
                self._sk120.close()
            except Exception:
                pass
        try:
            self._monitor.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

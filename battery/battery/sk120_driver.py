#!/usr/bin/env python3
"""
XY-SK120 Modbus RTU driver.

Register map (from XY-SK120 documentation):
  0x0000  REG_V_SET   Voltage setpoint     [unit: 0.01 V]
  0x0001  REG_I_SET   Current setpoint     [unit: 0.001 A]
  0x0002  REG_VOUT    Output voltage       [unit: 0.01 V]  (read-only)
  0x0003  REG_IOUT    Output current       [unit: 0.001 A] (read-only)
  0x0004  REG_POWER   Output power         [unit: 0.01 W]  (read-only)
  0x0005  REG_UIN     Input voltage        [unit: 0.01 V]  (read-only)
  0x0012  REG_ONOFF   Output on/off        0=off, 1=on
"""

from dataclasses import dataclass

from .modbus_rtu import ModbusRTU

# Register addresses
REG_V_SET = 0x0000
REG_I_SET = 0x0001
REG_VOUT = 0x0002
REG_IOUT = 0x0003
REG_POWER = 0x0004
REG_UIN = 0x0005
REG_ONOFF = 0x0012


@dataclass
class SK120Status:
    voltage_set: float   # V
    current_set: float   # A
    voltage_out: float   # V
    current_out: float   # A
    power_out:   float   # W
    voltage_in:  float   # V
    output_on:   bool


class SK120Driver:
    """
    High-level driver for the XY-SK120 charging module.

    Parameters
    ----------
    port      : serial port, e.g. '/dev/ttyUSB0'
    baudrate  : baud rate (default 9600 for SK120 Modbus)
    slave_id  : Modbus slave address (default 1)

    """

    def __init__(self, port: str, baudrate: int = 115200, slave_id: int = 1):
        self._bus = ModbusRTU(port, baudrate=baudrate, slave_id=slave_id)

    def close(self):
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ------------------------------------------------------------------
    # Setters
    # ------------------------------------------------------------------
    def set_voltage(self, volts: float) -> bool:
        """Set target voltage (V)."""
        raw = round(volts * 100)
        return self._bus.write_register(REG_V_SET, raw)

    def set_current(self, amps: float) -> bool:
        """Set target current (A)."""
        raw = round(amps * 1000)
        return self._bus.write_register(REG_I_SET, raw)

    def set_output(self, on: bool) -> bool:
        """Enable or disable output."""
        return self._bus.write_register(REG_ONOFF, 1 if on else 0)

    # ------------------------------------------------------------------
    # Getters
    # ------------------------------------------------------------------
    def get_status(self) -> SK120Status | None:
        """Read all key registers in one shot (6 consecutive regs + ONOFF)."""
        regs = self._bus.read_registers(REG_V_SET, 6)   # 0x0000-0x0005
        if regs is None:
            return None
        onoff_raw = self._bus.read_register(REG_ONOFF)
        if onoff_raw is None:
            return None
        return SK120Status(
            voltage_set=regs[0] / 100.0,
            current_set=regs[1] / 1000.0,
            voltage_out=regs[2] / 100.0,
            current_out=regs[3] / 1000.0,
            power_out=regs[4] / 100.0,
            voltage_in=regs[5] / 100.0,
            output_on=bool(onoff_raw),
        )

    def get_current_set(self) -> float | None:
        """Return current setpoint in amperes."""
        raw = self._bus.read_register(REG_I_SET)
        if raw is None:
            return None
        return raw / 1000.0

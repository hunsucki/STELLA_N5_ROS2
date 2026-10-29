"""Read INA226/INA219 registers without changing the sensor configuration."""

from smbus2 import SMBus


class StellaN5Monitor:
    """Measure bus voltage and signed shunt current (positive in load direction)."""

    MAX_V = 25.2
    MIN_V = 17.5
    CAPACITY_AH = 13.0

    def __init__(self, bus_num=1, addr=0x40, sensor_type='auto',
                 shunt_resistor=0.01):
        self.bus = SMBus(bus_num)
        self.addr = addr
        self.shunt_resistor = shunt_resistor
        try:
            if sensor_type == 'auto':
                if (self._read_word(0xfe) != 0x5449
                        or self._read_word(0xff) & 0xfff0 != 0x2260):
                    raise ValueError('Unknown sensor; configure sensor_type explicitly')
                sensor_type = 'ina226'
            if sensor_type not in ('ina226', 'ina219') or shunt_resistor <= 0:
                raise ValueError('Invalid sensor type or shunt resistance')
            self.sensor_type = sensor_type
        except Exception:
            self.close()
            raise

    def _read_word(self, register):
        raw = self.bus.read_word_data(self.addr, register)
        return ((raw & 0xff) << 8) | (raw >> 8)

    def read_voltage(self):
        raw = self._read_word(0x02)
        if self.sensor_type == 'ina219':
            return (raw >> 3) * 0.004
        return raw * 0.00125

    def read_current(self):
        raw = self._read_word(0x01)
        if raw & 0x8000:
            raw -= 65536
        lsb = 0.0000025 if self.sensor_type == 'ina226' else 0.00001
        return raw * lsb / self.shunt_resistor

    def get_soc(self, voltage):
        """Return a rough voltage estimate, not measured remaining capacity."""
        soc = (voltage - self.MIN_V) / (self.MAX_V - self.MIN_V) * 100
        return max(0.0, min(100.0, round(soc, 1)))

    def close(self):
        self.bus.close()

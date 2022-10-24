"""INAxxx Texas Instruments sensors module.

Внимание! для долговременной непрерывной работы токового шунта, не допускайте выделения на нем более половины(!) от его
максимальной рассеиваемой мощности!
Мощность, выделяемая на любом сопротивлении (постоянный ток), расчитывается по формуле: P=I^2*R
где: I - ток в Амперах; R - сопротивление в Омах.

Attention! for long-term continuous operation of the current shunt, do not allow more than half(!) of its maximum
dissipated power to be allocated on it!!
The power dissipated on any resistance (direct current) is calculated by the formula: P=I^2*R
where: I - current in Amperes; R - resistance in ohms"""
from sensor_pack import bus_service
from sensor_pack.base_sensor import BaseSensor, Device, Iterator, check_value
# from sensor_pack import bitfield
# import struct
# import array


class InaBase(Device):
    """Base class for INA current/voltage monitor"""
    def __init__(self, adapter: bus_service.BusAdapter, address):
        """shunt_resistance - сопротивление шунта, [Ом]"""
        super().__init__(adapter, address, True)    # All data bytes are transmitted most significant byte first.

    def _read_register(self, reg_addr, bytes_count=2) -> bytes:
        """считывает из регистра датчика значение.
        bytes_count - размер значения в байтах"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

        # BaseSensor
    def _write_register(self, reg_addr, value: [int, bytes, bytearray], bytes_count=2) -> int:
        """записывает данные value в датчик, по адресу reg_addr.
        bytes_count - кол-во записываемых данных"""
        byte_order = self._get_byteorder_as_str()[0]
        return self.adapter.write_register(self.address, reg_addr, value, bytes_count, byte_order)

    def _set_raw_cfg(self, value: int) -> int:
        return self._write_register(0x00, value, 2)

    def _get_raw_cfg(self) -> int:
        b = self._read_register(0x00, 2)
        return self.unpack("H", b)[0]


class INA219Simple(InaBase):
    """Класс для работы с датчиком TI INA219 без какой либо настройки!
    Диапазон измерения входного напряжения: 0-26 Вольт.
    Диапазон измерения напряжения на токоизмерительном шунте: ±320 милливолт.
    Никаких настроек нет!
    ---------------------
    A class for working with a TI INA219 sensor without any configuration!
    Input voltage measurement range: 0-26 Volts.
    Voltage measurement range on the current measuring shunt: ±320 millivolts.
    There are no settings!"""
    def __init__(self, adapter: bus_service.BusAdapter, address=0x40):
        super().__init__(adapter, address)
        self._set_raw_cfg(0b0011_1001_1001_1111)    # 0x399F    default setting, simple reading two voltages

    def get_shunt_voltage(self) -> float:
        """Возвращает напряжение на шунте в Вольтах. Чтобы вычислить ток через шунт, нужно это напряжение поделить
        на сопротивление шунта в Омах!!!
        Returns the shunt voltage in Volts. To calculate the current through the shunt, you need to divide this voltage
        by the resistance of the shunt in ohms !!!"""
        # DC ACCURACY:  ADC basic resolution: 12 bit;    Shunt voltage, 1 LSB step size: 10 μV
        reg_raw = self._read_register(0x01, 2)
        return 1E-5 * self.unpack("h", reg_raw)[0]

    def get_voltage(self) -> tuple:
        """Возвращает кортеж из входного измеряемого напряжения, флага готовности данных, флага математического
        переполнения (OVF).
        Флаг математического переполнения (OVF) устанавливается, когда расчеты мощности или тока выходят за допустимые
        пределы. Это указывает на то, что данные о токе и мощности могут быть бессмысленными!
        ------------------------------------------------------------------------------
        Returns a tuple of input measured voltage, data ready flag, math overflow flag (OVF).
        The Math Overflow Flag (OVF) is set when power or current calculations are out of range.
        This indicates that current and power data may be meaningless!"""
        # DC ACCURACY:  ADC basic resolution: 12 bit;    Bus voltage, 1 LSB step size: 4 mV
        reg_raw = self.unpack("h", self._read_register(0x02, 2))[0]
        #           voltage             data ready flag         math overflow flag
        return 0.004 * (reg_raw >> 3), bool(reg_raw & 0x02), bool(reg_raw & 0x01)


class INA219(INA219Simple, Iterator):
    """Class for work with TI INA219 sensor"""
    def __init__(self, adapter: bus_service.BusAdapter, address=0x40):
        """shunt_resistance - сопротивление шунта, [Ом]"""
        super().__init__(adapter, address)
        #
        self.bus_voltage_range = True  # False - 16 V; True - 32 V
        self.continuous_mode = True    # непрерывный или однократный режимы работы
        self.bus_voltage = True        # измерение напряжения на шине, режимы работы
        self.shunt_voltage = True      # измерение тока, путем измерения напряжения на шунте, режимы работы
        # value     range, mV
        # 0         ±40 mV
        # 1         ±80 mV
        # 2         ±160 mV
        # 3         ±320 mV     default
        self._current_shunt_voltage_range = 3   # диапазон напряжения, измеряемого на токовом шунте      0..3
        # делаю "недоступным" для пользователя. это лишние настройки!
        # разрешение/усреднение АЦП напряж. на токовом шунте. 12 bit. Conversion time: 532 μs
        self._current_adc_resolution = 3
        # разрешение/усреднение АЦП входного изменяемого напряж. 12 bit. Conversion time: 532 μs
        self._voltage_adc_resolution = 3

    @property
    def current_shunt_voltage_range(self):
        return self._current_shunt_voltage_range

    @current_shunt_voltage_range.setter
    def current_shunt_voltage_range(self, value):
        self._current_shunt_voltage_range = check_value(value, range(4),
                                                        f"Invalid current shunt voltage range: {value}")

    def set_config(self) -> int:
        """Настраивает датчик в соответствии с настройками"""
        raw_cfg = int(self.bus_voltage_range) << 13     # Bus Voltage Range bit
        raw_cfg |= self._current_shunt_voltage_range << 11   # PGA (Shunt Voltage Only)
        raw_cfg |= self._voltage_adc_resolution << 7         # BADC Bus ADC Resolution/Averaging
        raw_cfg |= self._current_adc_resolution << 3         # SADC Shunt ADC Resolution/Averaging
        # mode
        raw_cfg |= int(self.continuous_mode) << 2   # continuous
        raw_cfg |= int(self.bus_voltage) << 1   # Bus voltage
        raw_cfg |= int(self.shunt_voltage)      # Shunt voltage
        #
        return self._set_raw_cfg(raw_cfg)

    def get_config(self) -> int:
        """Считывает настройками датчика по шине"""
        raw_cfg = self._get_raw_cfg()
        # mode
        self.shunt_voltage = bool(raw_cfg & 0x0001)     # измерять напряжение на токовом шунте
        self.bus_voltage = bool(raw_cfg & 0x0002)       # измерять напряжение на шине
        self.continuous_mode = bool(raw_cfg & 0x0004)   # непрерывный режим измерений
        #
        self._current_adc_resolution = (raw_cfg & 0b0000_0000_0111_1000) >> 3
        self._voltage_adc_resolution = (raw_cfg & 0b0000_0111_1000_0000) >> 7
        self._current_shunt_voltage_range = (raw_cfg & 0b0000_1000_0000_0000) >> 11
        # диапазон входного измеряемого напряжения: False-16 V; True - 32 V
        self.bus_voltage_range = bool(raw_cfg & 0b0010_0000_0000_0000)
        #
        return raw_cfg

    # BaseSensor

    def get_power(self):
        reg_raw = self._read_register(0x03, 2)[0]
        ...

    def get_current(self):
        reg_raw = self._read_register(0x04, 2)[0]
        ...

    def get_id(self):
        return None

    def soft_reset(self):
        self._set_raw_cfg(0b11100110011111)

    def __iter__(self):
        return self

    def __next__(self):
        raise NotImplementedError

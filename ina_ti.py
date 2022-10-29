"""INAxxx Texas Instruments sensors module.

Внимание! для долговременной непрерывной работы токового шунта, не допускайте выделения на нем более половины(!) от его
максимальной рассеиваемой мощности!
Мощность, выделяемая на любом сопротивлении (постоянный ток), расчитывается по формуле: P=I^2*R
где: I - ток в Амперах; R - сопротивление в Омах.

Attention! for long-term continuous operation of the current shunt, do not allow more than half(!) of its maximum
dissipated power to be allocated on it!!
The power dissipated on any resistance (direct current) is calculated by the formula: P=I^2*R
where: I - current in Amperes; R - resistance in ohms"""
import math
from sensor_pack import bus_service
from sensor_pack.base_sensor import Device, Iterator, check_value


def get_exponent(value: float) -> int:
    """Возвращает десятичную степень числа"""
    return int(math.floor(math.log10(abs(value)))) if 0 != value else 0


class InaBase(Device):
    """Base class for INA current/voltage monitor"""

    def __init__(self, adapter: bus_service.BusAdapter, address):
        """shunt_resistance - сопротивление шунта, [Ом]"""
        super().__init__(adapter, address, True)  # All data bytes are transmitted most significant byte first.

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
        self._set_raw_cfg(0b0011_1001_1001_1111)  # 0x399F    default setting, simple reading two voltages

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
        # print(f"reg_raw: {hex(reg_raw)}")
        #           voltage             data ready flag         math overflow flag
        return 0.004 * (reg_raw >> 3), bool(reg_raw & 0x02), bool(reg_raw & 0x01)


class INA219(INA219Simple, Iterator):
    """Class for work with TI INA219 sensor"""

    # предел напряжения на шунте, Вольт
    _shunt_voltage_limit = 0.32
    # Предел измеряемого напряжения! И неважно, что чип измеряет до 32 Вольт!
    # В документации 26. Минус 1 вольт для запаса (Senses Bus Voltages from 0 to 26 V)
    vbus_max = 25

    def __init__(self, adapter: bus_service.BusAdapter, address=0x40, shunt_resistance: float = 0.1):
        """shunt_resistance - сопротивление шунта, [Ом].
        shunt_resistance - shunt resistance, [Ohm]"""
        super().__init__(adapter, address)
        #
        self.bus_voltage_range = True  # False - 16 V; True - 32 V
        self.continuous_mode = True  # непрерывный или однократный режимы работы
        self.bus_voltage = True  # измерение напряжения на шине, режимы работы
        self.shunt_voltage = True  # измерение тока, путем измерения напряжения на шунте, режимы работы
        # value     range, mV
        # 0         ±40 mV
        # 1         ±80 mV
        # 2         ±160 mV
        # 3         ±320 mV     default
        self._current_shunt_voltage_range = 3  # диапазон напряжения, измеряемого на токовом шунте      0..3
        # делаю "недоступным" для пользователя. это лишние настройки!
        # разрешение/усреднение АЦП напряж. на токовом шунте. 12 bit. Conversion time: 532 μs
        self._current_adc_resolution = 3
        # разрешение/усреднение АЦП входного изменяемого напряж. 12 bit. Conversion time: 532 μs
        self._voltage_adc_resolution = 3
        # dont set shunt_resistance to zero.zero and below zero!
        self._shunt_res = shunt_resistance
        self._lsb_current_reg = 0  # for calibrate method
        self._lsb_power_reg = 0  # for calibrate method
        self._maximum_current = 0  # from _calc method
        self.max_shunt_voltage_before_ovf = 0

    @staticmethod
    def _shunt_voltage_range_to_volt(shunt_voltage_range: int):
        """Преобразует индекс диапазона напряжения токового шунта в напряжение, Вольт.
        Converts the current shunt voltage range index to voltage, Volts."""
        index = check_value(shunt_voltage_range, range(4),
                            f"Invalid current shunt voltage range: {shunt_voltage_range}")
        return 0.040 * (2 ** index)

    def _get_calibr_reg(self) -> int:
        """Get calibration register value"""
        reg_raw = self._read_register(0x05, 2)
        return self.unpack("h", reg_raw)[0]     # >> 1# drop FS0 bit. Note that bit FS0 is not used in the calculation!

    def _set_calibr_reg(self, value: int):
        """Set calibration register value"""
        self._write_register(0x05, value, 2)
        # self._write_register(0x05, value << 1, 2)

    def _calc(self, max_expected_current: float, shunt_resistance: float) -> tuple:
        """Вычисляет и возвращает содержимое калибровочного регистра, значения наименее значимых битов
        регистров тока и мощности, наибольший измеряемый ток через шунт в А, наибольшую измеряемую мощность
        max_expected_current - наибольший ожидаемый ток через токовый шунт, Ампер
        shunt_resistance - сопротивление шунта, Ом
        -------------------------------------------------------------------------
        Calculates and returns the contents of the calibration register, the values of the least significant bits of
        the current and power registers, the largest measurable current through the shunt in Amps,
        the largest measurable power in watts.
        max_expected_current - maximum expected current through the current shunt, Ampere
        shunt_resistance - shunt resistance, Ohm"""

        # расчет максимального измеряемого тока, Ампер
        maximum_current = INA219._shunt_voltage_range_to_volt(self._current_shunt_voltage_range) / shunt_resistance
        # Вычисляю возможный диапазон весов младших разрядов (мин. = 15 бит, макс. = 12 бит)
        lsb_min = max_expected_current / (2 ** 15 - 1)  # uA/bit
        lsb_max = max_expected_current / 2 ** 12  # uA/bit
        # выбор lsb из диапазона lsb_min..lsb_max (желательно круглое число, близкое к lsb_min)
        k = 10**(1 + get_exponent(lsb_min))     # округление (в большую сторону) до 10 с учетом порядка/степени числа
        lsb_current = min(k, lsb_max)
        # print(f"DBG; lsb_min: {lsb_min}; lsb_max: {lsb_max}; lsb_current: {lsb_current}")
        # вычисляю значение для регистра калибровки
        cal_val = math.trunc(0.04096 / (lsb_current * shunt_resistance))
        # Calculate the power LSB
        lsb_power = 20 * lsb_current  # mW
        # Вычисляю максимальные значения тока и напряжения шунта до переполнения!
        max_current = lsb_current * (2 ** 15 - 1)  # before overflow

        if max_current >= maximum_current:
            max_current_before_ovf = maximum_current
        else:
            max_current_before_ovf = max_current

        max_shunt_voltage = max_current_before_ovf * shunt_resistance

        if max_shunt_voltage >= INA219._shunt_voltage_limit:
            max_shunt_voltage_before_ovf = INA219._shunt_voltage_limit
        else:
            max_shunt_voltage_before_ovf = max_shunt_voltage

        self.max_shunt_voltage_before_ovf = max_shunt_voltage_before_ovf
        # Вычисляю максимальную мощность, Вт
        max_power = max_current_before_ovf * INA219.vbus_max

        return cal_val, lsb_current, lsb_power, maximum_current, max_power

    def calibrate(self, max_expected_current: float) -> int:
        """Производит расчеты и запись значения в регистр калибровки. Вызови _calc до вызова этого метода!!!
        Performs calculations and writes the value to the calibration register.
        Call _calc before calling this method!!!"""
        calibr_reg, self._lsb_current_reg, self._lsb_power_reg, max_curr, max_pwr = self._calc(max_expected_current, self._shunt_res)
        self._set_calibr_reg(calibr_reg)
        if 0 >= self.max_shunt_voltage_before_ovf:
            raise ValueError("Invalid max_shunt_voltage_before_ovf value. Call _calc method before calibrate!")
        # автоустановка напряжения на шунте! self.max_shunt_voltage_before_ovf
        for sh_v_rng in range(4):
            if self._shunt_voltage_range_to_volt(sh_v_rng) >= self.max_shunt_voltage_before_ovf:
                self.current_shunt_voltage_range = sh_v_rng
                break
        else:
            raise ValueError("Shunt voltage range autoset failed!")
        return calibr_reg

    @property
    def shunt_resistance(self):
        """Возвращает сопротивление токового шунта в Омах.
        Returns the resistance of the current shunt in ohms."""
        return self._shunt_res

    @property
    def current_lsb(self):
        """Возвращает вес наименьшего разряда в регистре тока, Ампер.
        Returns the weight of the LSB in the current register, Ampere"""
        return self._lsb_current_reg

    @property
    def power_lsb(self):
        """Возвращает вес наименьшего разряда в регистре мощности, Ватт.
        Returns the weight of LSB in the power register, in watts."""
        return self._lsb_power_reg

    # value     range, mV
    # 0         ±40 mV
    # 1         ±80 mV
    # 2         ±160 mV
    # 3         ±320 mV     default
    @property
    def current_shunt_voltage_range(self):
        return self._current_shunt_voltage_range

    @current_shunt_voltage_range.setter
    def current_shunt_voltage_range(self, value):
        self._current_shunt_voltage_range = check_value(value, range(4),
                                                        f"Invalid current shunt voltage range: {value}")

    def set_config(self) -> int:
        """Настраивает датчик в соответствии с настройками"""
        raw_cfg = int(self.bus_voltage_range) << 13  # Bus Voltage Range bit
        raw_cfg |= self._current_shunt_voltage_range << 11  # PGA (Shunt Voltage Only)
        raw_cfg |= self._voltage_adc_resolution << 7  # BADC Bus ADC Resolution/Averaging
        raw_cfg |= self._current_adc_resolution << 3  # SADC Shunt ADC Resolution/Averaging
        # mode
        raw_cfg |= int(self.continuous_mode) << 2  # continuous
        raw_cfg |= int(self.bus_voltage) << 1  # Bus voltage
        raw_cfg |= int(self.shunt_voltage)  # Shunt voltage
        #
        return self._set_raw_cfg(raw_cfg)

    def get_config(self) -> int:
        """Считывает настройками датчика по шине"""
        raw_cfg = self._get_raw_cfg()
        # mode
        self.shunt_voltage = bool(raw_cfg & 0x0001)  # измерять напряжение на токовом шунте
        self.bus_voltage = bool(raw_cfg & 0x0002)  # измерять напряжение на шине
        self.continuous_mode = bool(raw_cfg & 0x0004)  # непрерывный режим измерений
        #
        self._current_adc_resolution = (raw_cfg & 0b0000_0000_0111_1000) >> 3
        self._voltage_adc_resolution = (raw_cfg & 0b0000_0111_1000_0000) >> 7
        self._current_shunt_voltage_range = (raw_cfg & 0b0000_1000_0000_0000) >> 11
        # диапазон входного измеряемого напряжения: False-16 V; True - 32 V
        self.bus_voltage_range = bool(raw_cfg & 0b0010_0000_0000_0000)
        #
        return raw_cfg

    # BaseSensor

    def get_power(self) -> float:
        reg_raw = self._read_register(0x03, 2)
        return self._lsb_power_reg * self.unpack("h", reg_raw)[0]

    def get_current(self) -> float:
        reg_raw = self._read_register(0x04, 2)
        return self._lsb_current_reg * self.unpack("h", reg_raw)[0]

    def get_id(self):
        return None

    def soft_reset(self):
        self._set_raw_cfg(0b11100110011111)

    def __iter__(self):
        return self

    def __next__(self) -> tuple:
        """Возвращает измеренные значения. кортеж, число.
        Return measured values. tuple, digit"""
        return self.get_voltage(), self.get_shunt_voltage()

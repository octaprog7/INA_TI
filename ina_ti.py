"""INAxxx Texas Instruments sensors module.

Внимание! для долговременной непрерывной работы токового шунта, не допускайте выделения на нем более половины(!) от его
максимальной рассеиваемой мощности!
Мощность, выделяемая на любом сопротивлении (постоянный ток), рассчитывается по формуле: P=I^2*R
где: I - ток в Амперах; R - сопротивление в Омах.

Attention! for long-term continuous operation of the current shunt, do not allow more than half(!) of its maximum
dissipated power to be allocated on it!!
The power dissipated on any resistance (direct current) is calculated by the formula: P=I^2*R
where: I - current in Amperes; R - resistance in ohms"""
import math
from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import Device, Iterator, check_value


def get_exponent(value: float) -> int:
    """Возвращает десятичную степень числа.
    Returns the decimal power of a number"""
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
        """Set raw configuration in register"""
        return self._write_register(0x00, value, 2)

    def _get_raw_cfg(self) -> int:
        """Get raw configuration from register"""
        b = self._read_register(0x00, 2)
        return self.unpack("H", b)[0]


class INA219Simple(InaBase):
    """Класс для работы с датчиком TI INA219 без какой либо настройки!
    Диапазон измерения входного напряжения: 0-26 Вольт.
    Диапазон измерения напряжения на токоизмерительном шунте: ±320 милливольт.
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
    # shunt voltage limit, Volt
    _shunt_voltage_limit = 0.32
    # Предел измеряемого напряжения! И неважно, что чип измеряет до 32 Вольт!
    # В документации 26. Минус 1 вольт для запаса (Senses Bus Voltages from 0 to 26 V)
    # Measured voltage limit! And it doesn't matter that the chip measures up to 32 volts!
    # In the documentation 26. Minus 1 volt for a margin (Senses Bus Voltages from 0 to 26 V)
    _vbus_max = 25

    def __init__(self, adapter: bus_service.BusAdapter, address=0x40, shunt_resistance: float = 0.1):
        """shunt_resistance - сопротивление шунта, [Ом].
        shunt_resistance - shunt resistance, [Ohm]"""
        super().__init__(adapter, address)
        # False - 16 V; True - 32 V
        self.bus_voltage_range = True
        # непрерывный или однократный режимы работы
        # continuous or single operation modes
        self.continuous_mode = True
        # если Истина, то измерение напряжения на шине производится! режимы работы
        # if True, then bus voltage measurement is performed! operating modes
        self.bus_voltage = True
        # если Истина, измерение тока, путем измерения напряжения на шунте, производится! режимы работы
        # if True, a current measurement, by measuring the voltage across the shunt, is done! operating modes
        self.shunt_voltage = True
        # value     range, mV
        # 0         ±40 mV
        # 1         ±80 mV
        # 2         ±160 mV
        # 3         ±320 mV     default
        self._current_shunt_voltage_range = 3  # диапазон напряжения, измеряемого на токовом шунте      0..3
        # делаю "недоступным" для пользователя. это лишние настройки! Устанавливаю наибольшее разрешение!
        # # make it "unavailable" to the user. those are redundant settings. I set the highest resolution!
        # разрешение/усреднение АЦП напряж. на токовом шунте. 12 bit. Conversion time: 532 μs
        self._current_adc_resolution = 3
        # разрешение/усреднение АЦП входного изменяемого напряж. 12 bit. Conversion time: 532 μs
        self._voltage_adc_resolution = 3
        # dont set shunt_resistance to zero.zero and below zero!
        self._shunt_res = shunt_resistance
        self._lsb_current_reg = 0  # for calibrate method
        self._lsb_power_reg = 0  # for calibrate method
        self._maximum_current = 0  # from _calc method
        self._maximum_power = 0  # from _calc method
        self._max_shunt_voltage_before_ovf = 0   # from _calc method
        # автоматический выбор диапазона измерения напряжения шины и тока через токовый шунт
        # automatic selection of bus voltage and current measurement range via current shunt
        self._auto_range = False
        # max_expected_current - наибольший ожидаемый ток через токовый шунт.
        # Если max_expected_current==None, то происходит автоматическое вычисление этого тока. Он устанавливается в
        # максимальное значение
        # max_expected_current - the maximum expected current through the current shunt.
        # If max_expected_current==None, then this current is automatically calculated. It is set to the maximum value.
        self._calc(shunt_resistance=shunt_resistance, max_expected_current=None)

    def get_bus_voltage_range(self) -> tuple:
        """Возвращает измеряемый диапазон напряжений на шине в Вольтах в виде кортежа (верхний_предел, нижний_предел).
        Returns the measured bus voltage range in Volts as a tuple (high_limit, low_limit)"""
        return INA219._vbus_max if self.bus_voltage_range else 16, 0

    @staticmethod
    def get_switch_direction(low_range_limit: float, hi_range_limit: float, value: float) -> tuple:
        """Возвращает направление переключения входного делителя диапазона, вниз или вверх.
        Returns the switching direction of the input range divider, down or up."""
        switch_up = abs(value / hi_range_limit) > 0.95
        switch_down = abs(value) < 0.85*abs(low_range_limit)
        if switch_up and switch_down:
            raise ValueError(f"Invalid input parameters: {low_range_limit}; {hi_range_limit}")
        return switch_up, switch_down

    def get_shunt_voltage(self) -> float:
        """Смотри описание INA219Simple.get_shunt_voltage.
        See the description of INA219Simple.get_shunt_voltage"""
        volt = super().get_shunt_voltage()
        if self.auto_range:
            ...
        return volt

    def get_voltage(self) -> tuple:
        """Смотри описание INA219Simple.get_voltage.
        See the description of INA219Simple.get_voltage."""
        # voltage, data_ready_flag, math_ovf
        t = super().get_voltage()

        if self.auto_range and t[1]:    # if auto and data_ready:
            hi_lim, low_lim = self.get_bus_voltage_range()
            sw_up, sw_down = self.get_switch_direction(low_lim, hi_lim, t[0])
            if sw_up and not sw_down:
                self.bus_voltage_range = True
            if not sw_up and sw_down:
                self.bus_voltage_range = False
            # set new config
            self.set_config()
        return t

    @staticmethod
    def shunt_voltage_range_to_volt(shunt_voltage_range: int):
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

    def _calc(self, shunt_resistance: float, max_expected_current: float = None) -> tuple:
        """Вычисляет и возвращает содержимое калибровочного регистра, значения наименее значимых битов
        регистров тока и мощности, наибольший измеряемый ток через шунт в Ампер, наибольшую измеряемую мощность, Вт
        max_expected_current - наибольший ожидаемый ток через токовый шунт, Ампер
        shunt_resistance - сопротивление шунта, Ом.
        Если вы не знаете, какое значение передавать в max_expected_current, передайте None!
        -------------------------------------------------------------------------
        Calculates and returns the contents of the calibration register, the values of the least significant bits of
        the current and power registers, the largest measurable current through the shunt in Amps,
        the largest measurable power in watts.
        max_expected_current - maximum expected current through the current shunt, Ampere
        shunt_resistance - shunt resistance, Ohm.
        If you don't know what value to pass to max_expected_current, pass None!"""

        # расчет максимального измеряемого тока, Ампер
        maximum_current = INA219._shunt_voltage_limit / shunt_resistance
        if max_expected_current is None:
            max_expected_current = maximum_current

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

        self._max_shunt_voltage_before_ovf = max_shunt_voltage_before_ovf
        # Вычисляю максимальную мощность, Вт
        max_power = max_current_before_ovf * INA219._vbus_max

        return cal_val, lsb_current, lsb_power, maximum_current, max_power

    def calibrate(self, max_expected_current: float) -> int:
        """Производит расчеты и запись значения в регистр калибровки. Вызови _calc до вызова этого метода!!!
        Performs calculations and writes the value to the calibration register.
        Call _calc before calling this method!!!"""
        calibr_reg, self._lsb_current_reg, self._lsb_power_reg, self._maximum_current, self._maximum_power = \
            self._calc(shunt_resistance=self._shunt_res, max_expected_current=max_expected_current)
        self._set_calibr_reg(calibr_reg)
        if 0 >= self._max_shunt_voltage_before_ovf:
            raise ValueError("Invalid max_shunt_voltage_before_ovf value. Call _calc method before calibrate!")
        # автоустановка напряжения на шунте! self.max_shunt_voltage_before_ovf
        for sh_v_rng in range(4):
            if self.shunt_voltage_range_to_volt(sh_v_rng) >= self._max_shunt_voltage_before_ovf:
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

    @property
    def auto_range(self):
        return self._auto_range

    @auto_range.setter
    def auto_range(self, value):
        self._auto_range = value

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

    def soft_reset(self):
        self._set_raw_cfg(0b11100110011111)

    def __iter__(self):
        return self

    def __next__(self) -> tuple:
        """Возвращает измеренные значения. кортеж, число.
        Return measured values. tuple, digit"""
        return self.get_voltage(), self.get_shunt_voltage()

"""INAxxx Texas Instruments sensors module.

Внимание! для долговременной непрерывной работы токового шунта, не допускайте выделения на нем более половины(!) от его
максимальной рассеиваемой мощности!
Мощность, выделяемая на любом сопротивлении (постоянный ток), рассчитывается по формуле: P=I**2 * R
где: I - ток в Амперах; R - сопротивление в Омах.

Attention! for long-term continuous operation of the current shunt, do not allow more than half(!) of its maximum
dissipated power to be allocated on it!!
The power dissipated on any resistance (direct current) is calculated by the formula: P=I**2 * R
where: I - current in Amperes; R - resistance in ohms"""
import math
from sensor_pack_2 import bus_service
from sensor_pack_2.base_sensor import DeviceEx, IBaseSensorEx, Iterator, check_value

from collections import namedtuple
from sensor_pack_2.bitfield import bit_field_info
from sensor_pack_2.bitfield import BitFields

def get_exponent(value: float) -> int:
    """Возвращает десятичную степень числа.
    Returns the decimal power of a number"""
    return int(math.floor(math.log10(abs(value)))) if 0 != value else 0


def _get_sadc_field(config: int) -> int:
    """Возвращает поле SADC (токовый шунт) регистра конфигурации"""
    return (0x038 & config) >> 3

# расшифровка поля MODE, регистра конфигурации
# Если continuous в Истина, то измерения проводятся автоматически, иначе их нужно запускать принудительно!
#  Если bus_voltage_enabled в Истина, то измерения входного НАПРЯЖЕНИЯ производятся! Иначе не производятся!
#  Если shunt_voltage_enabled в Истина, то измерения входного ТОКА производятся! Иначе не производятся!
ina219_operation_mode = namedtuple("ina219_operation_mode", "continuous bus_voltage_enabled shunt_voltage_enabled")
# имена полей регистра конфигурации
config_ina219 = namedtuple("config_ina219", "BRNG PGA BADC SADC MODE")

def _decode_operation_mode(operation_mode: int) -> ina219_operation_mode:
    """Декодирует значение поля MODE регистра конфигурации в понятный именованный кортеж"""
    _continuous = operation_mode & 0b100
    _bus_voltage = operation_mode & 0b010
    _shunt_voltage = operation_mode & 0b001
    #
    return ina219_operation_mode(continuous=_continuous, bus_voltage_enabled=_bus_voltage,
                                 shunt_voltage_enabled=_shunt_voltage)

def _build_operating_mode(continuous: bool = True, enable_shunt_voltage: bool = True,
                          enable_bus_voltage: bool = True) -> int:
    """Преобразует режим измерения в сырой формат для INA219"""
    raw = 0
    cv = continuous and not (enable_shunt_voltage or enable_shunt_voltage)
    if cv:
        raise ValueError("Режим, со значением 4 в поле MODE, запрещен! Смотрите 'Table 6. Mode Settings'")
    if continuous:
        raw &= 0b100
    if enable_bus_voltage:
        raw &= 0b010
    if enable_shunt_voltage:
        raw &= 0b001

    return raw


class InaBase(DeviceEx):
    """Base class for INA current/voltage monitor"""

    def __init__(self, adapter: bus_service.BusAdapter, address):
        """"""
        check_value(address, range(0x40, 0x50), f"Неверный адрес устройства: {address}")
        super().__init__(adapter, address, True)  # All data bytes are transmitted most significant byte first.

    def _get_16bit_reg(self, address: int, format_char: str) -> int:
        _raw = self.read_reg(address, 2)
        return self.unpack(format_char, _raw)[0]

    # BaseSensor
    def _set_raw_cfg(self, value: int) -> int:
        """Установить сырую конфигурацию в регистре. Set raw configuration in register."""
        return self.write_reg(0x00, value, 2)

    def _get_raw_cfg(self) -> int:
        """Get raw configuration from register"""
        return self._get_16bit_reg(0x00, "H")


class INA219Simple(InaBase):
    """Класс для работы с датчиком TI INA219 без какой либо настройки!
    Диапазон измерения входного напряжения: 0..26 Вольт. Рекомендую 0..24 Вольта,
    дополнительно защита от выбросов напряжения!!!
    Диапазон измерения напряжения на токоизмерительном шунте: ±320 милливольт.
    Никаких настроек нет!
    ---------------------
    A class for working with a TI INA219 sensor without any configuration!
    Input voltage measurement range: 0-26 Volts.
    Voltage measurement range on the current measuring shunt: ±320 millivolts.
    There are no settings!"""

    # для вычислений
    _lsb_shunt_voltage = 1E-5   # 10 uV
    _lsb_bus_voltage = 4E-3     # 4 mV

    @staticmethod
    def get_shunt_adc_lsb()->float:
        """Возвращает цену младшего разряда АЦП токового шунта. Не изменяется при изменении разрядности, что странно!"""
        return INA219Simple._lsb_shunt_voltage

    @staticmethod
    def get_bus_adc_lsb()->float:
        """Возвращает цену младшего разряда АЦП напряжения на шине. Не изменяется при изменении разрядности, что странно!"""
        return INA219Simple._lsb_bus_voltage

    def __init__(self, adapter: bus_service.BusAdapter, address=0x40):
        super().__init__(adapter, address)
        # 0x399F    настройка по умолчанию, простое считывание двух напряжений (входное-на шине и токового шунта).
        # Входного напряжения и напряжения на токовом шунте. Непрерывное произведение измерений.
        # Bus Voltage Range:    32 V    ("Senses Bus Voltages from 0 to 26 V". From page 1 of datasheet.)
        # Shunt Voltage Range:  ±320 mV
        # Bus ADC Resolution:   12 bit
        # Conversion Time:      532 us
        # Shunt ADC Resolution: 12 bit
        # Mode:                 Shunt and bus, continuous
        self._set_raw_cfg(0b0011_1001_1001_1111)

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мкс(!) преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!"""
        return 532

    def get_shunt_voltage(self) -> float:
        """Возвращает напряжение на токовом(!) шунте в Вольтах. Чтобы вычислить ток через шунт,
        нужно это напряжение поделить на сопротивление шунта в Омах!!!
        Returns the shunt voltage in Volts. To calculate the current through the shunt, you need to divide this voltage
        by the resistance of the shunt in ohms !!!"""
        # DC ACCURACY. ADC basic resolution: 12 bit;:
        #               Shunt voltage, 1 LSB step size: 10 μV
        #               Bus voltage, 1 LSB step size:   4 mV
        return self.get_shunt_adc_lsb() * self._get_16bit_reg(0x01, "h")

    def get_voltage(self) -> tuple:
        """Возвращает кортеж из входного измеряемого напряжения,
        флага готовности данных, флага математического переполнения (OVF).
        Флаг математического переполнения (OVF) устанавливается, когда расчеты мощности или тока выходят за допустимые
        пределы. Это указывает на то, что данные о токе и мощности могут быть бессмысленными!
        ------------------------------------------------------------------------------
        Returns a tuple of input measured voltage, data ready flag, math overflow flag (OVF).
        The Math Overflow Flag (OVF) is set when power or current calculations are out of range.
        This indicates that current and power data may be meaningless!"""
        # DC ACCURACY:  ADC basic resolution: 12 bit;    Bus voltage, 1 LSB step size: 4 mV
        reg_raw = self._get_16bit_reg(0x02, "h")
        # print(f"reg_raw: {hex(reg_raw)}")
        #           voltage             data ready flag         math overflow flag
        return self.get_bus_adc_lsb() * (reg_raw >> 3), bool(reg_raw & 0x02), bool(reg_raw & 0x01)


class INA219(INA219Simple, IBaseSensorEx, Iterator):
    """Class for work with TI INA219 sensor"""

    # предел напряжения на шунте из документации, Вольт
    # shunt voltage limit, Volt
    _shunt_voltage_limit = 0.32
    # Предел измеряемого напряжения! И неважно, что чип измеряет до 32 Вольт!
    # В документации 26. Минус 1 вольт для запаса (Senses Bus Voltages from 0 to 26 V).
    # "Senses Bus Voltages from 0 to 26 V"
    # Measured voltage limit! And it doesn't matter that the chip measures up to 32 volts!
    # In the documentation 26. Minus 1 volt for a margin (Senses Bus Voltages from 0 to 26 V)
    _vbus_max = 25

    # описание регистра конфигурации
    _config_reg_ina219 = (bit_field_info(name='RST', position=range(15, 16), valid_values=None),    # Reset Bit
                           # Bus Voltage Range, 0 - 16 V; 1 - 32 V
                           bit_field_info(name='BRNG', position=range(13, 14), valid_values=None),
                           # PGA (Current Shunt Voltage Only). 0 - +/-40 mV; 1 - +/-80 mV; 2 - +/-160 mV; 3 - +/-320 mV;
                           bit_field_info(name='PGA', position=range(11, 13), valid_values=range(6)),
                           # Bus ADC Resolution/Averaging. These bits adjust the Bus ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when averaging results for the Bus Voltage Register (02h).
                           bit_field_info(name='BADC', position=range(7, 11), valid_values=None),
                           # Shunt ADC Resolution/Averaging. These bits adjust the Shunt ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when averaging results for the Shunt Voltage Register (01h).
                           bit_field_info(name='SADC', position=range(3, 7), valid_values=None),
                           # Operating Mode. Selects continuous, triggered, or power-down mode of operation. These bits default to continuous shunt and bus measurement mode.
                           bit_field_info(name='MODE', position=range(3), valid_values=tuple(i for i in range(8) if 4 != i)),
                           )


    def __init__(self, adapter: bus_service.BusAdapter, address=0x40, shunt_resistance: float = 0.1):
        """shunt_resistance - сопротивление шунта, [Ом].
        shunt_resistance - shunt resistance, [Ohm]"""
        super().__init__(adapter, address)
        # для удобства работы с настройками
        self._bit_fields = BitFields(fields_info=INA219._config_reg_ina219)
        # False - 16 V; True - 32 V
        self._bus_voltage_range = None
        # value     range, mV   Gain
        # 0         ±40 mV      1
        # 1         ±80 mV      1/2
        # 2         ±160 mV     1/4
        # 3         ±320 mV     1/8
        self._shunt_voltage_range = None
        # value     Resolution, bit     Conversion Time, us
        #   0       9                   84
        #   1       10                  148
        #   2       11                  276
        #   3       12                  532
        self._bus_adc_resolution = None
        # все как и у _bus_adc_resolution
        self._shunt_adc_resolution = None
        #   Value       Mode
        # ---------------------------------------
        #   0           ИС Выключена
        #   1           Измерение напряжение на токовом шунте (бит №0 == 1), однократный режим измерения (бит №2 == 0)
        #   2           Измерение входного напряжение на шине (бит №1 == 1), однократный режим измерения (бит №2 == 0)
        #   3           Измерение напряжение на шине и токовом шунте, однократный режим измерения (бит №2 == 0)
        #   4           АЦП выключен (режим запрещен!). Бит №2 = 1, Бит №1 = 0, Бит №0 = 0
        #   5           Измерение напряжение на токовом шунте (бит №0 == 1), непрерывный режим измерения (бит №2 == 1)
        #   6           Измерение входного напряжение на шине (бит №1 == 1), непрерывный режим измерения (бит №2 == 1)
        #   7           Измерение напряжение на шине и токовом шунте, непрерывный режим измерения (бит №2 == 1)
        self._operating_mode = None

        # если Истина, измерение тока, путем измерения напряжения на шунте, производится! режимы работы
        # if True, a current measurement, by measuring the voltage across the shunt, is done! operating modes
        # self.shunt_voltage = True
        # делаю "недоступным" для пользователя. это лишние настройки! Устанавливаю наибольшее разрешение!
        # # make it "unavailable" to the user. those are redundant settings. I set the highest resolution!
        # разрешение/усреднение АЦП напряж. на токовом шунте. 12 bit. Conversion time: 532 μs
        # self._current_adc_resolution = 3
        # разрешение/усреднение АЦП входного изменяемого напряж. 12 bit. Conversion time: 532 μs
        # self._voltage_adc_resolution = 3
        # dont set shunt_resistance to zero.zero and below zero!
        self._shunt_res = shunt_resistance
        #
        #self._lsb_current_reg = 0  # for calibrate method
        #self._lsb_power_reg = 0  # for calibrate method
        #self._maximum_current = 0  # from _calc method
        #self._maximum_power = 0  # from _calc method
        #self._max_shunt_voltage_before_ovf = 0   # from _calc method
        # автоматический выбор диапазона измерения напряжения шины и тока через токовый шунт
        # automatic selection of bus voltage and current measurement range via current shunt
        self._auto_range = False
        # max_expected_current - наибольший ожидаемый ток через токовый шунт.
        # Если max_expected_current==None, то происходит автоматическое вычисление этого тока. Он устанавливается в
        # максимальное значение
        # max_expected_current - the maximum expected current through the current shunt.
        # If max_expected_current==None, then this current is automatically calculated. It is set to the maximum value.
        #self._calc(shunt_resistance=shunt_resistance, max_expected_current=None)
        self._continuous = None
        self._shunt_voltage_enabled = None
        self._bus_voltage_enabled = None

    def get_config(self, return_value: bool = True) -> [config_ina219, None]:
        """Считывает настройками датчика по шине"""
        raw_config = self._get_raw_cfg()
        #
        bf = self._bit_fields
        bf.source = raw_config
        bf.field_name = 'BRNG'
        self._bus_voltage_range = bf.get_field_value()
        bf.field_name = 'PGA'
        self._shunt_voltage_range = bf.get_field_value()
        bf.field_name = 'BADC'
        self._bus_adc_resolution = bf.get_field_value()
        bf.field_name = 'SADC'
        self._shunt_adc_resolution = bf.get_field_value()
        bf.field_name = 'MODE'
        self._operating_mode = bf.get_field_value()
        #
        decoded_op_mode = _decode_operation_mode(self._operating_mode)
        #
        self._continuous = decoded_op_mode.continuous
        self._shunt_voltage_enabled = decoded_op_mode.shunt_voltage_enabled
        self._bus_voltage_enabled = decoded_op_mode.bus_voltage_enabled
        #
        if return_value:
            return config_ina219(BRNG=self._bus_voltage_range, PGA=self._shunt_voltage_range,
                                 BADC=self._bus_adc_resolution, SADC=self._shunt_adc_resolution,
                                 MODE=self._operating_mode)

    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement.
        Не забудь вызвать get_config!"""
        return self._operating_mode in range(1, 4)

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement.
        Не забудь вызвать get_config!"""
        return self._operating_mode > 3

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мкс(!) преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!
        Не забудь вызвать get_config!"""
        bf = self._bit_fields
        s_adc_field = bf.get_field_value()    # выделяю поле SADC (токовый шунт)
        # print(f"DBG:get_conversion_cycle_time: {s_adc_field}")
        conv_time = 84, 148, 276, 532
        if s_adc_field < 8:
            s_adc_field &= 0x3  # 0..3
            return conv_time[s_adc_field]
        # 0x8..0xF
        s_adc_field -= 0x08     # 0..7
        coefficient = 2 ** s_adc_field
        return 532 * coefficient


    @property
    def bus_voltage_range(self) -> bool:
        """Возвращает измеряемый диапазон напряжений на шине. Если Истина то диапазон 0..25 Вольт, иначе 0..16 Вольт.
        Returns the measured bus voltage range. If True then the range is 0..25 Volts, otherwise 0..16 Volts."""
        return self._bus_voltage_range

    def get_shunt_voltage(self) -> float:
        """Смотри описание INA219Simple.get_shunt_voltage.
        See the description of INA219Simple.get_shunt_voltage"""
        return super().get_shunt_voltage()

    def get_voltage(self) -> tuple:
        """Смотри описание INA219Simple.get_voltage.
        See the description of INA219Simple.get_voltage."""
        # voltage, data_ready_flag, math_ovf
        return super().get_voltage()

    def set_adc_resolution(self, bus_adc_resol: int, shunt_adc_resol: int):
        """Устанавливает разрешение АЦП на шине и разрешение АЦП токового шунта. Допустимые значения от 9 до 12 включительно."""
        r = range(9, 13)
        if bus_adc_resol:
            check_value(bus_adc_resol, r, f"Неверное разрешение АЦП напряжения на шине: {bus_adc_resol}")
        if shunt_adc_resol:
            check_value(shunt_adc_resol, r, f"Неверное разрешение АЦП тока нагрузки: {shunt_adc_resol}")
        self._bus_adc_resolution = bus_adc_resol
        self._shunt_adc_resolution = shunt_adc_resol

    @property
    def shunt_resistance(self):
        """Возвращает сопротивление токового шунта в Омах.
        Returns the resistance of the current shunt in ohms."""
        return self._shunt_res

    @property
    def current_shunt_voltage_range(self) -> int:
        """Возвращает установленный диапазон напряжения на шунте.
        # value     range, mV
        # 0         ±40 mV
        # 1         ±80 mV
        # 2         ±160 mV
        # 3         ±320 mV     default"""
        return self._shunt_voltage_range

    @current_shunt_voltage_range.setter
    def current_shunt_voltage_range(self, value):
        self._shunt_voltage_range = check_value(
            value, range(4),f"Неверный диапазон текущего напряжения шунта: {value}")

    def set_config(self) -> int:
        """Настраивает датчик в соответствии с настройками. Возвращает значение настроек в сыром виде"""
        bf = self._bit_fields
        bf.source = self._get_raw_cfg()
        #
        bf.field_name = 'BRNG'
        bf.set_field_value(value=self._bus_voltage_range)
        bf.field_name = 'PGA'
        bf.set_field_value(value=self.current_shunt_voltage_range)
        bf.field_name = 'BADC'
        bf.set_field_value(value=self.bus_adc_resolution)
        bf.field_name = 'SADC'
        bf.set_field_value(value=self.shunt_adc_resolution)

        _mode = _build_operating_mode(continuous=self._continuous,enable_bus_voltage=self._bus_voltage_enabled,
                              enable_shunt_voltage=self._shunt_voltage_enabled)
        bf.field_name = 'MODE'
        bf.set_field_value(value=_mode)
        #
        _cfg = bf.source
        self._set_raw_cfg(_cfg)
        #
        return _cfg

    @property
    def shunt_voltage_enabled(self) -> bool:
        return self._shunt_voltage_enabled

    @property
    def bus_voltage_enabled(self) -> bool:
        return self._bus_voltage_enabled

    @property
    def bus_adc_resolution(self) -> int:
        return self._bus_adc_resolution

    @property
    def shunt_adc_resolution(self) -> int:
        return self._shunt_adc_resolution

    # BaseSensor

    def soft_reset(self):
        self._set_raw_cfg(0b11100110011111)

    def __iter__(self):
        return self

    def __next__(self) -> [float, tuple]:
        """Возвращает измеренные значения. кортеж, число."""
        _shunt, _bus = None, None
        if self.shunt_voltage_enabled:
            _shunt = self.get_shunt_voltage()
        if self.bus_voltage_enabled:
            _bus = self.get_voltage()
        #
        _a = not _shunt is None
        _b = not _bus is None
        if _a and _b:
            return _bus, _shunt
        if _a:
            return _shunt
        if _b:
            return _bus
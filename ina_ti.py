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
from sensor_pack_2.base_sensor import BaseSensorEx, IBaseSensorEx, Iterator, check_value

from collections import namedtuple
from sensor_pack_2.bitfield import bit_field_info
from sensor_pack_2.bitfield import BitFields

def get_exponent(value: float) -> int:
    """Возвращает десятичную степень числа.
    Returns the decimal power of a number"""
    return int(math.floor(math.log10(abs(value)))) if 0 != value else 0


# расшифровка поля MODE, регистра конфигурации
# Если continuous в Истина, то измерения проводятся автоматически, иначе их нужно запускать принудительно!
#  Если bus_voltage_enabled в Истина, то измерения входного НАПРЯЖЕНИЯ производятся! Иначе не производятся!
#  Если shunt_voltage_enabled в Истина, то измерения входного ТОКА производятся! Иначе не производятся!
ina219_operation_mode = namedtuple("ina219_operation_mode", "continuous bus_voltage_enabled shunt_voltage_enabled")
# имена полей регистра конфигурации
#   Бит         Имя         Описание
#   13          BRNG        Диапазон напряжения для АЦП напряжения на шине (входное напряжение)
#   11..12      PGA         Диапазоны напряжения для АЦП токового шунта
#   7..10       BADC        Разрешение/Усреднение АЦП шины
#   3..6        SADC        Разрешение/Усреднение АЦП токового шунта
#   2           CNTNS       Непрерывный режим работы(1)/однократный режим работы(0)
#   1           BADC_EN     АЦП напряжения на шине (входное напряжение) включен (1)
#   0           SADC_EN     АЦП напряжения на токовом шунте включен (1)
config_ina219 = namedtuple("config_ina219", "BRNG PGA BADC SADC CNTNS BADC_EN SADC_EN")

def _get_conv_time(value: int) -> int:
    """Возвращает время из полей SADC, BADC в микросекундах"""
    _conv_time = 84, 148, 276, 532
    if value < 8:
        value &= 0x3  # 0..3
        return _conv_time[value]
    # 0x8..0xF. Усреднение по 2, 4, 8, 16, 32, 64, 128 отсчетам
    value -= 0x08  # 0..7
    coefficient = 2 ** value
    return 532 * coefficient

class InaBase(BaseSensorEx):
    """Base class for INA current/voltage monitor"""

    def __init__(self, adapter: bus_service.BusAdapter, address):
        """"""
        check_value(address, range(0x40, 0x50), f"Неверный адрес устройства: {address}")
        super().__init__(adapter, address, True)

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

    # BaseSensorEx
    def soft_reset(self):
        self._set_raw_cfg(0b11100110011111)


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
        # Shunt ADC Resolution: 12 bit
        # Conversion Time:      532 us
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
        """Возвращает кортеж из входного измеряемого напряжения, флага готовности данных, флага математического переполнения (OVF).
        Флаг математического переполнения (OVF) устанавливается, когда расчеты мощности или тока выходят за допустимые
        пределы. Это указывает на то, что данные о токе и мощности могут быть бессмысленными!
        ------------------------------------------------------------------------------
        Хотя данные последнего преобразования могут быть прочитаны в любое время, бит готовности к преобразованию указывает,
        когда  доступны данные преобразования в регистрах вывода данных. Бит готовности данных устанавливается после завершения всех(!) преобразований,
        усреднения и умножения. Он сбрасывается при следующих событиях:
            1) Запись нового режима в биты режима работы в регистре конфигурации (за исключением отключения или отключения питания).
            2) Чтение регистра мощности

        Бит готовности (CNVR) к преобразованию устанавливается после завершения всех(!) операций преобразования, усреднения и умножения!
        ------------------------------------------------------------------------------
        Returns a tuple of input measured voltage, data ready flag, math overflow flag (OVF).
        The Math Overflow Flag (OVF) is set when power or current calculations are out of range.
        This indicates that current and power data may be meaningless!"""
        # DC ACCURACY:  ADC basic resolution: 12 bit;    Bus voltage, 1 LSB step size: 4 mV
        reg_raw = self._get_16bit_reg(0x02, "h")
        # print(f"reg_raw: {hex(reg_raw)}")
        #           voltage             data ready flag         math overflow flag
        return self.get_bus_adc_lsb() * (reg_raw >> 3), bool(reg_raw & 0x02), bool(reg_raw & 0x01)


class INA219(INA219Simple, BaseSensorEx, IBaseSensorEx, Iterator):
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
    # разрешенные значения для полей BADC, SADC
    _vval = tuple(i for i in range(0x10) if i not in range(4, 8))
    # описание регистра конфигурации
    _config_reg_ina219 = (bit_field_info(name='RST', position=range(15, 16), valid_values=None, description="Сбрасывает все регистры в значениям по умолчанию."),    # Reset Bit
                          # Bus Voltage Range, 0 - 16 V; 1 - 32 V
                          bit_field_info(name='BRNG', position=range(13, 14), valid_values=None, description="Переключатель диапазонов измеряемого напряжения на шине."),
                          # PGA (Current Shunt Voltage Only). 0 - +/-40 mV; 1 - +/-80 mV; 2 - +/-160 mV; 3 - +/-320 mV;
                          bit_field_info(name='PGA', position=range(11, 13), valid_values=range(4), description="Переключатель диапазонов напряжения на токовом шунте."),
                          # Bus ADC Resolution/Averaging. These bits adjust the Bus ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when averaging results for the Bus Voltage Register (02h).
                          bit_field_info(name='BADC', position=range(7, 11), valid_values=_vval, description="Биты регулируют разрешение АЦП шины или устанавливают количество выборок для усреднении результатов."),
                          # Shunt ADC Resolution/Averaging. These bits adjust the Shunt ADC resolution (9-, 10-, 11-, or 12-bit) or set the number of samples used when averaging results for the Shunt Voltage Register (01h).
                          bit_field_info(name='SADC', position=range(3, 7), valid_values=_vval, description="Биты регулируют разрешение АЦП токового шунта или устанавливают количество выборок для усреднения результатов."),
                          # Operating Mode. Selects continuous, triggered, or power-down mode of operation. These bits default to continuous shunt and bus measurement mode.
                          # bit_field_info(name='MODE', position=range(3), valid_values=tuple(i for i in range(8) if 4 != i), description="Непрерывный, однократный режим работы или режим пониженного энергопотребления."),
                          bit_field_info(name='CNTNS', position=range(2, 3), valid_values=None, description='1 - Непрерывный режим работы датчика, 0 - по запросу'),
                          # Внимание хотя бы один(!) АЦП должен быть ВКЛЮЧЕН в непрерывном режиме измерений! Смотри "Table 6. Mode Settings"
                          bit_field_info(name='BADC_EN', position=range(1, 2), valid_values=None, description='1 - АЦП напряжения на шине включен, 0 - выключен'),
                          bit_field_info(name='SADC_EN', position=range(0, 1), valid_values=None, description='1 - АЦП напряжения на токовом шунте включен, 0 - выключен'),
                          )


    def __init__(self, adapter: bus_service.BusAdapter, address=0x40, shunt_resistance: float = 0.1):
        """shunt_resistance - сопротивление шунта, [Ом].
        shunt_resistance - shunt resistance, [Ohm]"""
        super().__init__(adapter, address)
        # для удобства работы с настройками
        self._bit_fields = BitFields(fields_info=INA219._config_reg_ina219)
        # сопротивление токового шунта в Омах!
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
        # self._continuous = True
        # self._shunt_voltage_enabled = True
        # self._bus_voltage_enabled = False

    def get_config(self, return_value: bool = True) -> [config_ina219, None]:
        """Считывает настройками датчика по шине"""
        raw_config = self._get_raw_cfg()
        #
        bf = self._bit_fields
        bf.source = raw_config
        #
        if return_value:
            return config_ina219(BRNG=self.bus_voltage_range, PGA=self.current_shunt_voltage_range,
                                 BADC=self.bus_adc_resolution, SADC=self.shunt_adc_resolution,
                                 CNTNS=self.continuous, BADC_EN=self.bus_adc_enabled,
                                 SADC_EN=self.shunt_adc_enabled,
                                 )
    def is_single_shot_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме однократных измерений,
        каждое из которых запускается методом start_measurement.
        Не забудь вызвать get_config!"""
        return not self.is_continuously_mode()

    def is_continuously_mode(self) -> bool:
        """Возвращает Истина, когда датчик находится в режиме многократных измерений,
        производимых автоматически. Процесс запускается методом start_measurement.
        Не забудь вызвать get_config!"""
        return self._bit_fields['CNTNS']

    @property
    def continuous(self) -> bool:
        return self.is_continuously_mode()

    @continuous.setter      # qqq
    def continuous(self, value: bool):
        if value and not (self.bus_adc_enabled or self.shunt_adc_enabled):
            raise ValueError('В непрерывном режиме измерения хотя бы один АЦП должен быть включен!')
        self._bit_fields['CNTNS'] = value
        # self._continuous = value

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время в мкс(!) преобразования сигнала в цифровой код и готовности его для чтения по шине!
        Для текущих настроек датчика. При изменении настроек следует заново вызвать этот метод!
        Не забудь вызвать get_config!"""
        _t0, _t1 = 0, 0
        # bf = self._bit_fields
        if self.shunt_adc_enabled:
            adc_field = self.shunt_adc_resolution   #   bf['SADC']    # выделяю поле SADC (токовый шунт)
            _t0 = _get_conv_time(adc_field)
            print(f"DBG:get_conversion_cycle_time SADC: {adc_field}")
        if self.bus_adc_enabled:
            adc_field = self.bus_adc_resolution   #   bf['BADC']    # выделяю поле BADC (напряжение на шине)
            _t1 = _get_conv_time(adc_field)
            print(f"DBG:get_conversion_cycle_time BADC: {adc_field}")
        # возвращаю наибольшее значение, поскольку измерения производятся параллельно, как утверждает документация
        return max(_t0, _t1)


    def start_measurement(self, continuous: bool = True, enable_shunt_adc: bool = True,
                          enable_bus_adc: bool = True, current_auto_range = True):
        """Настраивает параметры датчика и запускает процесс измерения.
        continuous - если Истина, то новое измерение запускается автоматически после завершения предидущего
        enable_shunt_voltage - включить измерение напряжения на токовом шунте
        enable_bus_voltage - включить измерение напряжения на шине
        shunt_adc_resol, bus_adc_resol - кол-во бит/отсчетов измерения напряжения на шине и напряжения на токовом шунте
        current_auto_range - измерение напряжения на токовом шунте с автоматическим выбором его предела
        """
        self.bus_adc_enabled = enable_bus_adc
        self.shunt_adc_enabled = enable_shunt_adc
        self.continuous = continuous    # устанавливайте этот бит после bus_adc_enabled и shunt_adc_enabled
        # self.bus_adc_resolution = bus_adc_resol
        # self.shunt_adc_resolution = shunt_adc_resol
        self._auto_range = current_auto_range
        #
        cfg = self.set_config()
        print(f"DBG: set_config return: 0x{cfg:X}")

    @property
    def auto_range(self) -> bool:
        """Автовыбор предела падения напряжения на шунте"""
        return self._auto_range

    @property
    def bus_voltage_range(self) -> bool:
        """Возвращает измеряемый диапазон напряжений на шине. Если Истина то диапазон 0..25 Вольт, иначе 0..16 Вольт.
        Не забудь вызвать get_config!
        Returns the measured bus voltage range. If True then the range is 0..25 Volts, otherwise 0..16 Volts."""
        return self._bit_fields['BRNG']

    @bus_voltage_range.setter
    def bus_voltage_range(self, value: bool):
        self._bit_fields['BRNG'] = value

    def get_shunt_voltage(self) -> float:
        """Смотри описание INA219Simple.get_shunt_voltage.
        See the description of INA219Simple.get_shunt_voltage"""
        return super().get_shunt_voltage()

    def get_voltage(self) -> tuple:
        """Смотри описание INA219Simple.get_voltage.
        See the description of INA219Simple.get_voltage."""
        # voltage, data_ready_flag, math_ovf
        return super().get_voltage()

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
        # 3         ±320 mV     default
        Не забудь вызвать get_config!"""
        return self._bit_fields['PGA']

    @current_shunt_voltage_range.setter
    def current_shunt_voltage_range(self, value):
        """Устанавливает диапазон напряжения на шунте 0..3.
        Не забудь вызвать get_config!
        # value     range, mV
        # 0         ±40 mV
        # 1         ±80 mV
        # 2         ±160 mV
        # 3         ±320 mV"""
        self._bit_fields['PGA'] = value

    def set_config(self) -> int:
        """Настраивает датчик в соответствии с настройками. Возвращает значение настроек в сыром виде"""
        bf = self._bit_fields
        _cfg = bf.source
        print(f"DBG: _set_raw_cfg: 0x{_cfg:X}")
        self._set_raw_cfg(_cfg)
        #
        return _cfg

    @property
    def shunt_adc_enabled(self) -> bool:
        """Если Истина, то АЦП напряжения на токовом шунте включен!
        Не забудь вызвать get_config!"""
        return self._bit_fields['SADC_EN']

    @shunt_adc_enabled.setter
    def shunt_adc_enabled(self, value: bool):
        self._bit_fields['SADC_EN'] = value

    @property
    def bus_adc_enabled(self) -> bool:
        """Если Истина, то АЦП напряжения на шине включен!
        Не забудь вызвать get_config!"""
        return self._bit_fields['BADC_EN']

    @bus_adc_enabled.setter
    def bus_adc_enabled(self, value: bool):
        self._bit_fields['BADC_EN'] = value

    @property
    def bus_adc_resolution(self) -> int:
        """Разрешение АЦП напряжения на шине.
        0 - 9 бит
        1 - 10 бит
        2 - 11 бит
        3 - 12 бит
        4, 8 - 12 бит
        9..15 - количество отсчетов, которое используется для усреднения результата. 9 - 2 отсчета; 15 - 128 отсчетов,
        смотри 'Table 5. ADC Settings'
        Не забудь вызвать get_config!"""
        return self._bit_fields['BADC']

    @bus_adc_resolution.setter
    def bus_adc_resolution(self, value: int):
        self._bit_fields['BADC'] = value

    @property
    def shunt_adc_resolution(self) -> int:
        """Разрешение АЦП напряжения на токовом шунте.
        0 - 9 бит
        1 - 10 бит
        2 - 11 бит
        3 - 12 бит
        4, 8 - 12 бит
        9..15 - количество отсчетов, которое используется для усреднения результата. 9 - 2 отсчета; 15 - 128 отсчетов,
        смотри 'Table 5. ADC Settings'
        Не забудь вызвать get_config!"""
        return self._bit_fields['SADC']

    @shunt_adc_resolution.setter
    def shunt_adc_resolution(self, value: int):
        self._bit_fields['SADC'] = value

    def __iter__(self):
        return self

    def __next__(self) -> [float, tuple]:
        """Возвращает измеренные значения. кортеж, число."""
        _shunt, _bus = None, None
        if self.shunt_adc_enabled:
            _shunt = self.get_shunt_voltage()
        if self.bus_adc_enabled:
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
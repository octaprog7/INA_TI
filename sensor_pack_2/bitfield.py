# micropython
# MIT license
# Copyright (c) 2024 Roman Shevchik   goctaprog@gmail.com
"""Представление битового поля"""
from collections import namedtuple
from sensor_pack_2.base_sensor import check_value, get_error_str

# информация о битовом поле в виде именованного кортежа
# name: str  - имя
# position: range - место в номерах битах. position.start = первый бит, position.stop-1 - последний бит
# valid_values: range - диапазон допустимых значений, если проверка не требуется, следует передать None
bit_field_info = namedtuple("bit_field_info", "name position valid_values")


def _bitmask(bit_rng: range) -> int:
    """возвращает битовую маску по занимаемым битам"""
    # if bit_rng.step < 0 or bit_rng.start <= bit_rng.stop:
    #    raise ValueError(f"_bitmask: {bit_rng.start}; {bit_rng.stop}; {bit_rng.step}")
    return sum(map(lambda x: 2 ** x, bit_rng))


class BitFields:
    """Хранилище информации о битовых полях с доступом по индексу.
    _source - кортеж именованных кортежей, описывающих битовые поля;"""
    def __init__(self, fields_info: tuple[bit_field_info, ...]):
        self._fields_info = fields_info
        self._idx = 0
        # имя битового поля, которое будет параметром у методов get_value/set_value
        self._active_field_name = fields_info[0].name
        # значение, из которого будут извлекаться битовые поля
        self._source_val = 0

    def _get_field(self, field: [str, int, None]) -> bit_field_info:
        """для внутреннего использования"""
        return self.__getitem__(field if field else self.field_name)

    def _get_source(self, source: [int, None]) -> int:
        return source if source else self._source_val

    @property
    def source(self) -> int:
        """значение, из которого будут извлекаться/в котором будут изменятся битовые поля"""
        return self._source_val

    @source.setter
    def source(self, value):
        """значение, из которого будут извлекаться/изменятся битовые поля"""
        self._source_val = value

    @property
    def field_name(self) -> str:
        """имя битового поля, значение которого извлекается/изменяется методами get_value/set_value, если их
        параметр field is None"""
        return self._active_field_name

    @field_name.setter
    def field_name(self, value):
        """имя битового поля, значение которого извлекается/изменяется методами get_value/set_value, если их
        параметр field is None"""
        self._active_field_name = value

    def _by_name(self, name: str) -> [bit_field_info, None]:
        """возвращает информацию о битовом поле по его имени (поле name именованного кортежа) или None"""
        items = self._fields_info
        for item in items:
            if name == item.name:
                return item

    def __len__(self) -> int:
        return len(self._fields_info)

    def __getitem__(self, key: [int, str]) -> [bit_field_info, None]:
        """возвращает информацию о битовом поле по его имени/индексу или None"""
        fi = self._fields_info
        if isinstance(key, int):
            return fi[key]
        if isinstance(key, str):
            return self._by_name(key)

    def set_field_value(self, value: int, source: [int, None] = None, field: [str, int, None] = None,
                        validate: bool = True) -> int:
        """Записывает value в битовый диапазон, определяемый параметром field, в source.
        Возвращает значение с измененным битовым полем.
        Если field is None, то имя поля берется из свойства self._active_field_name.
        Если source is None, то значение поля, подлежащее изменению, изменяется в свойстве self._source_val"""
        item = self._get_field(field=field)
        rng = item.valid_values
        if rng and validate:
            check_value(value, rng, get_error_str("value", value, rng))
        pos = item.position
        bitmask = _bitmask(pos)
        src = self._get_source(source) & ~bitmask  # чистка битового диапазона
        src |= (value << pos.start) & bitmask  # установка битов в заданном диапазоне
        # print(f"DBG:set_field_value: {value}; {source}; {field}")
        if source is None:
            self._source_val = src
            # print(f"DBG:set_field_value: self._source_val: {self._source_val}")
        return src

    def get_field_value(self, validate: bool = False) -> [int, bool]:
        """возвращает значение битового поля, по его имени(self.field_name), из self.source."""
        f_name = self.field_name
        item = self._get_field(f_name)
        if item is None:
            raise ValueError(f"get_field_value. Поле с именем {f_name} не существует!")
        pos = item.position
        bitmask = _bitmask(pos)
        val = (self.source & bitmask) >> pos.start  # выделение маской битового диапазона и его сдвиг вправо
        if item.valid_values and validate:
            raise NotImplemented("get_value validate")
        if 1 == len(pos):
            return 0 != val     # bool
        return val              # int

    # протокол итератора
    def __iter__(self):
        return self

    def __next__(self) -> bit_field_info:
        ss = self._fields_info
        try:
            self._idx += 1
            return ss[self._idx - 1]
        except IndexError:
            self._idx = 0   # для возможности выполнения повторной итерации!
            raise StopIteration

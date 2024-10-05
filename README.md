Модуль MicroPython для управления INA219 - Двунаправленный монитор тока/мощности с интерфейсом I2C.

# Внимание
Если вы не знаете закон Ома, то это чтиво не для Вас!

# Описание
INA219 — это монитор тока и мощности с интерфейсом I2C. Устройство измеряет как падение напряжения на шунте, так и 
напряжение питания на шине с программируемым временем преобразования и фильтрацией. Программируемое значение калибровки 
в сочетании с внутренним умножителем позволяет напрямую считывать ток в амперах. Дополнительный умножитель вычисляет мощность в ваттах.

# Применения
* Серверы
* Телекоммуникационное оборудование
* Ноутбуки
* Управление питанием
* Зарядные устройства
* Сварочное оборудование
* Источники питания
* Испытательное оборудование

# Питание
Напряжение питания 3.3, 5.0 В (от 2.7 В до 5.5 В)!

## Адрес датчика
Диапазон адресов датчика: 0x40..0x4F.

# Шина I2C
Просто подключите контакты (VCC, GND, SDA, SCL) платы с INA219 к соответствующим контактам Arduino, 
ESP или любой другой платы с прошивкой MicroPython! Правильно подключите нагрузку, ток которой должен пройти через шунт,
установленный на плате. Обычно это чип-резистор, сопротивлением 0.1 Ом. Подайте ток в нагрузку, при этом напряжение на шунте
не должно превысить 0.32 Вольта и шунт не должен перегреваться! 
Подайте питание на плату!

# Загрузка ПО в плату
Загрузите прошивку micropython на плату NANO(ESP и т. д.), а затем файлы: main.py, ina_ti.py и папку sensor_pack_2 полностью!
Затем откройте main.py в своей IDE и запустите/выполните его.

# Режимы работы монитора тока и напряжения
## Ручной
На каждое измерение нужен вызов метода start_measurement(continuous = False, .. .

## Автоматический
Вызовом метода start_measurement, датчик переводится в режим автоматического выполнения измерений. start_measurement(continuous = True, .. .

## Параметры в методах
### def start_measurement(continuous: bool, enable_calibration: bool, enable_shunt_adc: bool, enable_bus_adc: bool): 

* continuous - если Истина, то новое измерение запускается автоматически после завершения предидущего
* enable_calibration - если Истина, то происходит калибровка под заданное сопротивление шунта и ток в нагрузке
* enable_shunt_adc - включить измерение напряжения на токовом шунте
* enable_bus_adc - включить измерение напряжения на шине

Остальные параметры должны быть установлены до вызова start_measurement! 

### def current_shunt_voltage_range(self) -> int:
Возвращает число от 0 до 3.   

| raw voltage range | voltage range, мВ |
|-------------------|-------------------|
| 0                 | ±40               |	
| 1                 | ±80               |
| 2                 | ±160	             |   
| 3                 | ±320	             |

### def bus_adc_resolution(self) -> int:
Возвращает разрешение АЦП на шине в сыром виде, 0..15. То же самое для метода shunt_adc_resolution(self).

| raw | разрешение/кол-во отсчетов | Время преобразования |
|-----|----------------------------|----------------------|
| 0   | 9                          | 84 мкс               |
| 1   | 10                         | 148 мкс              |
| 2   | 11	                        | 276 мкс              |
| 3   | 12	                        | 532 мкс              |
| 8   | 12                         | 532 мкс              |
| 9   | 2*	                        | 1.06 мс              |
| 10  | 4*	                        | 2.13 мс              |
| 11  | 8*	                        | 4.26 мс              |
| 12  | 16*                        | 8.51 мс              |
| 13  | 32*                        | 17.02 мс             |
| 14  | 64*                        | 34.05 мс             |
| 15  | 128*                       | 68.10 мс             |

'*' - количество усредняемых отсчетов.

# Предупреждение для INA219
Никогда не подавайте на вывод Vin(+), Vin(-) напряжение больше 26 Вольт!

# Ветка experimental
Файл ina_ty.py содержит два класса. Это INA219 и INA226.

# Плата с INA219
![alt text](https://github.com/octaprog7/INA_TI/blob/master/pics/board.jpg)
# Среда разработки (IDE)
## IDE
![alt text](https://github.com/octaprog7/INA_TI/blob/master/pics/ide_0.png)

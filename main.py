import utime
from machine import I2C
from sensor_pack.bus_service import I2cAdapter
import ina_ti


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    # Warning!!!
    # Replace id=1 with id=0 if you are using the first I2C port !!!
    i2c = I2C(id=1, freq=400_000)  # on Arduino Nano RP2040 Connect tested
    adaptor = I2cAdapter(i2c)
    # bme - sensor
    ina219 = ina_ti.INA219Simple(adaptor)
    print(f"\tshunt voltage: {ina219.get_shunt_voltage()}")
    print(f"\tbus voltage: {ina219.get_voltage()}")
    #
    # while True:
    #    shunt_v = ina219.get_shunt_voltage()
    #    t = ina219.get_voltage()
    #    print(f"Shunt voltage: {shunt_v} V; Bus voltage: {t[0]} V; data ready flag: {t[1]}; overflow flag: {t[2]}")
    #    utime.sleep_ms(1000)
    del ina219
    ina219 = ina_ti.INA219(adapter=adaptor, address=0x40, shunt_resistance=0.1)
    ina219.bus_voltage_range = False    # 16 V
    ina219.shunt_voltage = False        # skip meas shunt voltage
    ina219.calibrate(max_expected_current=1.0)
    #
    while True:
        shunt_v = ina219.get_shunt_voltage()
        t = ina219.get_voltage()
        print(f"Shunt voltage: {shunt_v} V; Bus voltage: {t[0]} V; data ready flag: {t[1]}; overflow flag: {t[2]}")
        utime.sleep_ms(333)

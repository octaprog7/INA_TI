import utime
import sys
from machine import I2C
from sensor_pack_2.bus_service import I2cAdapter
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
    cfg = ina219.get_config()
    print(f"config: {cfg}")
    print(f"\tshunt voltage: {ina219.get_shunt_voltage()}")
    print(f"\tbus voltage: {ina219.get_voltage()}")
    #
    wait_time_us = ina219.get_conversion_cycle_time()
    print(f"wait_time_us: {wait_time_us} мкс.")
    for _ in range(100):
        shunt_v = ina219.get_shunt_voltage()
        t = ina219.get_voltage()
        print(f"Shunt voltage: {shunt_v} V; Bus voltage: {t[0]} V; data ready flag: {t[1]}; overflow flag: {t[2]}")
        utime.sleep_us(wait_time_us)
    del ina219
    
    sys.exit(0)
    
    ina219 = ina_ti.INA219(adapter=adaptor, address=0x40, shunt_resistance=0.1)
    ina219.bus_voltage_range = False    # 16 V
    ina219.shunt_voltage = True        	# skip meas shunt voltage
    # ina219.current_shunt_voltage_range = 2		# 160 mV
    ina219.set_config()
    ina219.calibrate(max_expected_current=1.0)		# 1.0 A * 0.1 Ohm = 0.1 Volt max on shunt resistance!

    while True:
        shunt_v = ina219.get_shunt_voltage()
        t = ina219.get_voltage()
        print(f"Shunt voltage: {shunt_v} V; Bus voltage: {t[0]} V; data ready flag: {t[1]}; overflow flag: {t[2]}")
        print(f"Bus voltage: {t[0]} V; Current: {ina219.get_current()} Amper; Power: {ina219.get_power()} Watt")
        utime.sleep_ms(333)

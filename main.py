import time
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
    # cfg = ina219.get_config()
    # print(f"config: {cfg}")
    print(f"\tshunt voltage: {ina219.get_shunt_voltage()}")
    print(f"\tbus voltage: {ina219.get_voltage()}")
    #
    wait_time_us = ina219.get_conversion_cycle_time()
    print(f"wait_time_us: {wait_time_us} мкс.")
    for _ in range(1_000_000_000_000):
        shunt_v = ina219.get_shunt_voltage()
        t = ina219.get_voltage()
        curr = shunt_v / 0.1
        print(f"Shunt: {shunt_v} V;\tBus: {t[0]} V;\tcurr: {curr} A;\tdata rdy flag: {t[1]};\tovf flag: {t[2]}")
        time.sleep_us(wait_time_us)
        time.sleep_ms(100)
    del ina219
    
    # sys.exit(0)

    print(32 * "-")
    ina219 = ina_ti.INA219(adapter=adaptor, address=0x40, shunt_resistance=0.1)
    # ina219.bus_voltage_range = False    # 16 V
    ina219.bus_voltage_enabled = True
    ina219.shunt_voltage_enabled = True
    # ina219.shunt_voltage = True        	# skip meas shunt voltage
    # ina219.current_shunt_voltage_range = 2		# 160 mV
    # ina219.set_config()
    # ina219.calibrate(max_expected_current=1.0)		# 1.0 A * 0.1 Ohm = 0.1 Volt max on shunt resistance!
    ina219.bus_adc_resolution = 0x0A
    ina219.shunt_adc_resolution = 0x0A
    ina219.current_shunt_voltage_range = 3
    ina219.start_measurement(continuous=True)
    cfg = ina219.get_config()
    print(f"configuration: {cfg}")
    # sys.exit(0)
    # print(f"operating mode: {ina219.operating_mode}")
    wait_time_us = ina219.get_conversion_cycle_time()
    print(f"wait_time_us: {wait_time_us} мкс.")
    while True:
        shunt_v = ina219.get_shunt_voltage()
        t = ina219.get_voltage()
        print(f"Shunt: {shunt_v} V; Bus: {t[0]} V; data rdy flag: {t[1]}; ovf flag: {t[2]}")
        # print(f"Bus voltage: {t[0]} V; Current: {ina219.get_current()} Amper; Power: {ina219.get_power()} Watt")
        time.sleep_us(wait_time_us)
        # sys.exit(0)

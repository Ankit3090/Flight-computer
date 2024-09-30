from bmp280 import *
from machine import Pin, I2C
import utime

ERROR = 0

sclPin = Pin(1)
sdaPin = Pin(0)

i2c_object = I2C(0, scl=sclPin, sda=sdaPin, freq=100000)

bmp280_object = BMP280(i2c_object, addr=0x76, use_case=BMP280_CASE_WEATHER)

bmp280_object.power_mode = BMP280_POWER_NORMAL
bmp280_object.oversample = BMP280_OS_HIGH
bmp280_object.temp_os = BMP280_TEMP_OS_8
bmp280_object.press_os = BMP280_TEMP_OS_4
bmp280_object.standby = BMP280_STANDBY_250
bmp280_object.iir = BMP280_IIR_FILTER_2

print("BMP Object created successfully !")
utime.sleep(2)
print("\n")

def altitude_HYP(hPa, temperature):
    temperature_k = temperature + 273.15
    pressure_ratio = 1013.25 / hPa
    h = (((pressure_ratio ** (1 / 5.257)) - 1) * temperature_k) / 0.0065
    return h

def altitude_IBF(pressure):
    pressure_ratio = pressure / 1013.25
    altitude = 44330 * (1 - (pressure_ratio ** (1 / 5.255)))
    return altitude 

while True:
    temperature = bmp280_object.temperature
    pressure_hPa = bmp280_object.pressure / 100.0
    h = altitude_HYP(pressure_hPa, temperature)
    altitude = altitude_IBF(pressure_hPa)

     print("Temperature: {:2f} C".format(temperature))
    
    print("Pressure: {:2f} hPa".format(pressure_hPa))
    print("Altitude (HYP): {:2f} meters".format(h))
    print(h)
    print(altitude)
    print("\n")

    utime.sleep(1)

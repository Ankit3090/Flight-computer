from machine import Pin, I2C, UART
from bmp280 import *
import machine
import utime, time
cs = machine.Pin(1, machine.Pin.OUT)
i2c = machine.I2C(id=0, sda=machine.Pin(0), scl=machine.Pin(1))
MPU6050_ADDR = 0x68
MPU6050_ADDR1 = 0x69
MPU6050_PWR_MGMT_1 = 0x6B
MPU6050_ACCEL_XOUT_H = 0x3B
MPU6050_ACCEL_XOUT_L = 0x3C
MPU6050_ACCEL_YOUT_H = 0x3D
MPU6050_ACCEL_YOUT_L = 0x3E
MPU6050_ACCEL_ZOUT_H = 0x3F
MPU6050_ACCEL_ZOUT_L = 0x40
MPU6050_LSBG = 16384.0
bus = machine.I2C(id=0,scl=Pin(1),sda=Pin(0),freq=200000)
bmp = BMP280(bus)
bmp.use_case(BMP280_CASE_INDOOR)
gpsModule = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
print(gpsModule)

buff = bytearray(255)

TIMEOUT = False
FIX_STATUS = False

latitude = ""
longitude = ""
satellites = ""
GPStime = ""
def mpu6050_init(i2c):
    i2c.writeto_mem(MPU6050_ADDR, MPU6050_PWR_MGMT_1, bytes([0]))
    i2c.writeto_mem(MPU6050_ADDR1, MPU6050_PWR_MGMT_1, bytes([0]))
def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) |  (l[0] ^ 255) + 1
def mpu6050_get_accel1(i2c):
    accel_x_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l = i2c.readfrom_mem(MPU6050_ADDR, MPU6050_ACCEL_ZOUT_L, 1)
    return [combine_register_values(accel_x_h, accel_x_l) / MPU6050_LSBG,
            combine_register_values(accel_y_h, accel_y_l) / MPU6050_LSBG,
            combine_register_values(accel_z_h, accel_z_l) / MPU6050_LSBG]
def mpu6050_get_accel2(i2c):
    accel_x_h1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_XOUT_H, 1)
    accel_x_l1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_XOUT_L, 1)
    accel_y_h1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_YOUT_H, 1)
    accel_y_l1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_YOUT_L, 1)
    accel_z_h1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_ZOUT_H, 1)
    accel_z_l1 = i2c.readfrom_mem(MPU6050_ADDR1, MPU6050_ACCEL_ZOUT_L, 1)
    return [combine_register_values(accel_x_h1, accel_x_l1) / MPU6050_LSBG,
            combine_register_values(accel_y_h1, accel_y_l1) / MPU6050_LSBG,
            combine_register_values(accel_z_h1, accel_z_l1) / MPU6050_LSBG]
f_ax=0
al=0.85
a=[]
b=[]
def getGPS(gpsModule):
    global FIX_STATUS, TIMEOUT, latitude, longitude, satellites, GPStime
    
    timeout = time.time() + 8 
    while True:
        gpsModule.readline()
        buff = str(gpsModule.readline())
        parts = buff.split(',')
    
        if (parts[0] == "b'$GPGGA" and len(parts) == 15):
            if(parts[1] and parts[2] and parts[3] and parts[4] and parts[5] and parts[6] and parts[7]):
                print(buff)
                
                latitude = convertToDegree(parts[2])
                if (parts[3] == 'S'):
                    latitude = -latitude
                longitude = convertToDegree(parts[4])
                if (parts[5] == 'W'):
                    longitude = -longitude
                satellites = parts[7]
                GPStime = parts[1][0:2] + ":" + parts[1][2:4] + ":" + parts[1][4:6]
                FIX_STATUS = True
                break
                
        if (time.time() > timeout):
            TIMEOUT = True
            break
        utime.sleep_ms(500)
def convertToDegree(RawDegrees):

    RawAsFloat = float(RawDegrees)
    firstdigits = int(RawAsFloat/100) 
    nexttwodigits = RawAsFloat - float(firstdigits*100) 
    
    Converted = float(firstdigits + nexttwodigits/60.0)
    Converted = '{0:.6f}'.format(Converted) 
    return str(Converted)
def low(a,b):
    return  [(al*a[0]+(1-al)*b[0]),(al*a[1]+(1-al)*b[1]),(al*a[2]+(1-al)*b[2])]
if __name__ == "__main__":
    i2c = machine.I2C(id=0, sda=machine.Pin(0), scl=machine.Pin(1))
    mpu6050_init(i2c)
    
    while True:
        L1=mpu6050_get_accel1(i2c)
        L2=mpu6050_get_accel2(i2c)
        pressure=bmp.pressure
        p_bar=pressure/100000
        p_mmHg=pressure/133.3224
        temperature=bmp.temperature
        local_pressure = p_bar*1000 # Unit : hPa
        sea_level_pressure = 1013.25 # Unit : hPa
        pressure_ratio = local_pressure / sea_level_pressure
        altitude = 44330*(1-(pressure_ratio**(1/5.255)))
        L3=low(L1,mpu6050_get_accel1(i2c))
        L4=low(L2,mpu6050_get_accel2(i2c))
        getGPS(gpsModule)
        acc1x=L3[0]
        acc1y=L3[1]
        acc1z=L3[2]
        acc2x=L3[0]
        acc2y=L3[1]
        acc2z=L3[2] 
        print("Acceleration 1: [{},{},{}] g".format(acc1x,acc1y,acc1z))
        print("Acceleration 2: [{},{},{}] g".format(acc2x,acc2y,acc2z))
        print("Pressure:", pressure)
        print("Temperature: ", temperature)
        print("Altitude: ",altitude)
        print("Latitude: "+latitude)
        print("Longitude: "+longitude)
        print("Satellites: " +satellites)
        print("Time: "+GPStime)
        print("----------------------")
            
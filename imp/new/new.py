import time
from imu import MPU6050
from bmp280 import *
from machine import I2C, Pin
from utime import sleep
import utime
from math import degrees, atan2, sqrt

# Define low-pass filter constants
alpha = 0.98  # Low-pass filter coefficient for accelerometer (adjust as needed)
dt = 0.02     # Time interval (adjust according to your sampling rate)

# Create instances of MPU6050 for each sensor
i2c1 = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2c2 = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
mpu1 = MPU6050(i2c1)
mpu2 = MPU6050(i2c2)

# Initialize variables to store the sensor data
accel_data1 = [0, 0, 0]
gyro_data1 = [0, 0, 0]
accel_data2 = [0, 0, 0]
gyro_data2 = [0, 0, 0]

# Initialize orientation variables
angle = [0, 0, 0]

# Number of readings to average
num_readings = 10

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

# Function to apply a low-pass filter
def low_pass_filter(filtered, raw, alpha):
    for i in range(3):
        filtered[i] = alpha * raw[i] + (1.0 - alpha) * filtered[i]

# Kalman filter class for accelerometer data
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, initial_estimate):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.estimate_error = 1

    def update(self, measurement):
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance

        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error

        return self.estimate

# Kalman filter variables for yaw
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03
angle = 0.0
bias = 0.0
P = 0.0

# Create Kalman filters for accelerometer data
accel_x_filter1 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)
accel_y_filter1 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)
accel_z_filter1 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)
accel_x_filter2 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)
accel_y_filter2 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)
accel_z_filter2 = KalmanFilter(process_variance=0.001, measurement_variance=0.02, initial_estimate=0)

# Kalman filter for yaw
def kalman_filter(yaw_measurement, angle, bias, P):
    # Prediction
    rate = gyro_data1[2] - bias
    angle += dt * rate
    P += Q_angle

    # Update
    S = P + R_measure
    K = P / S
    angle += K * (yaw_measurement - angle)
    bias += K * rate
    P -= K * S

    return angle, bias, P
def altitude_HYP(hPa, temperature):
    temperature_k = temperature + 273.15
    pressure_ratio = 1013.25 / hPa
    h = (((pressure_ratio ** (1 / 5.257)) - 1) * temperature_k) / 0.0065
    return h

def altitude_IBF(pressure):
    pressure_ratio = pressure / 1013.25
    altitude = 44330 * (1 - (pressure_ratio ** (1 / 5.255)))
    return altitude
# Main loop

    
        

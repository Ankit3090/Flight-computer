# Import necessary libraries
import time
from imu import MPU6050
from machine import I2C, Pin
from utime import sleep
from math import degrees

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

# Number of readings to average
num_readings = 10

# Function to apply a low-pass filter
def low_pass_filter(filtered, raw, alpha):
    for i in range(3):
        filtered[i] = alpha * raw[i] + (1.0 - alpha) * filtered[i]

# Kalman filter variables for yaw
Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03
angle = 0.0
bias = 0.0
P = 0.0

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

# Main loop
while True:
    for _ in range(num_readings):
        # Read accelerometer and gyro data from each sensor
        accel_data1 = [sum(x) for x in zip(accel_data1, mpu1.accel.xyz)]
        gyro_data1 = [sum(x) for x in zip(gyro_data1, mpu1.gyro.xyz)]
        accel_data2 = [sum(x) for x in zip(accel_data2, mpu2.accel.xyz)]
        gyro_data2 = [sum(x) for x in zip(gyro_data2, mpu2.gyro.xyz)]
        sleep(0.02)  # Adjust the delay as needed

        # Calculate the average
        accel_data1 = [x / num_readings for x in accel_data1]
        gyro_data1 = [x / num_readings for x in gyro_data1]
        accel_data2 = [x / num_readings for x in accel_data2]
        gyro_data2 = [x / num_readings for x in gyro_data2]

        # Apply a low-pass filter to accelerometer data
        filtered_accel1 = [0, 0, 0]
        filtered_accel2 = [0, 0, 0]

        low_pass_filter(filtered_accel1, accel_data1, alpha)
        low_pass_filter(filtered_accel2, accel_data2, alpha)

        # Calculate and print the yaw angle from gyro data
        yaw_acc1 = degrees(gyro_data1[2])
        yaw_acc2 = degrees(gyro_data2[2])
        yaw_measurement = (yaw_acc1 + yaw_acc2) / 2

        # Call the Kalman filter for yaw angle
        angle, bias, P = kalman_filter(yaw_measurement, angle, bias, P)

        # Print the final yaw angle (filtered with Kalman filter)
        print("Yaw (Degrees):", angle)

import time
from imu import MPU6050
from machine import I2C, Pin
from utime import sleep
from math import atan2, degrees, sqrt

# Define low-pass filter constants
alpha = 0.98  # Low-pass filter coefficient for accelerometer (adjust as needed)
dt = 0.02     # Time interval (adjust according to your sampling rate)

# Kalman filter class
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

# Define the low-pass filter function
def low_pass_filter(filtered, raw, alpha):
    for i in range(3):
        filtered[i] = alpha * raw[i] + (1.0 - alpha) * filtered[i]

# Create Kalman filters for accelerometer data
accel_x_filter1 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)
accel_y_filter1 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)
accel_z_filter1 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)
accel_x_filter2 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)
accel_y_filter2 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)
accel_z_filter2 = KalmanFilter(process_variance=0.01, measurement_variance=0.1, initial_estimate=0)

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

    # Update the Kalman filters with accelerometer data
    filtered_accel1 = [
        accel_x_filter1.update(filtered_accel1[0]),
        accel_y_filter1.update(filtered_accel1[1]),
        accel_z_filter1.update(filtered_accel1[2])
    ]

    filtered_accel2 = [
        accel_x_filter2.update(filtered_accel2[0]),
        accel_y_filter2.update(filtered_accel2[1]),
        accel_z_filter2.update(filtered_accel2[2])
    ]

    # Calculate the complementary filter orientation
    roll_acc1 = atan2(filtered_accel1[1], filtered_accel1[2])
    pitch_acc1 = atan2(-filtered_accel1[0], sqrt(filtered_accel1[1]**2 + filtered_accel1[2]**2))
    roll_acc2 = atan2(filtered_accel2[1], filtered_accel2[2])
    pitch_acc2 = atan2(-filtered_accel2[0], sqrt(filtered_accel2[1]**2 + filtered_accel2[2]**2))

    roll = roll_acc1 + roll_acc2
    pitch = pitch_acc1 + pitch_acc2

    # Print the final orientation and averaged values
    # Print the final orientation (Roll and Pitch) without words, only gaps
    print(degrees(roll), degrees(pitch))

    avg_accel_combined = [(accel_data1[i] + accel_data2[i]) / 2 for i in range(3)]
    #print("Average Accelerometer Data (Sensor 1 & 2):", avg_accel_combined)

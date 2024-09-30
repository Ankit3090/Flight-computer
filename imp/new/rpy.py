import time
from imu import MPU6050
from machine import I2C, Pin
from utime import sleep

# Create instances of MPU6050 for each sensor
i2c1 = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
i2c2 = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
mpu1 = MPU6050(i2c1)
mpu2 = MPU6050(i2c2)

# Number of readings to average
num_readings = 10

# Time step (adjust according to your sampling rate)
dt = 0.02

# Kalman filter variables for yaw
Q_angle = 0.02
Q_bias = 0.005
R_measure = 0.015

angle_yaw = 0.0
bias_yaw = 0.0
P_yaw = 0.0

# Kalman filter variables for roll
angle_roll = 0.0
bias_roll = 0.0
P_roll = 0.0

# Kalman filter variables for pitch
angle_pitch = 0.0
bias_pitch = 0.0
P_pitch = 0.0

# Low-pass filter coefficient for gyroscope (adjust as needed)
alpha = 0.98

# Kalman filter function
def kalman_filter(yaw_measurement, angle, bias, P):
    # Prediction
    rate = yaw_measurement - bias
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
    gyro_data1 = [0, 0, 0]  # Reset gyro_data1 to zero
    gyro_data2 = [0, 0, 0]  # Reset gyro_data2 to zero

    for _ in range(num_readings):
        gyro_data1 = [sum(x) for x in zip(gyro_data1, mpu1.gyro.xyz)]
        gyro_data2 = [sum(x) for x in zip(gyro_data2, mpu2.gyro.xyz)]
        sleep(0.02)  # Adjust the delay as needed

    # Calculate the average
    gyro_data1 = [x / num_readings for x in gyro_data1]
    gyro_data2 = [x / num_readings for x in gyro_data2]

    # Apply a low-pass filter to the gyroscope data
    low_pass_gyro1 = [0, 0, 0]
    low_pass_gyro2 = [0, 0, 0]

    for i in range(3):
        low_pass_gyro1[i] = alpha * gyro_data1[i] + (1.0 - alpha) * low_pass_gyro1[i]
        low_pass_gyro2[i] = alpha * gyro_data2[i] + (1.0 - alpha) * low_pass_gyro2[i]

    # Calculate roll, pitch, and yaw angles based on integrated gyro data for each sensor
    roll_sensor1 = low_pass_gyro1[0] 
    roll_sensor2 = low_pass_gyro2[0]
    roll_measurement = (roll_sensor1 + roll_sensor2) / 2

    pitch_sensor1 = low_pass_gyro1[1]
    pitch_sensor2 = low_pass_gyro2[1]
    pitch_measurement = (pitch_sensor1 + pitch_sensor2) / 2

    yaw_acc1 = low_pass_gyro1[2]
    yaw_acc2 = low_pass_gyro2[2]

    yaw_measurement = (yaw_acc1 + yaw_acc2) / 2
    angle_yaw, bias_yaw, P_yaw = kalman_filter(yaw_measurement, angle_yaw, bias_yaw, P_yaw)
    
    angle_roll, bias_roll, P_roll = kalman_filter(roll_measurement, angle_roll, bias_roll, P_roll)
    angle_pitch, bias_pitch, P_pitch = kalman_filter(pitch_measurement, angle_pitch, bias_pitch, P_pitch)
    
    print("Roll (Avg): {:.2f}".format(angle_roll+1.15), "Pitch (Avg): {:.2f}".format(angle_pitch+1.74), "Yaw: {:.2f}".format(angle_yaw + 0.72))

    sleep(0.02)  # Add a delay if needed to control the print rate

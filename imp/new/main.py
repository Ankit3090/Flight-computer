from new2 import *

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

        # Calculate the complementary filter orientation for roll and pitch
        roll_acc1 = atan2(filtered_accel1[1], filtered_accel1[2])
        pitch_acc1 = atan2(-filtered_accel1[0], sqrt(filtered_accel1[1]**2 + filtered_accel1[2]**2))
        roll_acc2 = atan2(filtered_accel2[1], filtered_accel2[2])
        pitch_acc2 = atan2(-filtered_accel2[0], sqrt(filtered_accel2[1]**2 + filtered_accel2[2]**2))

        roll = roll_acc1 + roll_acc2
        pitch = pitch_acc1 + pitch_acc2

        yaw_acc1 = degrees(gyro_data1[2])
        yaw_acc2 = degrees(gyro_data2[2])
        yaw_measurement = (yaw_acc1 + yaw_acc2) / 2
        angle, bias, P = kalman_filter(yaw_measurement, angle, bias, P)
        temperature = bmp280_object.temperature
        pressure_hPa = bmp280_object.pressure / 100.0
        h = altitude_HYP(pressure_hPa, temperature)
        altitude = altitude_IBF(pressure_hPa)

        print(degrees(roll),",",degrees(pitch),",",angle,",",h)
        avg_accel_combined_x = (filtered_accel1[0] + filtered_accel2[0]) / 2
        avg_accel_combined_y = (filtered_accel1[1] + filtered_accel2[1]) / 2
        avg_accel_combined_z = (filtered_accel1[2] + filtered_accel2[2]) / 2
    #print("Avg Accelerometer Data (X): {:.2f}, (Y): {:.2f}, (Z): {:.2f}".format(avg_accel_combined_x, avg_accel_combined_y, avg_accel_combined_z))

        

    #print("Temperature: {:2f} C".format(temperature))
    
    #print("Pressure: {:2f} hPa".format(pressure_hPa))
    #print("Altitude (HYP): {:2f} meters".format(h))
        
    #print(altitude)
        #print("\n")


    




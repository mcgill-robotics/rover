import pyrealsense2 as rs
import rospy
from realsense_imu.msg import Realsense_imu_data, Realsense_computed_angle
from time import sleep
import math
from std_msgs.msg import Float32MultiArray

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
profile = pipeline.start(config)

rospy.init_node('realsense_imu', anonymous=False)
real_sense_publisher = rospy.Publisher('realsense_imu_data', Realsense_imu_data, queue_size=1)
direction_publisher = rospy.Publisher('realsense_direction', Realsense_computed_angle, queue_size=1)

first_frame = True
alpha = 0.98
totalgyroangleY = 0

try:
    while True:
        f = pipeline.wait_for_frames()

        accel = f[0].as_motion_frame().get_motion_data()
        gyro = f[1].as_motion_frame().get_motion_data()

        ts = f.get_timestamp()

        if first_frame:
            first_frame = False
            last_ts = ts

            # accelerometer calculation
            accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
            accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
            accel_angle_y = math.degrees(math.pi)

            continue
        
        else:
            dt_gyro = (ts - last_ts) / 1000
            last_ts = ts

            gyro_angle_x = gyro.x * dt_gyro
            gyro_angle_y = gyro.y * dt_gyro
            gyro_angle_z = gyro.z * dt_gyro

            dangleX = gyro_angle_x * 57.2958
            dangleY = gyro_angle_y * 57.2958
            dangleZ = gyro_angle_z * 57.2958

            totalgyroangleX = accel_angle_x + dangleX
            # totalgyroangleY = accel_angle_y + dangleY
            totalgyroangleY = accel_angle_y + dangleY + totalgyroangleY
            totalgyroangleZ = accel_angle_z + dangleZ

            #accelerometer calculation
            accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
            accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
            # accel_angle_y = math.degrees(math.pi)
            accel_angle_y = 0

            #combining gyrometer and accelerometer angles
            combinedangleX = totalgyroangleX * alpha + accel_angle_x * (1-alpha)
            combinedangleZ = totalgyroangleZ * alpha + accel_angle_z * (1-alpha)
            combinedangleY = totalgyroangleY
            # Z = 0
            # Y = 180
            # X = -90
            #print("Angle -  Z: " + str(round(combinedangleX,2)) + "   Y: " + str(round(combinedangleY,2)) + "   X: " + str(round(combinedangleZ,2)))
            
            imu_data = Realsense_imu_data()
            imu_data.accelorometer = [accel_angle_x, accel_angle_y, accel_angle_z]
            imu_data.gyroscope = [round(gyro_angle_x, 3), round(gyro_angle_y, 3), round(gyro_angle_z, 3)]
            real_sense_publisher.publish(imu_data)

            direction = Realsense_computed_angle(angles=[combinedangleX, combinedangleY, combinedangleZ])
            direction_publisher.publish(direction)

            sleep(0.05)
finally:
    pipeline.stop()
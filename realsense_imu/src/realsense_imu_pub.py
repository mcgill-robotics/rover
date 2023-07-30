import pyrealsense2 as rs
import rospy
from realsense_imu.msg import Realsense_imu_data

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
profile = pipeline.start(config)

rospy.init_node('realsense_imu', anonymous=False)
real_sense_publisher = rospy.Publisher('realsense_imu_data', Realsense_imu_data, queue_size=1)

try:
    while True:
        frames = pipeline.wait_for_frames()

        motion_frame = frames.first_or_default(rs.stream.accel)

        if motion_frame:
            accel_data = motion_frame.as_motion_frame().get_motion_data()

            motion_frame = frames.first_or_default(rs.stream.gyro)
            gyro_data = motion_frame.as_motion_frame().get_motion_data()

            imu_data = Realsense_imu_data()
            imu_data.accelorometer = [accel_data.x, accel_data.y, accel_data.z]
            imu_data.gyroscope = [gyro_data.x, gyro_data.y, gyro_data.z]

            real_sense_publisher.publish(imu_data)
finally:
    pipeline.stop()
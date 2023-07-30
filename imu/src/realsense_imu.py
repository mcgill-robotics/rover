import pyrealsense2 as rs
import cv2
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)

profile = pipeline.start(config)
try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()

        # Get the color frame
        color_frame = frames.get_color_frame()

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Display the color frame
        cv2.imshow('Intel RealSense Camera', color_image)

        motion_frame = frames.first_or_default(rs.stream.accel)

        if motion_frame:
            accel_data = motion_frame.as_motion_frame().get_motion_data()

            motion_frame = frames.first_or_default(rs.stream.gyro)
            gyro_data = motion_frame.as_motion_frame().get_motion_data()

            print("Accelerometer Data (m/s^2): x={:.3f}, y={:.3f}, z={:.3f}".format(
                accel_data.x, accel_data.y, accel_data.z))
            print("Gyroscope Data (rad/s): x={:.3f}, y={:.3f}, z={:.3f}\n".format(
                gyro_data.x, gyro_data.y, gyro_data.z))

        # Exit the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
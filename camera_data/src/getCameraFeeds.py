import rospy
from std_msgs.msg import String
import cv2
from camera_config import CAMERA_CONFIG  # Import the camera configuration

class CameraHandler:
    def __init__(self):
        self.camera_names = list(CAMERA_CONFIG.keys())
        self.available_cameras = []

        # Check if cameras are available and open them
        for name, path in CAMERA_CONFIG.items():
            newCap = cv2.VideoCapture(path)
            if newCap.isOpened():
                self.available_cameras.append(name)  # Store the camera name
                newCap.release()  # Release after checking

    def publish_camera_names(self):
        rospy.init_node('camera_name_publisher')
        pub = rospy.Publisher('/available_cameras', String, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            cameras_str = ",".join(self.available_cameras)  # Join the camera names into a comma-separated string
            pub.publish(cameras_str)
            rate.sleep()

if __name__ == '__main__':
    camHandler = CameraHandler()
    camHandler.publish_camera_names()
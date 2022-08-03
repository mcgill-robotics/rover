import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Node_CameraFrameSub():

    def __init__(self):
        self.frames = None 
        rospy.init_node('camera_frame_subscriber')
        self.camera_frame_subscriber = rospy.Subscriber('/camera_frames', Image, self.display_frames)
    

    def display_frames(self, frame):
        self.frames = self.ros_to_openCV_image(frame)
        self.frames = cv2.imdecode(self.frames, 1)
        cv2.imshow("Live Feed", self.frames)
        cv2.waitKey(10)  # Look into to reduce latency

    def ros_to_openCV_image(self, ros_image):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')
        return cv_image

    

if __name__ == "__main__":
    driver = Node_CameraFrameSub()
    rospy.spin()
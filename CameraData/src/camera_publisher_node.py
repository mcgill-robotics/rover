import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Node_CameraFramePub():

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.frames = None

        rospy.init_node('camera_frame_publisher')
        self.camera_frame_publisher = rospy.Publisher('/camera_frames', Image, queue_size=1)
        self.run()

    def run(self):
        while self.video_capture is not None and self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame = cv2.imencode(".jpg", frame, encode_params)
            self.frames = self.openCV_to_ros_image(frame)
            if ret:
                self.camera_frame_publisher.publish(self.frames)
            time.sleep(0.1) # look into to reduce latency

    def openCV_to_ros_image(self, cv_image):
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        return ros_image

    

if __name__ == "__main__":
    driver = Node_CameraFramePub()
    rospy.spin()
#!/usr/bin/env python3
import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import sys

stop = False

class Node_CameraFramePub():

    def __init__(self):
        self.frames = None
        self.video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.video_capture.open(0)

        rospy.init_node('camera_frame_publisher')
        self.camera_select_subscriber = rospy.Subscriber("camera_selection", Int16, self.select_camera)
        _30fps_ns = 30000000
        self.timer = rospy.Timer(rospy.Duration(0, _30fps_ns), self.timer_callback)
        self.camera_frame_publisher = rospy.Publisher('/camera_frames', Image, queue_size=10)

    def timer_callback(self, _):
        ret, frame = self.video_capture.read()
        if ret:
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 90]
            _, frame = cv2.imencode(".jpg", frame, encode_params)
            self.frames = self.openCV_to_ros_image(frame)
            print("Publishing frame")
            self.camera_frame_publisher.publish(self.frames)
        else: 
            print("No camera found")

    def openCV_to_ros_image(self, cv_image):
        try:
            bridge = CvBridge()
            ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
            return ros_image
        except:
            print("Cannot translate image correctly")

    def select_camera(self, x):
        self.video_capture.release()
        self.video_capture.open(x.data * 2)
        print(x.data)


if __name__ == "__main__":
    driver = Node_CameraFramePub()
    rospy.spin()

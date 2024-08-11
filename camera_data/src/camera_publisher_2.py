#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import base64
import cv2
from camera_config import CAMERA_CONFIG  # Import the camera configuration

class Node_CameraFramePublishers():
    def __init__(self, camera_name):
        self.camera_name = camera_name
        self.camera_path = CAMERA_CONFIG[self.camera_name]  # Get the udev path from the config
        rospy.init_node(f'camera_frame_publisher_{self.camera_name.replace(" ", "_")}')
        self.sub = rospy.Subscriber(f'camera_selection_{self.camera_name.replace(" ", "_")}', String, self.selection_callback)
        self.webcam_publisher()

    def webcam_publisher(self):
        self.pub1 = rospy.Publisher(f'camera_frames_{self.camera_name.replace(" ", "_")}', String, queue_size=10)
        rate = rospy.Rate(30)  # 30hz
        self.cap = cv2.VideoCapture(self.camera_path)  # Open the webcam by udev path

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                frame_resized = cv2.resize(frame, (640, 480))
                _, encoded_image = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                base64_image = base64.b64encode(encoded_image).decode('utf-8')
                self.pub1.publish(base64_image)
            rate.sleep()

    def selection_callback(self, msg):
        self.cap.release()
        self.camera_path = CAMERA_CONFIG.get(msg.data, self.camera_path)  # Update the camera path based on selection
        self.cap = cv2.VideoCapture(self.camera_path)  # Open the new camera by udev path

if __name__ == '__main__':
    try:
        Node_CameraFramePublishers("Pan Tilt Camera")  # Replace with actual camera name
    except rospy.ROSInterruptException:
        pass
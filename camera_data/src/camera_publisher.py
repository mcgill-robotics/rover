#!/usr/bin/env python3
import os, sys
from pydoc_data.topics import topics

currentdir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(currentdir)
import rospy
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from PyQt5 import QtGui
import sys
import numpy as np

stop = False


class Node_CameraFramePub():

    def __init__(self):
        self.frames = None
        self.video_capture0 = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.video_capture0.open(0)
        self.video_capture1 = cv2.VideoCapture(1, cv2.CAP_V4L2)
        self.video_capture1.open(1)
        self.video_capture2 = cv2.VideoCapture(2, cv2.CAP_V4L2)
        self.video_capture2.open(2)
        self.video_capture3 = cv2.VideoCapture(3, cv2.CAP_V4L2)
        self.video_capture3.open(3)
        self.video_capture4 = cv2.VideoCapture(4, cv2.CAP_V4L2)
        self.video_capture4.open(4)
        self.video_capture5 = cv2.VideoCapture(5, cv2.CAP_V4L2)
        self.video_capture5.open(5)

        rospy.init_node('camera_frame_publisher')
        self.camera_select_subscriber = rospy.Subscriber("camera_selection", Int16, self.select_camera)
        self.timer = rospy.Timer(rospy.Duration(0.03), self.timer_callback)
        self.camera_frame_publisher = rospy.Publisher('/camera_frames', Image, queue_size=10)

    def timer_callback(self, event):
        try:
            ret0, frame0 = self.video_capture0.read()
            ret1, frame1 = self.video_capture1.read()
            ret2, frame2 = self.video_capture2.read()
            ret3, frame3 = self.video_capture3.read()
            ret4, frame4 = self.video_capture4.read()
            ret5, frame5 = self.video_capture5.read()
            frames = []
            ret_values = [ret0, ret1, ret2, ret3, ret4, ret5]
            frame_values = [frame0, frame1, frame2, frame3, frame4, frame5]
            for i in range(6):
                if ret_values[i]:
                    frames.append(frame_values[i])
        except:
            pass
        have_picture_cameras = len(frames)
        if 0 < have_picture_cameras <= 3:
            empty_picture_num = 3 - have_picture_cameras
            empty_picture = np.zeros((1000, 292), dtype=np.uint8)
            # resize frames to connect
            frames_resized = []
            for i in range(len(frames)):
                frames_resized.append(cv2.resize(frames[i], (1000, 292)))
            # connect pictures
            frames_resized = tuple(frames_resized)
            image = np.vstack(frames)
            # connect empty pictures to makesure 3 pictures connected
            for i in range(empty_picture_num):
                image = np.vstack((image, empty_picture))
            # convert to ros format and publish
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame = cv2.imencode(".jpg", image, encode_params)
            self.frames = self.openCV_to_ros_image(frame)
            self.camera_frame_publisher.publish(self.frames)

        elif 6 > len(frame_values) > 3:
            empty_picture_num = 6 - have_picture_cameras
            empty_picture = np.zeros((500, 292), dtype=np.uint8)
            # resize frames to connect
            frames_resized = []
            for i in range(len(frames)):
                frames_resized.append(cv2.resize(frames[i], (500, 292)))
            frames = tuple(frames)
            # connect pictures
            image_line1 = np.hstack(frames[0, 1])
            image_line_2 = np.hstack(frames[2, 3])
            if empty_picture_num == 1:
                image_line_3 = np.hstack((frames[4],empty_picture))
            elif empty_picture_num == 2:
                image_line_3 = np.hstack((empty_picture, empty_picture))
            # connect empty pictures to makesure 3 pictures connected
            image = np.vstack(image_line1,image_line_2,image_line_3)
            # convert to ros format and publish
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame = cv2.imencode(".jpg", image, encode_params)
            self.frames = self.openCV_to_ros_image(frame)
            self.camera_frame_publisher.publish(self.frames)

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


# if __name__ == "__main__":
#     driver = Node_CameraFramePub()
#     rospy.spin()
if __name__ == "__main__":
    video_capture0 = cv2.VideoCapture(0, cv2.CAP_V4L2)
    video_capture0.open(0)
    video_capture1 = cv2.VideoCapture(1, cv2.CAP_V4L2)
    video_capture1.open(1)
    video_capture2 = cv2.VideoCapture(2, cv2.CAP_V4L2)
    video_capture2.open(2)
    video_capture3 = cv2.VideoCapture(3, cv2.CAP_V4L2)
    video_capture3.open(3)
    video_capture4 = cv2.VideoCapture(4, cv2.CAP_V4L2)
    video_capture4.open(4)
    video_capture5 = cv2.VideoCapture(5, cv2.CAP_V4L2)
    video_capture5.open(5)

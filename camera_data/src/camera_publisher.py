#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Node_CameraFramePublishers():
    def __init__(self):
        rospy.init_node('camera_frame_publisher')
        self.sub = rospy.Subscriber('camera_selection', Int16, self.selection_callback)
        self.webcam_publisher()

    def webcam_publisher(self):
        self.pub = rospy.Publisher('camera_frames', Image, queue_size=10)
        rate = rospy.Rate(20) # 20hz

        self.cap = cv2.VideoCapture(0) # Open the webcam
        bridge = CvBridge()

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                # Resize the frame to 640x480
                frame_resized = cv2.resize(frame, (640, 480))
                # Convert the resized frame to ROS format
                ros_image = bridge.cv2_to_imgmsg(frame_resized, "bgr8")
                # Publish the resized frame
                self.pub.publish(ros_image)
            rate.sleep()

    def selection_callback(self, msg):
        # when received a int i, close the current stream and open the new stream i
        self.cap.release()
        self.cap = cv2.VideoCapture(msg.data)

if __name__ == '__main__':
    try:
        Node_CameraFramePublishers()
    except rospy.ROSInterruptException:
        pass

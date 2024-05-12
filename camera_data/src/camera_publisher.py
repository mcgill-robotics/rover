#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def webcam_publisher():
    rospy.init_node('webcam_publisher', anonymous=True)
    pub = rospy.Publisher('camera_frames', Image, queue_size=10)
    rate = rospy.Rate(30) # 10hz

    cap = cv2.VideoCapture(0) # Open the first webcam
    bridge = CvBridge()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Convert the frame to ROS format
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the frame
            pub.publish(ros_image)
        rate.sleep()

if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass

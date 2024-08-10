#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, String
from cv_bridge import CvBridge
import base64
import cv2

class Node_CameraFramePublishers():
    def __init__(self):
        self.camera_idx = 1
        rospy.init_node(f'camera_frame_publisher_{self.camera_idx}')
        self.sub = rospy.Subscriber(f'camera_selection_{self.camera_idx}', Int16, self.selection_callback)
        self.webcam_publisher()

    def webcam_publisher(self):
        # self.pub = rospy.Publisher('camera_frames', Image, queue_size=10)
        self.pub1 = rospy.Publisher(f'camera_frames_{self.camera_idx}', String, queue_size=10)
        rate = rospy.Rate(30) # 20hz

        self.cap = cv2.VideoCapture(4) # Open the webcam
        bridge = CvBridge()

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                frame_resized = cv2.resize(frame, (640, 480))
                # Convert the frame to JPEG format and compress
                _, encoded_image = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                # Convert to base64
                base64_image = base64.b64encode(encoded_image).decode('utf-8')
                # Publish as a string
                self.pub1.publish(base64_image)
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

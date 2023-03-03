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

stop = False

class Node_CameraFramePub():

    def __init__(self):
        self.frames = None
        self.video_capture = cv2.VideoCapture(0, cv2.CAP_V4L2)

        rospy.init_node('camera_frame_publisher')
        self.camera_select_subscriber = rospy.Subscriber("camera_selection", Int16, self.select_camera)
        self.camera_frame_publisher = rospy.Publisher('/camera_frames', Image, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def timer_callback(self, event):
        print("Publishing frame")
        try:
            ret, frame = self.video_capture.read()   
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, frame = cv2.imencode(".jpg", frame, encode_params)
            self.frames = self.openCV_to_ros_image(frame)
            if ret:
                self.camera_frame_publisher.publish(self.frames)
        except:
            print("No camera found")
        finally:
            pass
        

    def openCV_to_ros_image(self, cv_image):
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        return ros_image
    

    def select_camera(self, x):
        self.video_capture = cv2.VideoCapture(x.data, cv2.CAP_V4L2)
        print(x.data)






if __name__ == "__main__":
    driver = Node_CameraFramePub()
    rospy.spin()
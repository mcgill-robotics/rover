#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import base64

class ArucoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_detector', anonymous=True)


        # Load the ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Initialize OpenCV Bridge
        self.bridge = CvBridge()

        self.camera_id = 1
        # Subscribers and Publishers
        self.start_sub = rospy.Subscriber('/aruco_start_detection', String, self.start_detection_callback)
        #self.result_pub = rospy.Publisher('/aruco_detection_result', String, queue_size=10)
        self.pub = rospy.Publisher(f'camera_frames_{self.camera_id+1}', String, queue_size=10)
        

        # Flag to control detection
        #self.detecting = False

    def start_detection_callback(self, msg):
        # if msg.data.lower() == 'start':
        #     self.detecting = True
        #     rospy.loginfo("ArUco detection started.")
        # elif msg.data.lower() == 'stop':
        #     self.detecting = False
        #     rospy.loginfo("ArUco detection stopped.")

        # if not self.detecting:
        #     return
        
        # else:
        cap = cv2.VideoCapture(self.camera_id)

        if not cap.isOpened():
            print("Error: Could not open video capture.")
            return

        
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)


        if ids is not None:
            frame_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))  # Green border

            # Draw the marker IDs as text
            for i, marker_id in enumerate(ids.flatten()):
                # Calculate text position
                x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                text = f"ID: {marker_id}"
                
                # Put text on the frame with red color
                cv2.putText(frame_markers, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            frame_markers = frame

        
        rate = rospy.Rate(30) # 20hz

        frame_resized = cv2.resize(frame_markers, (640, 480))
        # Convert the frame to JPEG format and compress
        _, encoded_image = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        # Convert to base64
        base64_image = base64.b64encode(encoded_image).decode('utf-8')
        # Publish as a string
        self.pub.publish(base64_image)
        rate.sleep()

    

if __name__ == '__main__':
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

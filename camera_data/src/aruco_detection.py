#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge
import base64
from camera_config import CAMERA_CONFIG  # Import the camera configuration
import cv2
import numpy as np

class arucopublisher():
    def __init__(self, camera_indes):
        rospy.init_node(f'aruco_publisher')
        self.sub = rospy.Subscriber(f'camera_selection_{self.camera_name.replace(" ", "_")}', String, self.selection_callback)
        self.webcam_publisher()


camera_index = 0

def main():
    # Load the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters_create()

    # Open the video capture (0 for the default camera, or provide a video file path)
    # ui sub topic
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open video capture.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Convert frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Draw detected markers
        frame_markers = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
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

        # Display the frame with markers
        cv2.imshow('ArUco Marker Detection', frame_markers)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

pub = rospy.Publisher(f'camera_frames_{camera_index}', String, queue_size=10)


if __name__ == "__main__":
    main()

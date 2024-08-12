import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from cv2 import aruco
import base64


class Node_ArucoDetector():

    def __init__(self):
        rospy.init_node('aruco_detector')
        self.pub1 = rospy.Publisher(f'camera_frames_1', String, queue_size=10)
        self.main()


    def main(self):
        # Import aruco after cv2
        rate = rospy.Rate(30)

        # Load the ArUco dictionary and parameters
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        parameters = aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        # Open the video capture (0 for the default camera, or provide a video file path)
        cap = cv2.VideoCapture(1)

        if not cap.isOpened():
            print("Error: Could not open video capture.")
            return

        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Detect ArUco markers
            corners, ids, rejectedImgPoints = detector.detectMarkers(frame)

            # Draw detected markers
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

            frame_resized = cv2.resize(frame, (640, 480))
            # Convert the frame to JPEG format and compress
            _, encoded_image = cv2.imencode('.jpg', frame_resized, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            # Convert to base64
            base64_image = base64.b64encode(encoded_image).decode('utf-8')
            # Publish as a string
            self.pub1.publish(base64_image)
            rate.sleep()

        # Release the capture and close windows
        cap.release()


if __name__ == "__main__":
    Node_ArucoDetector()

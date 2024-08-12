import cv2
import numpy as np

def main():
    # Load the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters_create()

    # Open the video capture (0 for the default camera, or provide a video file path)
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

if __name__ == "__main__":
    main()

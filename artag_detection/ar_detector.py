#!/usr/bin/env python3

import cv2
import rospy
from cv2 import aruco
import glob
import numpy as np

# define names of each possible ArUco tag OpenCV supports (just for personal reference)
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

webcam = cv2.VideoCapture(0) # captures webcam feed (or whatever camera we end up using)

global cameraMatrix # creates variables for camera matrix and distortion coefficients
global distCoeffs

# Using a ChAruco Board to calibrate my webcam; will have to take images from diff. viewpoints using whatever cam we're using
def camera_calibration():
	chAruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_4X4_50"])
	chAruco_board = cv2.aruco.CharucoBoard_create(squaresX = 7, squaresY = 9, squareLength = 0.024, markerLength = 0.019,
												 dictionary = chAruco_dict)
	chAruco_params = cv2.aruco.DetectorParameters_create()

	# stores info for board + image's size which is determined later
	board_corners = []
	board_ids = []
	image_size = None

	boardPics = glob.glob('**/board-pic*.jpg') # obtain set of photos

	for img in boardPics:
		board_pic = cv2.imread(img)
		greyscale = cv2.cvtColor(board_pic, cv2.COLOR_BGR2GRAY)

		(corners, ids, rejects) = cv2.aruco.detectMarkers(greyscale, chAruco_dict, parameters = chAruco_params)

		response, cHcorners, cHids = cv2.aruco.interpolateCornersCharuco(markerCorners = corners, markerIds = ids, image = greyscale, board = chAruco_board)
		
		if response > 20:
			board_corners.append(cHcorners)
			board_ids.append(cHids)

			if not image_size:
				image_size = greyscale.shape[::-1]

	# Calls method to calibrate, and assign variables for camera and distortion coefficient matrices
	global cameraMatrix, distCoeffs

	retval, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
        charucoCorners = board_corners,
        charucoIds=board_ids,
        board=chAruco_board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None)

# Function for estimating the pose of aruco marker	
def estimateTagPose():
	camera_calibration() # calibrates camera and receive camera and distortion coeff. matrices
	while not rospy.is_shutdown():
		ret, newFrame = webcam.read() # reads current frame and saves it if available

		# continuously loop through keys and values to find the matching dictionary
		for (arType, arDict) in ARUCO_DICT.items():
			arDict = cv2.aruco.Dictionary_get(arDict) # obtain dictionary and create parameters for detection
			arParams = cv2.aruco.DetectorParameters_create()

			(corners, ids, rejects) = cv2.aruco.detectMarkers(newFrame, arDict, parameters = arParams) # corners: x-y coords; ids: int values

			if len(corners) > 0: # at least one marker was detected
				ids = ids.flatten() # puts ids in the same list (1d array)
				cv2.aruco.drawDetectedMarkers(newFrame, corners) # draws box of detection and top left dot

				rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, cameraMatrix, distCoeffs) # obtain pose and assign to rotation vector (rvec) and translation vector (tvec)
				newFrame = cv2.aruco.drawAxis(newFrame, cameraMatrix, distCoeffs, rvec, tvec, 0.02) # draws axes
				
				print(type(rvec))

		cv2.imshow('AR DETECTOR', newFrame)

		if cv2.waitKey(1) & 0xFF == ord('q'): # when 'q' is pressed, exit loop and close window
			break

	webcam.release()
	cv2.destroyAllWindows()

estimateTagPose()


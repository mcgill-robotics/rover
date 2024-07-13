import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class aruco_det:

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

    def __init__(self, aruco_type, intrinsic_camera, distortion):
        self.aruco_dict_type = self.ARUCO_DICT[aruco_type]
        self.intrinsic_camera = intrinsic_camera
        self.distortion = distortion
        self.bridge = CvBridge()
        rospy.init_node('aruco_autonomy', anonymous=True)
        self.aruco_sub = rospy.Subscriber("/camera/image_raw", Image, self.pose_estimation)
        self.aruco_pub = rospy.Publisher('/camera/aruco_det', Image, queue_size=10)
        rospy.spin()

    def aruco_display(self, corners, ids, rejected, image):    
        if len(corners) > 0:        
            ids = ids.flatten()     
            for (markerCorner, markerID) in zip(corners, ids):          
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                
                cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)
                print("[Inference] ArUco marker ID: {}".format(markerID))           
        return image

    def pose_estimation(self, msg): #  If poseless, do not try to estimate pose, simply detect
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(self.aruco_dict_type)
        parameters = cv2.aruco.DetectorParameters()
        parameters.minDistanceToBorder = 0
        refineParams = cv2.aruco.RefineParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters, refineParams)
        corners, ids, rejected_img_points = detector.detectMarkers(gray)
        if len(corners) > 0:
            for i in (0, len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, self.intrinsic_camera, self.distortion)
                cv2.aruco.drawDetectedMarkers(frame, corners) 
                cv2.drawFrameAxes(frame, self.intrinsic_camera, self.distortion, rvec, tvec, 0.01)  
        elif len(rejected_img_points[0]) > 0:
            cv2.aruco.drawDetectedMarkers(frame, [rejected_img_points[0]], borderColor=(0, 0, 255))

        self.aruco_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__': 
    # detector = aruco_det(Dictionary, intrinsic parameters of ZED camera, extrinsic parameters of ZED camera)
    detector = aruco_det("DICT_5X5_1000", 
        np.array(((933.15867, 0, 648.5),(0,933.1586, 366.8340148925781),(0,0,1))), 
        np.array((-0.043693598,0.0146164996,-0.006573319,-0.000216900)))

from aruco_detect import aruco_det
import numpy as np
import cv2
import os

class aruco_testing:
    def __init__(self):
        pass

if __name__ == '__main__': 
    # detector = aruco_det(Dictionary, intrinsic parameters of ZED camera, extrinsic parameters of ZED camera)
    detector = aruco_det("DICT_5X5_1000", np.array(((933.15867, 0, 648.5),(0,933.1586, 366.8340148925781),(0,0,1))), np.array((-0.043693598,0.0146164996,-0.006573319,-0.000216900)))
    path = "/home/styops/Pictures"
    imName= "test1.png"
    img_path = os.path.join(path, imName)
    img = cv2.imread(img_path)
    output = detector.pose_estimation(img, False)
    cv2.imshow('img',img)
    cv2.waitKey(0)
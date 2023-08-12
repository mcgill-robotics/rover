import cv2
import numpy as np
import matplotlib.pyplot as plt
import imutils

x_thres = 10
y_thres = 10

image1 = "/home/shaswata/Desktop/test/img1.jpg"
image2 = "/home/shaswata/Desktop/test/img2.jpg"
image3 = "/home/shaswata/Desktop/test/img3.jpg"

image1 = cv2.imread(image1)
image2 = cv2.imread(image2)
image3 = cv2.imread(image3)
images = [image1, image2, image3]


stitcher = cv2.Stitcher_create()
retval, pano = stitcher.stitch(images)
y, x, _ = pano.shape

cropped_image = pano[int(0.01*y_thres*y):y-int(0.01*y_thres*y), int(0.01*x_thres*x):x-int(0.01*x_thres*x)]
cv2.imwrite("/home/shaswata/Desktop/test/out.jpg", cropped_image)
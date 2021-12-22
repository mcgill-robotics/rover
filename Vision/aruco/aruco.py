import cv2
import cv2.aruco as aruco

VideoCap=True
cap=cv2.VideoCapture(0)

def rescaleFrame(frame, scale=0.75):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)

    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

def findAruco(img, marker_size=6, total_markers=250, draw=True):
    gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    key=getattr(aruco,f'DICT_{marker_size}X{marker_size}_{total_markers}')
    arucoDict=aruco.Dictionary_get(key)
    arucoParam=aruco.DetectorParameters_create()
    bbox,ids,_=aruco.detectMarkers(gray,arucoDict, parameters=arucoParam)
    if draw:
        aruco.drawDetectedMarkers(img, bbox)
    return bbox, ids


while True:
    if VideoCap: 
        _,img=cap.read()
    else:
        img=cv2.imread("pic2.jpg")
        img = rescaleFrame(img, scale=.4)
    bbox,ids = findAruco(img)
    print(bbox)
    if cv2.waitKey(1)==45:
        break
    cv2.imshow('Image', img)
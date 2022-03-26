import cv2
import sys
import os

# gets location of getCameraFeeds to be recognized as a module
dir = os.path.dirname(__file__)
filename_camFeed = os.path.join(dir, 'camera_feed_view')
sys.path.insert(0,filename_camFeed)

# adds set of aruco functions to file
filename_aruco = os.path.join(dir, 'aruco')
sys.path.insert(0,filename_aruco)

import getCameraFeeds as gcf
import aruco as ar

# Main aruco finder loop
if __name__ == '__main__':
    camHandler = gcf.CameraHandler()
    while True:
        retAndFrame = camHandler.get_all_feeds()
        # display each frame (camera feed) in the list
        for ret, frame, index in retAndFrame:
            if ret:
                cv2.imshow(f"frame {index}", frame)

        # detect aruco tags in each camera
        for cap in camHandler.vids:
            _,img=cap.read()
            bbox,ids = ar.findAruco(img)
            print(bbox)

        # press 'q' to stop displaying
        if cv2.waitKey(1) & 0xFF == ord('q'):
            for vid in camHandler.vids:
                vid.release()
                cv2.destroyAllWindows()
            break
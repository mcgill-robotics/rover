import cv2
import sys
import os

# gets location of getCameraFeeds to be recognized as a module
dir = os.path.dirname(__file__)
filename = os.path.join(dir, '../src')
sys.path.insert(0,filename)

import getCameraFeeds as gcf

# Runnable for displaying camera feeds
camHandler = gcf.CameraHandler()
while True:
    retAndFrame = camHandler.get_all_feeds()
    # display each frame (camera feed) in the list
    for ret, frame, index in retAndFrame:
        cv2.imshow(f"frame {index}", frame)

    # press 'q' to stop displaying
    if cv2.waitKey(1) & 0xFF == ord('q'):
        for vid in camHandler.vids:
            vid.release()
            cv2.destroyAllWindows()
        break


import cv2


class CameraHandler:
    vids = []  # available cameras

    def __init__(self):
        # Loop 10 times and collect all found cameras
        # Assumes sequential indices
        for i in range (10):
            newCap = cv2.VideoCapture(i)

            # check if object is not
            if newCap is not None and newCap.isOpened():
                # add the camera found
                self.vids.append(newCap)

    def get_all_feeds(self):
        retAndFrame = []
        # read each capture object in vids and add as a tuple to retAndFrame
        for i in range(len(self.vids)):
            newFeed = self.vids[i].read()
            retAndFrame.append((newFeed[0], newFeed[1], i))
        
        return retAndFrame

# # Runnable for displaying camera feeds
if __name__ == '__main__':
    camHandler = CameraHandler()
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

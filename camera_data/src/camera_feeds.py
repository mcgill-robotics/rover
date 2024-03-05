import cv2


class CameraHandler:
    available_cameras = []

    def __init__(self):
        # Collect all found cameras
        # Assumes sequential indices
        for i in range (25):
            newCap = cv2.VideoCapture(i, cv2.CAP_V4L2)

            if newCap is not None and newCap.open(i):
                self.available_cameras.append(newCap)

    def get_all_feeds(self):
        ret_frame = []
        # read each capture object in vids and add as a tuple to retAndFrame
        for camera in self.available_cameras:
            newFeed = camera.read()
            ret_frame.append((newFeed[0], newFeed[1]))
        
        return ret_frame

    def run_feeds(self):
        while True:
            # display each frame (camera feed) in the list
            for index, (ret, frame) in enumerate(self.get_all_feeds()):
                if ret:
                    cv2.imshow(f"frame {index}", frame)

            # press 'q' to stop displaying
            if cv2.waitKey(1) & 0xFF == ord('q'):
                for vid in self.available_cameras:
                    vid.release()
                    cv2.destroyAllWindows()
                break

# Runnable to display camera feeds
if __name__ == '__main__':
    camera_handler = CameraHandler()
    camera_handler.run_feeds()

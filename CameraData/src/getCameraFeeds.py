import cv2


class CameraFeed:
    vids = []  # available cameras

    def __init__(self):
        # counter for init method
        index = 0

        # Loop until invalid cam is found
        # Maybe not the most efficient way; assumes sequential indices
        while True:
            newCap = cv2.VideoCapture(index)

            # Check if camera object created is not null or unable to be opened
            if newCap.isOpened():
                # add the camera found
                self.vids.append(newCap)
                index += 1
            else:  # stop searching otherwise
                break

    def get_all_feeds(self):
        while True:
            retAndFrame = []
            # read each capture object in vids and add as a tuple to retAndFrame
            for i in range(len(self.vids)):
                newFeed = self.vids[i].read()
                retAndFrame.append((newFeed[0], newFeed[1], i))

            # display each frame (camera feed) in the list
            for ret, frame, index in retAndFrame:
                cv2.imshow(f"frame {index}", frame)

            # press 'q' to stop displaying
            if cv2.waitKey(1) & 0xFF == ord('q'):
                for vid in self.vids:
                    vid.release()
                    cv2.destroyAllWindows()
                break


if __name__ == '__main__':
    setOfCameras = CameraFeed()
    setOfCameras.get_all_feeds()

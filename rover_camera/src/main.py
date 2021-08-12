import rospy as rp
import std_msgs
import cv2
from functools import *

def get_cameras(maxNum: int):
    index = 0
    arr = []
    while index <= maxNum:
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:
            arr.append(index)
        cap.release()
        index += 1
    return arr

class CameraManager:
    def __init__(self):
        rp.init_node("rover_camera")
        self.active_camera_index = 0
        cam_indices = get_cameras(100)
        self.cameras = []
        for index in cam_indices:
            self.cameras.append(cv2.VideoCapture(index))
        self.forward_sub = rp.Subscriber("/cam_control/forward", std_msgs.msg.Int32, partial(self.handle_forward, self))

    def update(self):
        rval, frame = self.cameras[0].read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def handle_forward(self):
        self.active_camera_index += 1

def main():
    camera_manager = CameraManager()
    camera_manager.update()


if __name__ == "__main__":
    main()


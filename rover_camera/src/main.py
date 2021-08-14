from __future__ import annotations
import rospy as rp
import std_msgs
import cv2
from functools import *
from typing import TypeVar, Generic

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


T = TypeVar("T")
class Node(Generic[T]):
    def __init__(self, element: T=None, next: Node[T]=None, prev: Node[T]=None):
        self.element = element
        self.next = next
        self.prev = prev

class CameraManager:
    def __init__(self):
        rp.init_node("rover_camera")
        self.active_camera_index = 0
        cam_indices = get_cameras(100)
        self.active_cam = Node[cv2.VideoCapture]()
        start = self.active_cam
        for index in cam_indices:
            new_cam = cv2.VideoCapture(index)
            self.active_cam.element = new_cam
            self.active_cam.next = Node[cv2.VideoCapture]()
            self.active_cam = self.active_cam.next

        self.active_cam.next = start

        head = start
        while True:
            head.next.prev = head
            head = head.next
            if head is start:
                break

        head = start
        while True:
            if head.element is None:
                head.prev.next = head.next
                head.next.prev = head.prev
            head = head.next
            if head is start:
                break
        
        self.forward_sub = rp.Subscriber("/cam_control/forward", std_msgs.msg.Int32, partial(self.handle_forward, self))
        self.backard_sub = rp.Subscriber("/cam_control/backward", std_msgs.msg.Int32, partial(self.handle_backward, self))

    def update(self):
        rval, frame = self.cameras[0].read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def handle_forward(self, _1, _2):
        print("forward")
        self.active_cam = self.active_cam.next

    def handle_backward(self, _1, _2):
        print("backward")
        self.active_cam = self.active_cam.prev

def main():
    camera_manager = CameraManager()
    rp.spin()


if __name__ == "__main__":
    main()


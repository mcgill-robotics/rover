#!/usr/bin/env python3

# Simple script for taking photos every few secs because I'm lazy :p
import cv2
import time

cam = cv2.VideoCapture(0)
num = 1

while(True):
    ret, frame = cam.read()
    cv2.imwrite('board-pic'+str(num)+'.jpg', frame)
    time.sleep(3)
    num += 1

    print(str(num) + " pic(s) taken!")

    if cv2.waitKey(1) == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
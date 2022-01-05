
import cv2

vids = [cv2.VideoCapture(i) for i in [0, 1, 2, 3]]
ids = [0, 1, 2, 3]

while True:
    retAndFrame = [(vids[i].read()[0], vids[i].read()[1], i) for i in range(len(vids))]
    for ret, frame, index in retAndFrame:
        cv2.imshow(f"frame {index}", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

for i in vids:
    i.release()
    cv2.destroyAllWindows()


import cvzone
import cv2

from cvzone.ClassificationModule import Classifier

cap = cv2.VideoCapture(0)
myClassifier = Classifier('RockModel/keras_model.h5','RockModel/labels.txt')


while True:
    _, img = cap.read()
    predictions, index = myClassifier.getPrediction(img)
    print(predictions)
    cv2.imshow("Image", img)
    cv2.waitKey(1)
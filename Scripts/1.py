
import cv2
cap = cv2.VideoCapture(1)
# cap.set(3, X)
# cap.set(4, Y)
while True:
    ret, frame = cap.read()
    cv2.imshow("Frame", frame)

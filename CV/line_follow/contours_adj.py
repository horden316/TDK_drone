import cv2
import numpy as np
#screen resolution
#0817 add keyinput for thershold
X=160
Y=120
h=50
center =  (int(X/2),int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)
cross_size = 5
cap = cv2.VideoCapture(0)
cap.set(3, X)
cap.set(4, Y)
def distanceCalculate(p1, p2):
    """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis
while True:
    keyPress = cv2.waitKey(20)
    ret, frame = cap.read()
    #frame = cv2.imread('./webcam/opencv_frame_0.png')
    low_b = np.uint8([255,255,255])
    high_b = np.uint8([h,h,h])
    mask = cv2.inRange(frame, high_b, low_b)
    remask = cv2.bitwise_not(mask)
    contours, hierarchy = cv2.findContours(remask, 1, cv2.CHAIN_APPROX_SIMPLE)
    cv2.line(frame, (center_x, center_y-cross_size), (center_x, center_y+cross_size), (0, 0, 255), 1)
    cv2.line(frame, (center_x-cross_size, center_y), (center_x+cross_size, center_y), (0, 0, 255), 1)
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        blackbox = cv2.minAreaRect(c)
        (x_min, y_min), (w_min, h_min), angle = blackbox
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
        if angle < -45:
            angle = 90 + angle
        if w_min < h_min and angle > 0:
            angle = (90 - angle) * -1
        if w_min > h_min and angle < 0:
            angle = 90 + angle
        print("Angle:" + str(angle))
        cv2.putText(frame, "Angle: " + str(angle), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1,
                        cv2.LINE_AA)
        if M["m00"] !=0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("X : "+str(cx)+" Y : "+str(cy))
            # if cx >= 120 :
            #     print("Turn Left")
            # if cx < 120 and cx > 40 :
            #     print("On Track!")
            # if cx <=40 :
            #     print("Turn Right")
            #centroid circle
            cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)
            #centroid line
            cv2.line(frame,  center, (cx,cy), (0,255,255), 1)
            #BGR
            cv2.line(frame,  center, (cx,cy), (0,255,255), 1)
            distance = distanceCalculate(center, (cx,cy))
            cv2.putText(frame, "Distance: " + str(distance), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1,
                        cv2.LINE_AA)
        
    else :
        print("I don't see the line")
    #cv2.drawContours(frame, c, -1, (0,255,0), 5)
    cv2.putText(remask, "H: " + str(h), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1,
                        cv2.LINE_AA)
    cv2.imshow("Mask",remask)
    cv2.imshow("Frame",frame)
    if keyPress & 0xff == ord('q'):   # 1 is the time in ms
        break
    elif keyPress & 0xff == ord('a'):
        h=h-1
    elif keyPress & 0xff == ord('d'):
        h=h+1
cap.release()
cv2.destroyAllWindows()


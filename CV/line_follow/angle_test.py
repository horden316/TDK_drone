import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import os
import time
import math
h=50

while True:
    keyPress = cv2.waitKey(20)
    basepath=os.getcwd()
    frame=cv2.imread(basepath+"/line_following_scripts/curve8.png")
    #frame = cv2.imread('./webcam/opencv_frame_0.png')
    low_b = np.uint8([255,255,255])
    high_b = np.uint8([h,h,h])
    mask = cv2.inRange(frame, high_b, low_b)
    remask = cv2.bitwise_not(mask)
    contours, hierarchy = cv2.findContours(remask, 1, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        blackbox = cv2.minAreaRect(c)
        (x_min, y_min), (w_min, h_min), angle = blackbox
        box = cv2.boxPoints(blackbox)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
        #print("angle:"+str(angle))
        if angle < -45:
            angle = 90 + angle
            print("after angle1:" + str(angle))
        if w_min < h_min and angle > 0:
            angle = (90 - angle) * -1
            print("after angle2:" + str(angle))
        if w_min > h_min and angle < 0:
            angle = 90 + angle
            print("after angle3:" + str(angle))
        #print("after angle:" + str(angle))
        cv2.putText(frame, "Angle: " + str(angle), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1,
                        cv2.LINE_AA)
        if M["m00"] !=0 :
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            
            # if cx >= 120 :
            #     print("Turn Left")
            # if cx < 120 and cx > 40 :
            #     print("On Track!")
            # if cx <=40 :
            #     print("Turn Right")
            
        
    else :
        print("I don't see the line")
    #cv2.drawContours(frame, c, -1, (0,255,0), 5)
    cv2.imshow("Mask",remask)
    cv2.imshow("Frame",frame)
    if keyPress & 0xff == ord('q'):   # 1 is the time in ms
        break


cv2.destroyAllWindows()
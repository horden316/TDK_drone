import cv2
import numpy as np
import os
import time
#screen resolution
#0817 add keyinput for thershold
X=160
Y=120
h=50
center =  (int(X/2),int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)
cross_size = 5
cap = cv2.VideoCapture("C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\video.mp4")
# cap.set(3, X)
# cap.set(4, Y)
def distanceCalculate(p1, p2):
    """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis
while True:
    keyPress = cv2.waitKey(20)
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))
    frame_color = frame.copy()
    display = frame.copy()
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
    #####################################red light detect#####################################
    hsvFrame = cv2.cvtColor(frame_color, cv2.COLOR_BGR2HSV)
    # Set range for red color and 
    # define mask
    #red_lower = np.array([136, 87, 111], np.uint8)
    red_lower = np.array([93, 83, 204], np.uint8)
    #red_upper = np.array([180, 255, 255], np.uint8)
    red_upper = np.array([198, 130, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    contours_color, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours_color):
        c_color = max(contours, key=cv2.contourArea)
        M = cv2.moments(c_color)
        area = cv2.contourArea(contour)
        if(area > 800):
            red = True
            x, y, w, h = cv2.boundingRect(contour)
            frame = cv2.rectangle(frame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2)
            if M["m00"] !=0 :
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("X : "+str(cx)+" Y : "+str(cy))
                cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)
                cv2.putText(frame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255)) 
    #######################################blue_circle_detect#######################################
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    blue_mask = cv2.bitwise_not(blue_mask)
    # Convert to grayscale.
    gray = cv2.cvtColor(frame_color, cv2.COLOR_BGR2GRAY)
    # Blur using 3 * 3 kernel.
    gray_blurred = cv2.blur(blue_mask, (3, 3))
    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_blurred, 
                    cv2.HOUGH_GRADIENT, 1, 20, param1 = 100,
                param2 = 20, minRadius = 30, maxRadius = 80)
    
    # Draw circles that are detected.
    if detected_circles is not None:
    
        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
    
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
    
            # Draw the circumference of the circle.
            cv2.circle(frame, (a, b), r, (0, 255, 0), 2)
    
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(frame, (a, b), 1, (0, 0, 255), 3)
            # cv2.imshow("Detected Circle", img)
    cv2.imshow("Mask",remask)
    cv2.imshow("Frame",frame)
    cv2.imshow("Disply",display)
    cv2.imshow("red_mask", red_mask)
    cv2.imshow("blue_mask", blue_mask)
    if keyPress & 0xff == ord('q'):   # 1 is the time in ms
        break

cap.release()
cv2.destroyAllWindows()



# Python code for Multiple Color Detection
from pickle import TRUE
import cv2
import numpy as np
import time
X=160
Y=120
red = False
# Capturing video through webcam
webcam = cv2.VideoCapture(0)
# Start a while loop
webcam.set(3, X)
webcam.set(4, Y)
start = time.time()
while(1):
    print("red:"+ str(red))
    # Reading the video from the
    # webcam in image frames
    _, imageFrame = webcam.read() 
    # Convert the imageFrame in 
    # BGR(RGB color space) to 
    # HSV(hue-saturation-value)
    # color space 
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color and 
    # define mask
    #red_lower = np.array([136, 87, 111], np.uint8)
    red_lower = np.array([110, 80, 60], np.uint8)
    #red_upper = np.array([180, 255, 255], np.uint8)
    red_upper = np.array([236, 221, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and 
    # define mask
    #green_lower = np.array([25, 52, 72], np.uint8)
    green_lower = np.array([25, 52, 90], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # Set range for blue color and
    # define mask
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    
        
    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")
        
    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                                mask = red_mask)
        
    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = green_mask)
        
    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                                mask = blue_mask)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                            cv2.RETR_TREE,
                                            cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        area = cv2.contourArea(contour)
        if(area > 800):
            red = True
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2)
            if M["m00"] !=0 :
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("X : "+str(cx)+" Y : "+str(cy))
                cv2.circle(imageFrame, (cx,cy), 5, (0,0,255), -1)
                cv2.putText(imageFrame, "Red Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))
            start = time.time()
        if((time.time() - start)>0.5):
            red = False  

    # # Creating contour to track green color
    if not red:
        contours, hierarchy = cv2.findContours(green_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
            
        for pic, contour in enumerate(contours):
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            area = cv2.contourArea(contour)
            if(area > 1500):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                            (x + w, y + h),
                                            (0, 255, 0), 2)
                if M["m00"] !=0 :
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print("X : "+str(cx)+" Y : "+str(cy))
                    cv2.circle(imageFrame, (cx,cy), 5, (0,0,255), -1)
                    cv2.putText(imageFrame, "Green Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, (0, 255, 0))

    # # Creating contour to track blue color
    # contours, hierarchy = cv2.findContours(blue_mask,
    #                                         cv2.RETR_TREE,
    #                                         cv2.CHAIN_APPROX_SIMPLE)
    # for pic, contour in enumerate(contours):
    #     area = cv2.contourArea(contour)
    #     if(area > 300):
    #         x, y, w, h = cv2.boundingRect(contour)
    #         imageFrame = cv2.rectangle(imageFrame, (x, y),
    #                                     (x + w, y + h),
    #                                     (255, 0, 0), 2)
                
    #         cv2.putText(imageFrame, "Blue Colour", (x, y),
    #                     cv2.FONT_HERSHEY_SIMPLEX,
    #                     1.0, (255, 0, 0))
 
    # Program Termination
    cv2.imshow("hsv", hsvFrame)
    cv2.imshow("green_mask", green_mask)
    cv2.imshow("red_mask", red_mask)
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        webcam.release()
        cv2.destroyAllWindows()
        break
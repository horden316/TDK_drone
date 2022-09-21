import cv2
import numpy as np
webcam = cv2.VideoCapture(0)
while(1):
    _, img = webcam.read() 
    # color space 
    hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #set blue thres
    blue_lower = np.array([94, 80, 2])
    blue_upper = np.array([120, 255, 255])
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    blue_mask = cv2.bitwise_not(blue_mask)
    # Convert to grayscale.
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Blur using 3 * 3 kernel.
    gray_blurred = cv2.blur(blue_mask, (3, 3))
    cv2.imshow("gray_blurred", gray_blurred)
    cv2.imshow("blue_mask", blue_mask)
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
            cv2.circle(img, (a, b), r, (0, 255, 0), 2)
    
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(img, (a, b), 1, (0, 0, 255), 3)
            # cv2.imshow("Detected Circle", img)
    cv2.imshow("Detected Circle", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        webcam.release()
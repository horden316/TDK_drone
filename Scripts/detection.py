import numpy as np
import cv2
import time
X = 160
Y = 120
center = (int(X/2), int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)
red = False


def distanceCalculate(p1, p2):
    """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis


def line_detect(frame, draw_frame, line_mask=50, cross_size=5, c_area=300):
    ############init_val#############
    (cx, cy) = (0.0, 0.0)
    angle = 0
    out_mask = 0
    x_distance = 0.0
    y_distance = 0.0
    #################################
    low_b = np.uint8([255, 255, 255])
    high_b = np.uint8([line_mask, line_mask, line_mask])
    mask = cv2.inRange(frame, high_b, low_b)
    remask = cv2.bitwise_not(mask)
    out_mask = cv2.merge((remask, remask, remask))
    contours, hierarchy = cv2.findContours(
        remask, 1, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        area = cv2.contourArea(contour)
        if (area > c_area):
            blackbox = cv2.minAreaRect(c)
            (x_min, y_min), (w_min, h_min), angle = blackbox
            box = cv2.boxPoints(blackbox)
            box = np.int0(box)
            cv2.drawContours(draw_frame, [box], 0, (0, 0, 255), 3)
            if angle < -45:
                angle = 90 + angle
            if w_min < h_min and angle > 0:
                angle = (90 - angle) * -1
            if w_min > h_min and angle < 0:
                angle = 90 + angle
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                x_distance = center[0]-cx
                y_distance = center[1]-cy
                # print("X : "+str(cx)+" Y : "+str(cy))
                # centroid circle
                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)
                # centroid line
                cv2.line(draw_frame,  center,
                         (cx, cy), (0, 255, 255), 1)
                # BGR
                cv2.line(draw_frame,  center,
                         (cx, cy), (0, 255, 255), 1)
                distance = distanceCalculate(center, (cx, cy))
    return (cx, cy), angle, frame, out_mask, x_distance, y_distance


def traffic_detect(frame, draw_frame, red_lower, red_upper, c_area=800):
    ############init_val#############
    red = None
    (cx, cy) = (0.0, 0.0)
    out_mask = 0
    x_distance = 0.0
    y_distance = 0.0
    #################################
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red_mask2 = cv2.merge((red_mask, red_mask, red_mask))
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        c_color = max(contours, key=cv2.contourArea)
        M = cv2.moments(c_color)
        area = cv2.contourArea(contour)
        if (area > 800):
            red = True
            x, y, w, h = cv2.boundingRect(contour)
            draw_frame = cv2.rectangle(draw_frame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                x_distance = center[0]-cx
                y_distance = center[1]-cy
                print("X : "+str(cx)+" Y : "+str(cy))
                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(draw_frame, "Red Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
    return red, red_mask2, draw_frame, (cx, cy), x_distance, y_distance


def drop_detect(frame, draw_frame, blue_lower, blue_upper):
    ############init_val#############
    drop = None
    (a, b) = (0.0, 0.0)
    out_mask = 0
    x_distance = 0.0
    y_distance = 0.0
    #################################
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # set blue thres
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    blue_mask = cv2.bitwise_not(blue_mask)
    out_mask = cv2.merge((blue_mask, blue_mask, blue_mask))
    # Convert to grayscale.
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Blur using 3 * 3 kernel.
    gray_blurred = cv2.blur(blue_mask, (3, 3))
    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 20, param1=100,
                                        param2=20, minRadius=30, maxRadius=80)

    # Draw circles that are detected.
    if detected_circles is not None:
        drop = True
        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            x_distance = center[0]-a
            y_distance = center[1]-b
            # Draw the circumference of the circle.
            cv2.circle(draw_frame, (a, b), r, (0, 255, 0), 2)
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(draw_frame, (a, b), 1, (0, 0, 255), 3)
            # cv2.imshow("Detected Circle", img)
    return drop, out_mask, draw_frame, (a, b), x_distance, y_distance


def traffic_detect(frame, draw_frame, red_lower, red_upper, c_area=800):
    ############init_val#############
    red = None
    (cx, cy) = (0.0, 0.0)
    out_mask = 0
    x_distance = 0.0
    y_distance = 0.0
    #################################
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red_mask2 = cv2.merge((red_mask, red_mask, red_mask))
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        c_color = max(contours, key=cv2.contourArea)
        M = cv2.moments(c_color)
        area = cv2.contourArea(contour)
        if (area > 800):
            red = True
            x, y, w, h = cv2.boundingRect(contour)
            draw_frame = cv2.rectangle(draw_frame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                x_distance = center[0]-cx
                y_distance = center[1]-cy
                print("X : "+str(cx)+" Y : "+str(cy))
                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(draw_frame, "Red Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
    return red, red_mask2, draw_frame, (cx, cy), x_distance, y_distance


def red_h_detect(frame, draw_frame, red_lower, red_upper, c_area=800):
    ############init_val#############
    red_h = None
    (cx, cy) = (0.0, 0.0)
    out_mask = 0
    x_distance = 0.0
    y_distance = 0.0
    #################################
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_h_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
    red_h_mask2 = cv2.merge((red_h_mask, red_h_mask, red_h_mask))
    contours, hierarchy = cv2.findContours(red_h_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        c_color = max(contours, key=cv2.contourArea)
        M = cv2.moments(c_color)
        area = cv2.contourArea(contour)
        if (area > 800):
            red = True
            x, y, w, h = cv2.boundingRect(contour)
            draw_frame = cv2.rectangle(draw_frame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)
            if M["m00"] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                x_distance = center[0]-cx
                y_distance = center[1]-cy
                print("X : "+str(cx)+" Y : "+str(cy))
                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(draw_frame, "Red Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
    return red_h, red_h_mask2, draw_frame, (cx, cy), x_distance, y_distance

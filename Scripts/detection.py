import numpy as np
import cv2
X = 160
Y = 120
center = (int(X/2), int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)


class line_detection():
    def __init__(self, frame, line_mask_thres=50, cross_size=5):
        h = line_mask_thres
        cross_size = cross_size
        self.low_b = np.uint8([255, 255, 255])
        self.high_b = np.uint8([h, h, h])
        self.mask = cv2.inRange(frame, high_b, low_b)
        self.remask = cv2.bitwise_not(mask)
        self.frame = frame
        cv2.line(frame, (center_x, center_y-cross_size),
                 (center_x, center_y+cross_size), (0, 0, 255), 1)
        cv2.line(frame, (center_x-cross_size, center_y),
                 (center_x+cross_size, center_y), (0, 0, 255), 1)

    def distanceCalculate(p1, p2):
        """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
        dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
        return dis

    def line_detection(self):
        contours, hierarchy = cv2.findContours(
            self.remask, 1, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            area = cv2.contourArea(contour)
            if (area > 800):
                blackbox = cv2.minAreaRect(c)
                (x_min, y_min), (w_min, h_min), angle = blackbox
                box = cv2.boxPoints(blackbox)
                box = np.int0(box)
                cv2.drawContours(self.frame, [box], 0, (0, 0, 255), 3)
                if angle < -45:
                    angle = 90 + angle
                if w_min < h_min and angle > 0:
                    angle = (90 - angle) * -1
                if w_min > h_min and angle < 0:
                    angle = 90 + angle
                print("Angle:" + str(angle))
                cv2.putText(self.frame, "Angle: " + str(angle), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1,
                            cv2.LINE_AA)
                if M["m00"] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print("X : "+str(cx)+" Y : "+str(cy))
                    # centroid circle
                    cv2.circle(self.frame, (cx, cy), 5, (0, 0, 255), -1)
                    # centroid line
                    cv2.line(self.frame,  self.enter,
                             (cx, cy), (0, 255, 255), 1)
                    # BGR
                    cv2.line(self.frame,  self.center,
                             (cx, cy), (0, 255, 255), 1)
                    distance = self.distanceCalculate(self.enter, (cx, cy))
                else:
                    print("I don't see the line")
                #cv2.drawContours(frame, c, -1, (0,255,0), 5)
                cv2.imshow("Mask", remask)
                cv2.imshow("Frame", frame)
                # def traffic_detection():
                # def drop_detection():
                # def des_detection();

from turtle import color
import cv2
import numpy as np
import time
colors = []
red = False
thres = 10
lb = [0, 0, 0]
ub = [0, 0, 0]
def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())
def do_nothing():
    pass
def main():
    capture = cv2.VideoCapture("C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\red_h2.jpg")
    start = time.time()
    while True:
        _, frame = capture.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS_FULL)
        if colors:
            cv2.putText(frame, str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
        cv2.imshow('frame', frame)
        cv2.setMouseCallback('frame', on_mouse_click, frame)
        print(colors)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break
    capture.release()
    cv2.destroyAllWindows()

    # avgb = int(sum(c[0] for c in colors) / len(colors))
    # avgg = int(sum(c[0] for c in colors) / len(colors))
    # avgr = int(sum(c[0] for c in colors) / len(colors))
    # print avgb, avgg, avgr

    minb = min(c[0] for c in colors)
    ming = min(c[1] for c in colors)
    minr = min(c[2] for c in colors)
    maxb = max(c[0] for c in colors)
    maxg = max(c[1] for c in colors)
    maxr = max(c[2] for c in colors)
    print (minr, ming, minb, maxr, maxg, maxb)

    lb = [minb,ming,minr]
    ub = [maxb,maxg,maxr]
    print (lb, ub)
    cv2.namedWindow("Slider")
    cv2.resizeWindow("Slider", 640, 480)
    cv2.createTrackbar("R MAX", "Slider", 0, 255, do_nothing)
    cv2.createTrackbar("R MIN", "Slider", 0, 255, do_nothing)
    cv2.createTrackbar("G MAX", "Slider", 0, 255, do_nothing)
    cv2.createTrackbar("G MIN", "Slider", 0, 255, do_nothing)
    cv2.createTrackbar("B MAX", "Slider", 0, 255, do_nothing)
    cv2.createTrackbar("B MIN", "Slider", 0, 255, do_nothing)
    capture = cv2.VideoCapture("C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\red_h2.jpg")
    while(1):
        r_max = cv2.getTrackbarPos("R MAX", "Slider")
        r_min = cv2.getTrackbarPos("R MIN", "Slider")
        g_max = cv2.getTrackbarPos("G MAX", "Slider")
        g_min = cv2.getTrackbarPos("G MIN", "Slider")
        b_max = cv2.getTrackbarPos("B MAX", "Slider")
        b_min = cv2.getTrackbarPos("B MIN", "Slider")
        # average_color = [63, 153, 172]
        #print("red:"+ str(red))
        _, imageFrame = capture.read() 
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        # color_upper = np.array([ub[0] + thres, ub[1] + thres, ub[2] + thres], np.uint8)
        # color_lower = np.array([lb[0] - thres, lb[1] - thres, lb[2] - thres], np.uint8)
        color_upper = np.array([b_max, g_max, r_max], np.uint8)
        color_lower = np.array([b_min, g_min, r_min], np.uint8)
        print("color_upper:" + str(color_upper))
        print("color_lower:" + str(color_lower))
        color_mask = cv2.inRange(hsvFrame, color_lower, color_upper)

        kernal = np.ones((5, 5), "uint8")
            
        # For red color
        color_mask = cv2.dilate(color_mask, kernal)
        res_color = cv2.bitwise_and(imageFrame, imageFrame, 
                                    mask = color_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(color_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            print("upper:" + str(color_upper))
            print("lower:" + str(color_lower))
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            area = cv2.contourArea(contour)
            if(area > 800):
                red = True
                x, y, w, h = cv2.boundingRect(contour)
                # imageFrame = cv2.rectangle(imageFrame, (x, y), 
                #                             (x + w, y + h), 
                #                             (0, 0, 255), 2)
                if M["m00"] !=0 :
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print("X : "+str(cx)+" Y : "+str(cy))
                    # cv2.circle(imageFrame, (cx,cy), 5, (0,0,255), -1)
                    # cv2.putText(imageFrame, "Red Colour", (x, y),
                    #         cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    #         (0, 0, 255))
                start = time.time()
            if((time.time() - start)>0.5):
                red = False  

        cv2.imshow("color_mask", color_mask)
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            capture.release()
            cv2.destroyAllWindows()
            break

if __name__ == "__main__":
    main()
    
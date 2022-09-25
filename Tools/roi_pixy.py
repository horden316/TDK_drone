import cv2
import numpy as np
import time
showCrosshair = False
fromCenter = False
start = time.time()
red = False
webcam = cv2.VideoCapture(0)
thres = 40
if __name__ == '__main__' :
    while(1):
        # Read image
        # im = cv2.imread("./webcam/opencv_frame_0.png")
        ret, frame =webcam.read()
        #show frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord(' '): 
            webcam.release()
            cv2.destroyAllWindows()
            break
    # Select ROI
    r = cv2.selectROI("Image", frame, fromCenter, showCrosshair)
    print(r)
    # Crop image
    imCrop = frame[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
    # Display cropped image
    #cv2.imshow("Image", imCrop)
    average_color_row = np.average(imCrop, axis=0)
    average_color = np.average(average_color_row, axis=0)
    print("Average Color:" + str(average_color))

    d_img = np.ones((312,312,3), dtype=np.uint8)
    d_img[:,:] = average_color

    cv2.imshow('Source image',imCrop)
    cv2.imshow('Average Color',d_img)
    if cv2.waitKey(0) & 0xFF == ord(' '): 
        cv2.destroyAllWindows()
    webcam = cv2.VideoCapture(0)
    while(1):
        # average_color = [63, 153, 172]
        #print("red:"+ str(red))
        _, imageFrame = webcam.read() 
        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # color_upper = np.array([average_color[0] + thres, average_color[1] + thres, average_color[2] + thres], np.uint8)
        # color_lower = np.array([average_color[0] - thres, average_color[1] - thres, average_color[2] - thres], np.uint8)
        color_upper = np.array([255, 255, 255], np.uint8)
        color_lower = np.array([0, 0, 0], np.uint8)
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

        cv2.imshow("color_mask", color_mask)
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            webcam.release()
            cv2.destroyAllWindows()
            break


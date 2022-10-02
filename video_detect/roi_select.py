from time import sleep
import cv2
import numpy as np
showCrosshair = False
fromCenter = False
webcam = cv2.VideoCapture(0)
if __name__ == '__main__' :
    while(1):
        # Read image
        # im = cv2.imread("./webcam/opencv_frame_0.png")
        ret, frame =webcam.read()
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
        #cv2.waitKey(0)
        if cv2.waitKey(1) & 0xFF == ord(' '): break
    webcam.release()
    cv2.destroyAllWindows()


import cv2
import numpy as np
#screen resolution
X=160
Y=120
center =  (int(X/2),int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)
cross_size = 5
cap = cv2.VideoCapture(1)
FixedText_array=[]
#VideoWriter
fourcc = cv2.VideoWriter_fourcc('M','P','4','V') #指定影像編碼方式
out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (1280,  960))

#建立空frame2
blank_width=1280
blank_height=960

cap.set(3, X)
cap.set(4, Y)
SetFixedText_seq=0
def WriteText(frame2, text, seq): #(frame,文字,第幾個)
    Y_offset=480
    font_gap_px=20
    
    font_start_Y_px = seq*font_gap_px
    cv2.putText(frame2, text, (0, Y+Y_offset+font_start_Y_px), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)

def SetFixedText(text):
    global SetFixedText_seq
    global FixedText_array
    # print(FixedText_array[SetFixedText_seq])
    FixedText_array.append(text)
    print("number:" + str(SetFixedText_seq))
    SetFixedText_seq = SetFixedText_seq+1

def WriteFixedText(frame2): #(frame,文字,第幾個)
    X_offset=960
    #font_gap_px=20

    global SetFixedText_seq
    global FixedText_array
    
    #font_start_Y_px = SetFixedText_seq*font_gap_px
    for i in range (len(FixedText_array)):
        cv2.putText(frame2, FixedText_array[i], (X+X_offset,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)

def distanceCalculate(p1, p2):
    """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis
while True:
    ret, frame = cap.read()

    frame2=np.zeros((blank_height, blank_width,3),np.uint8)

    #frame = cv2.imread('./webcam/opencv_frame_0.png')
    low_b = np.uint8([255,255,255])
    high_b = np.uint8([50,50,50])
    mask = cv2.inRange(frame, high_b, low_b)
    remask = cv2.bitwise_not(mask)
    contours, hierarchy = cv2.findContours(remask, 1, cv2.CHAIN_APPROX_SIMPLE)
    cv2.line(frame, (center_x, center_y-cross_size), (center_x, center_y+cross_size), (0, 0, 255), 1)
    cv2.line(frame, (center_x-cross_size, center_y), (center_x+cross_size, center_y), (0, 0, 255), 1)
    if len(contours) > 0 :
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        # blackbox = cv2.minAreaRect(contours_blk[0])
        # (x_min, y_min), (w_min, h_min), angle = blackbox
        #     if angle < -45:
        #         angle = 90 + angle
        #     if w_min < h_min and angle > 0:
        #         angle = (90 - angle) * -1
        #     if w_min > h_min and angle < 0:
        #         angle = 90 + angle

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
            WriteText(frame2, "Distance: " + str(distance),1)
            WriteFixedText(frame2)

    else :
        print("I don't see the line")
        SetFixedText("I don't see the line")
    cv2.drawContours(frame, c, -1, (0,255,0), 5)
    cv2.imshow("Mask",remask)
    cv2.imshow("Frame",frame)

    h,w,_ = frame.shape
    frame2[0:h, 0:w] = frame
    cv2.imshow("frame2", frame2)

    
    out.write(frame2)


    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        break
cap.release()
out.release()
cv2.destroyAllWindows()


# add PID control to follow.py
from curses import KEY_PPAGE
from msilib.schema import Error
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math
from control import control
from PID import PID
SetFixedText_seq = 0
FixedText_array = []

# VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 指定影像編碼方式
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (480,  360))

# 建立空frame2
blank_width = 480
blank_height = 360

# screen resolution
X = 160
Y = 120
center = (int(X/2), int(Y/2))
center_x = int(X/2)
center_y = int(Y/2)
cross_size = 5
cap = cv2.VideoCapture(0)
cap.set(3, X)
cap.set(4, Y)
# PID variables
Kp = 0.8
Ki = 0
Kd = 0
Target_value = 0
last_Err = 0
total_Err = 0
output = 0
#################################drone connection#################################
connection_string = '/dev/ttyACM0'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=115200)


def WriteText(frame2, text, seq):  # (frame,文字,第幾個)
    Y_offset = 0
    font_gap_px = 20

    font_start_Y_px = seq*font_gap_px
    cv2.putText(frame2, text, (0, Y+Y_offset+font_start_Y_px),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)


def SetFixedText(text):
    global SetFixedText_seq
    global FixedText_array
    # print(FixedText_array[SetFixedText_seq])
    FixedText_array.append(text)
    SetFixedText_seq = SetFixedText_seq+1


def WriteFixedText(frame2):  # (frame,文字,第幾個)
    X_offset = 0
    # font_gap_px=20

    global SetFixedText_seq
    global FixedText_array

    #font_start_Y_px = SetFixedText_seq*font_gap_px
    for i in range(len(FixedText_array)):
        cv2.putText(frame2, FixedText_array[i], (X+X_offset, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)


##############主程式##############
time.sleep(2)
print("takeoff")
yawangle = math.degrees(vehicle.attitude.yaw)
control.arm_and_takeoff_nogps(aTargetAltitude=0.5, DEFAULT_TAKEOFF_THRUST=0.55,
                              SMOOTH_TAKEOFF_THRUST=0.52, limit_time=10, default_yaw=True)
# 起飛後直行一段時間
control.set_attitude(yaw_angle=yawangle, pitch_angle=-
                     5, thrust=0.5, duration=2)

start = time.time()
while True:
    ret, frame = cap.read()
    frame2 = np.zeros((blank_height, blank_width, 3), np.uint8)
    low_b = np.uint8([255, 255, 255])
    high_b = np.uint8([50, 50, 50])
    mask = cv2.inRange(frame, high_b, low_b)
    remask = cv2.bitwise_not(mask)
    kernel = np.ones((3, 3), np.uint8)
    erosion = cv2.erode(remask, kernel, iterations=1)
    contours, hierarchy = cv2.findContours(erosion, 1, cv2.CHAIN_APPROX_SIMPLE)
    cv2.line(frame, (center_x, center_y-cross_size),
             (center_x, center_y+cross_size), (0, 0, 255), 1)
    cv2.line(frame, (center_x-cross_size, center_y),
             (center_x+cross_size, center_y), (0, 0, 255), 1)
    if time.time() - start > 20:
        print("Setting LAND mode...")
        SetFixedText("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        break
    for pic, contour in enumerate(contours):
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            area = cv2.contourArea(contour)
            if (area > 800):
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

                if M["m00"] != 0:
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    # centroid line
                    cv2.line(frame,  center, (cx, cy), (0, 255, 255), 1)
                    # BGR
                    cv2.line(frame,  center, (cx, cy), (0, 255, 255), 1)
                    #distance = distanceCalculate(center, (cx,cy))
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    print("X : "+str(cx)+" Y : "+str(cy))
                    #########################Roll 的 PID 控制#########################
                    x_distance = center[0]-cx
                    y_distance = center[1]-cy
                    pid_roll = PID(
                        Error=x_distance, Kp=0.8, Ki=0, Kd=0, max_angle=15, a=0.2)
                    roll_angle = pid_roll.PID()
            # yaw調整(yaw_angle)1絕對調整
                    if angle > 0:
                        theta = 90 - angle
                        yaw_angle = yawangle-theta
                        print("current_yaw:" +
                              str(math.degrees(vehicle.attitude.yaw)))
                        WriteText(frame2, "current_yaw:" +
                                  str(math.degrees(vehicle.attitude.yaw)), 4)
                        print("set:"+str(yawangle-theta))
                        WriteText(frame2, "set:"+str(yawangle-theta), 3)
                        print("yaw right")
                        WriteText(frame2, "yaw right", 5)
                    elif angle <= 0:
                        theta = 90 + angle
                        yaw_angle = yawangle+theta
                        print("current_yaw:" +
                              str(math.degrees(vehicle.attitude.yaw)))
                        WriteText(frame2, "current_yaw:" +
                                  str(math.degrees(vehicle.attitude.yaw)), 4)
                        print("set:"+str(yawangle+theta))
                        WriteText(frame2, "set:"+str(yawangle+theta), 3)
                        print("yaw left")
                        WriteText(frame2, "yaw left", 5)
                    else:
                        print("Pitch Forward")
                        WriteText(frame2, "Pitch Forward", 5)
                        print("current_yaw:" +
                              str(math.degrees(vehicle.attitude.yaw)))
                        WriteText(frame2, "current_yaw:" +
                                  str(math.degrees(vehicle.attitude.yaw)), 4)
            # pitch(直走)觸發條件
                    if theta > -15 and theta < 15 and x_distance > -30 and x_distance < 30:
                        pitch_angle = -5
                    else:
                        pitch_angle = 0

                    ###########################送出set_altitude 指令###########################
                    control.set_attitude(pitch_angle=pitch_angle, yaw_angle=yaw_angle,
                                         roll_angle=roll_angle, thrust=0.5)

        else:
            print("I don't see the line")
            WriteText(frame2, "I don't see the line", 1)
        #cv2.drawContours(frame, c, -1, (0,255,0), 5)
        # cv2.imshow("Mask",remask)
        # cv2.imshow("Erosion",erosion)
    cv2.imshow("Frame", frame)

    h, w, _ = frame.shape
    frame2[0:h, 0:w] = frame
    cv2.imshow("frame2", frame2)

    WriteFixedText(frame2)

    out.write(frame2)

    if cv2.waitKey(1) & 0xff == ord('q'):   # 1 is the time in ms
        print("Setting LAND mode...")
        SetFixedText("Setting LAND mode...")
        vehicle.mode = VehicleMode("LAND")
        break
print("Close vehicle object")
vehicle.close()
print('mission complete')
cap.release()
out.release()
cv2.destroyAllWindows()

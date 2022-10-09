import cv2
import numpy as np
import os
import time
from log import log
from detection import *
# from control import *  # 會直接與mavlink連接
import math
section = 1
# cap = cv2.VideoCapture(
#     "C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\video.mp4")
cap = cv2.VideoCapture(0)
##############狀態參數##############
status = None
red_count = 0
thrust = 0
##############時間參數##############
red_stay_time = time.time()
##############mask參數##############
# 黑線mask
h = 50
# 紅燈mask
red_lower = np.array([93, 83, 204], np.uint8)
red_upper = np.array([198, 130, 255], np.uint8)
# 藍色投放點mask
blue_lower = np.array([94, 80, 2])
blue_upper = np.array([120, 255, 255])
line_mask = 0
blue_mask = 0
red_mask = 0
# LOG 錄製
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 指定影像編碼方式
out = cv2.VideoWriter("output"+str(int(time.time())) +
                      ".avi", fourcc, 20.0, (480,  360))
##################起飛準備##################
while (1):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))
    draw_frame = frame.copy()
    ###################機體狀態###################
    # c_alt = vehicle.rangefinder.distance
    # c_pitch = math.degree(vehicle.attitude.roll)
    # c_roll = math.degree(vehicle.attitude.roll)
    # c_yaw = math.degree(vehicle.attitude.roll)

    #########################section-1#########################
    if section == -1:
        status = "takeoff_follow"
        print("takeoff_follow")
    #########################section0#########################
    if section == 0:
        status = "takeoff_forward"
        print("takeoff_forward")
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h)
        # def takeoff () return thrust
        # def moveforward

    #########################section1#########################
    #走線 + 紅燈辨識
    if section == 1:
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h)
        red, red_mask, draw_frame, (tx, ty), t_x_dis, t_y_dis = traffic_detect(
            frame=frame, draw_frame=draw_frame, red_lower=red_lower, red_upper=red_upper)
        if (lx, ly) == (0, 0) and line_angle == 0:
            print("I don't see the line")
        if red == True:
            red_count += 1
            red_stay_time = time.time()
            print("stay")
            status = "stay"

        # 如果轉false 清除目標值再走
        if (time.time()-red_stay_time) > 0.1:
            status = "stay clear pitch roll"
            pitch_angle = 0
            roll_angle = 0
        # 如果轉false 等1再走
        if (time.time()-red_stay_time) > 3:
            print("move forward")
            status = "move forward"
            # 跳下個section ###!!!!!!隱患若是偵測到錯的redlight 將會跳下個section 法一:下個section也偵測red
            # 若是辨識過red
            if red_count > 0:
                section = 2
    #########################section2#########################
    # 走線 + 投遞:
    if section == 2:
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=50)
        drop, blue_mask, draw_frame, (bx, by), x_distance, y_distance = drop_detect(
            frame, draw_frame, blue_lower=blue_lower, blue_upper=blue_upper)
        if drop == True:
            print("stay & drop")
        else:
            print("move forward")
    #########################section3#########################
    # if section==3:
    # if section==4:
    ######################### SET ALTITUDE #########################
    back_frame = log(frame=draw_frame, lane_mask=line_mask, red_mask=red_mask,
                     drop_mask=blue_mask, h_mask=0, ex_frame=0, lane_xy=(lx, ly), lane_angle=line_angle,
                     lane_dis=line_x_dis, status=status, section=section, thrust=thrust)

    # log(frame=(100, 0, 0), lane_mask=(100, 0, 0), red_mask=(100, 0, 0), drop_mask=(100, 0, 0), h_mask=(100, 0, 0), ex_frame=(100, 0, 0),
    #     show=True, alt=0.0, pitch=0.0, roll=0.0, yaw=0.0,
    #     t_alt=0.0, t_pitch=0.0, t_roll=0.0, t_yaw=0.0,
    #     lane_xy=(0.0, 0.0), lane_angle=0.0, lane_dis=0.0,
    #     target="None", target_xy=(0.0, 0.0), status="None", section=0)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
    out.write(back_frame)
cap.release()
cv2.destroyAllWindows()

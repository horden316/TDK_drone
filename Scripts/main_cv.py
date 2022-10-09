import cv2
import numpy as np
import os
import time
from log import log
from detection import *
section = 2
# cap = cv2.VideoCapture(
#     "C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\video.mp4")
cap = cv2.VideoCapture(0)
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
while (1):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))
    draw_frame = frame.copy()
    #########################section0#########################
    if section == 0:
        print("take_off")
    #########################section1#########################
    #走線 + 紅燈辨識
    if section == 1:
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=50)
        red, red_mask, draw_frame, (tx, ty), t_x_dis, t_y_dis = traffic_detect(
            frame=frame, draw_frame=draw_frame, red_lower=red_lower, red_upper=red_upper)
        if red == True:
            print("stay")
        else:
            print("move forward")
    #########################section2#########################
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

    log(frame=draw_frame, lane_mask=line_mask, red_mask=red_mask,
        drop_mask=blue_mask, h_mask=0, ex_frame=0, lane_xy=(lx, ly), lane_angle=line_angle,
        lane_dis=line_x_dis)

    # log(frame=(100, 0, 0), lane_mask=(100, 0, 0), red_mask=(100, 0, 0), drop_mask=(100, 0, 0), h_mask=(100, 0, 0), ex_frame=(100, 0, 0),
    #     show=True, alt=0.0, pitch=0.0, roll=0.0, yaw=0.0,
    #     t_alt=0.0, t_pitch=0.0, t_roll=0.0, t_yaw=0.0,
    #     lane_xy=(0.0, 0.0), lane_angle=0.0, lane_dis=0.0,
    #     target="None", target_xy=(0.0, 0.0), status="None", section=0)
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break
cap.release()
cv2.destroyAllWindows()

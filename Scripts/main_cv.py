import cv2
import numpy as np
import os
import time
from log import log
from detection import *
# from control import *  # 會直接與mavlink連接
from movement import *
import math
section = 0
cap = cv2.VideoCapture(
    "C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\video.mp4")
# cap = cv2.VideoCapture(0)
##############狀態參數##############
drop_cnt = 0
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
# blue h mask
blue_h_lower = np.array([94, 80, 2])
blue_h_upper = np.array([120, 255, 255])
# LOG 錄製
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 指定影像編碼方式
out = cv2.VideoWriter("output"+str(int(time.time())) +
                      ".avi", fourcc, 20.0, (480,  360))
# create white frame
white = np.zeros([120, 160, 3], dtype=np.uint8)
white.fill(255)
# create white line frame
line = cv2.rectangle(white, (60, 0), (100, 120), (0, 0, 0), -1)
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
        hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        out_mask = cv2.inRange(hsvFrame, blue_h_lower, blue_h_upper)
        frame[out_mask > 0] = (255, 255, 255)
        cv2.imshow("frame1", frame)
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h, c_area=1000)
        # def takeoff () return thrust
        # def moveforward
        section = 2

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
        drop, mask_blue, blue_mask, draw_frame, (bx, by), x_distance, y_distance = drop_detect(
            frame, draw_frame, blue_lower=blue_lower, blue_upper=blue_upper)
        mask_blue = cv2.bitwise_not(mask_blue)
        frame[mask_blue > 0] = (255, 255, 255)
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=50)
        number_of_blue_pix = np.sum(mask_blue == 255)
        print("blue pix=" + str(number_of_blue_pix))
        mask_blue = cv2.bitwise_not(mask_blue)
        frame[mask_blue > 0] = (255, 255, 255)
        if (number_of_blue_pix < 18000):
            line_angle = 100000000000
        print(drop_cnt)
        if drop == True:
            drop_cnt += 1
            pitch_angle, roll_angle, yaw_angle, thrust, status = stay(
                x=bx, y=by, current_alt=0, angle=None, current_yaw=0, thrust=0.5)
            if (drop_cnt > 100):
                target = "drop"
                target_xy = (bx, by)
                # 校正位置
                pitch_angle, roll_angle, yaw_angle, thrust, status = stay(
                    x=bx, y=by, current_alt=0, angle=None, current_yaw=0, thrust=0.5)
                # 拋物
                # servo(servo_open=True)
                print("丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟")
        else:
            pitch_angle, roll_angle, yaw_angle, status, thrust = move_forward(
                x=lx, current_alt=0, angle=line_angle, move_pitch_angle=-1, stay_pitch_angle=0, current_yaw=0, thrust=0.5)
    #########################section3#########################
    # if section==3:
    # if section==4:
    ######################### SET ALTITUDE #########################
    back_frame = log(frame=draw_frame, lane_mask=line_mask, red_mask=red_mask,
                     drop_mask=blue_mask, h_mask=0, ex_frame=0, lane_xy=(lx, ly), lane_angle=line_angle,
                     lane_dis=line_x_dis, status=status, section=section, thrust=drop_cnt)

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

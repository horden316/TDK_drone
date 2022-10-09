import cv2
import numpy as np
import os
import time
from log import log
from detection import *
# from control import *  # 會直接與mavlink連接
from movement import *
import math
# cap = cv2.VideoCapture(
#     "C:\\Users\\ericn\\Desktop\\TDK26\\TDK_drone\\video_detect\\video.mp4")
cap = cv2.VideoCapture(0)
##############狀態參數##############
status = None
red_count = 0
thrust = 0
section = 2  # 設定起始階段
status1 = ""
status2 = ""
target = ""
target_xy = (0.0, 0.0)
setAltitude = 0
drop_cnt = 0
##############時間參數##############
red_stay_time = time.time()
section_time = time.time()
##############mask參數##############
# 黑線mask
h = 50
# 紅燈mask
# red_lower = np.array([93, 83, 204], np.uint8)
red_lower = np.array([40, 40, 0], np.uint8)
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
section_time = time.time()
while (1):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (160, 120))
    draw_frame = frame.copy()
    ###################機體狀態###################
    # c_alt = vehicle.rangefinder.distance
    # c_pitch = math.degree(vehicle.attitude.roll)
    # c_roll = math.degree(vehicle.attitude.roll)
    # c_yaw = math.degree(vehicle.attitude.roll)
    c_alt = 0.9
    c_pitch = 0
    c_roll = 0
    c_yaw = 0
    #########################section0#########################
    # 原地起飛
    if section == 0:
        setAltitude = 0.8
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h)
        # def takeoff
        thrust, status1, EM_land = takeoff(current_alt=c_alt, setAltitude=setAltitude,
                                           setThrust=0.52, EM_land_time=10)
        # def moveforward move_pitch_angle = 0
        pitch_angle, roll_angle, yaw_angle, status2, _ = move_forward(
            x=lx, current_alt=c_alt, angle=line_angle, move_pitch_angle=0, stay_pitch_angle=0, current_yaw=c_yaw)
        status = status1 + status2
        if EM_land == True:
            break
        # section 切換時間
        elif ((time.time()-section_time) > 15):
            section_time = time.time()
            section = 2
    #########################section1#########################
    # 直走起飛
    if section == 1:
        setAltitude = 0.8
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h)
        # def takeoff
        thrust, status1, EM_land = takeoff(current_alt=c_alt, setAltitude=setAltitude,
                                           setThrust=0.52, EM_land_time=10)
        # def moveforward move_pitch_angle = 0
        pitch_angle, roll_angle, yaw_angle, status2, _ = move_forward(
            x=lx, current_alt=c_alt, angle=line_angle, move_pitch_angle=-1, stay_pitch_angle=0, current_yaw=c_yaw)
        status = status1 + status2
        if EM_land == True:
            break
        # section 切換時間
        elif ((time.time()-section_time) > 15):
            section_time = time.time()
            section = 2

    #########################section2#########################
    #走線 + 紅燈辨識
    if section == 2:
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=h)
        red, red_mask, draw_frame, (tx, ty), t_x_dis, t_y_dis = traffic_detect(
            frame=frame, draw_frame=draw_frame, red_lower=red_lower, red_upper=red_upper)
        if red == True:
            red_count += 1
            red_stay_time = time.time()
            # print("stay")
            # status = "stay"
            pitch_angle, roll_angle, yaw_angle, thrust, status = stay(
                x=tx, y=ty, current_alt=c_alt, angle=None, current_yaw=c_yaw, thrust=0.5)
            # LOG 資訊
            target = "red light"
            target_xy = (tx, ty)
        # 如果轉false 清除目標值再走
        if (time.time()-red_stay_time) > 0.1:
            status = "stay clear pitch roll"
            pitch_angle = 0
            roll_angle = 0
        # 如果轉false 等1再走
        if (time.time()-red_stay_time) > 3:
            # print("move forward")
            # status = "move forward"
            pitch_angle, roll_angle, yaw_angle, status, thrust = move_forward(
                x=lx, current_alt=c_alt, angle=None, move_pitch_angle=-1, stay_pitch_angle=0, current_yaw=0, thrust=0.5)
            # 跳下個section ###!!!!!!隱患若是偵測到錯的redlight 將會跳下個section 法一:下個section也偵測red
            # 若是辨識過red
            if red_count > 0:
                section = 3
    #########################section3#########################
    # 走線 + 投遞:
    if section == 3:
        (lx, ly), line_angle, line_frame, line_mask, line_x_dis, line_y_dis = line_detect(
            frame=frame, draw_frame=draw_frame, line_mask=50)
        drop, blue_mask, draw_frame, (bx, by), x_distance, y_distance = drop_detect(
            frame, draw_frame, blue_lower=blue_lower, blue_upper=blue_upper)
        print(drop_cnt)
        if drop == True:
            drop_cnt += 1
            pitch_angle, roll_angle, yaw_angle, thrust, status = stay(
                x=bx, y=by, current_alt=c_alt, angle=None, current_yaw=c_yaw, thrust=0.5)
            if (drop_cnt > 1000):
                target = "drop"
                target_xy = (bx, by)
                # 校正位置
                pitch_angle, roll_angle, yaw_angle, thrust, status = stay(
                    x=bx, y=by, current_alt=c_alt, angle=None, current_yaw=c_yaw, thrust=0.5)
                # 拋物
                # servo(servo_open=True)
                print("丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟丟")
        else:
            pitch_angle, roll_angle, yaw_angle, status, thrust = move_forward(
                x=lx, current_alt=c_alt, angle=None, move_pitch_angle=-1, stay_pitch_angle=0, current_yaw=0, thrust=0.5)
    #########################section4#########################
    # 走線 + 降落:
    # if section==4:

    ######################### SET ALTITUDE #########################
    # set_attitude(pitch_angle=pitch_angle, yaw_angle=yaw_angle,
    #              roll_angle=roll_angle, thrust=thrust)
    ######################### LOG #########################
    back_frame = log(frame=draw_frame, lane_mask=line_mask, red_mask=red_mask, drop_mask=blue_mask, h_mask=(100, 0, 0), ex_frame=(100, 0, 0),
                     show=True, alt=c_alt, pitch=c_pitch, roll=c_roll, yaw=c_yaw,
                     t_alt=setAltitude, t_pitch=pitch_angle, t_roll=roll_angle, t_yaw=yaw_angle,
                     lane_xy=(lx, ly), lane_angle=line_angle,
                     target=target, target_xy=target_xy, status=status, section=section, thrust=thrust)
    # log(frame=(100, 0, 0), lane_mask=(100, 0, 0), red_mask=(100, 0, 0), drop_mask=(100, 0, 0), h_mask=(100, 0, 0), ex_frame=(100, 0, 0),
    #     show=True, alt=0.0, pitch=0.0, roll=0.0, yaw=0.0,
    #     t_alt=0.0, t_pitch=0.0, t_roll=0.0, t_yaw=0.0,
    #     lane_xy=(0.0, 0.0), lane_angle=0.0, lane_dis=0.0,
    #     target="None", target_xy=(0.0, 0.0), status="None", section=0, thrust=0)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        print("Key pressed EM land")
        break
    # out.write(back_frame)
# set_attitude(pitch_angle=0, yaw_angle=0, roll_angle=0, thrust=0)
# vehicle.armed = False
print("Mission end")
cap.release()
cv2.destroyAllWindows()

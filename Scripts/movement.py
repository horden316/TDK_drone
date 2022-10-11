# takeoff
# takeoff follow
# stay
# move forward
# landing
from PID import PID
import time
import math
start_EM_land_time = time.time()
target_Altitude_reach = False
X = 160
Y = 120
center_x = int(X/2)
center_y = int(Y/2)
status = ""
pitch_angle = 0
roll_angle = 0
yaw_angle = 0
thrust = 0
EM_land = False


def takeoff(current_alt, setAltitude=0.6, setThrust=0.6, EM_land_time=10):
    global start_EM_land_time
    global target_Altitude_reach
    global EM_land
    global status
    if (time.time() - start_EM_land_time > EM_land_time) and target_Altitude_reach == False:
        EM_land = True
        print("take off timeout: EM_landing!!!!!")
        status = "EM_landing"
        thrust = 0
    if current_alt >= setAltitude:
        thrust = 0.5
        target_Altitude_reach = True
    else:
        status = "takeoff"
        thrust = setThrust
    return thrust, status, EM_land


def stay(x, y, current_alt, angle=None, current_yaw=0, thrust=0.5):
    ######## init ########
    global pitch_angle
    global roll_angle
    global yaw_angle
    global status
    status = "stay"
    x_dis = center_x-x
    y_dis = center_y-y
    # 隨高度降低p值
    PID_p = 0.3
    PID_i = 0.001

    if x > 0:
        # roll PID
        pid_roll = PID(
            Error=x_dis, Kp=PID_p, Ki=PID_i, Kd=0, max_angle=15, a=0.2)
        roll_angle = pid_roll.PID()
        # pitch PID
        pid_pitch = PID(
            Error=y_dis, Kp=PID_p, Ki=PID_i, Kd=0, max_angle=15, a=0.2)
        pitch_angle = pid_pitch.PID()
    # yaw_angle######### 可能有BUG
    if angle is None:
        yaw_angle = current_yaw
    else:
        if angle > 0:
            theta = 90 - angle
            yaw_angle = current_yaw-theta
            if (yaw_angle > 180):
                yaw_angle = -180 + (yaw_angle-180)
            if (yaw_angle < -180):
                yaw_angle = 180 + (yaw_angle+180)
            print("set:"+str(yaw_angle))
            print("yaw right")
            pitch_angle = 0
        elif angle < 0:
            theta = 90 + angle
            yaw_angle = current_yaw+theta
            if (yaw_angle > 180):
                yaw_angle = -180 + (yaw_angle-180)
            if (yaw_angle < -180):
                yaw_angle = 180 + (yaw_angle+180)
            print("set:"+str(yaw_angle))
            print("yaw left")
            pitch_angle = 0
        # else:
        #     yaw_angle = current_yaw
    return pitch_angle, roll_angle, yaw_angle, thrust, status


def move_forward(x, current_alt, angle=None, move_pitch_angle=-1, stay_pitch_angle=0, current_yaw=0, thrust=0.5, alpha=0):
    ######## init ########
    global pitch_angle
    global roll_angle
    global yaw_angle
    global status
    status = "move_forward"
    x_dis = center_x-x

    # 隨高度降低p值
    if current_alt < 0.4:
        PID_p = 0.0
        PID_i = 0.0
    else:
        PID_p = 0.23
        PID_i = 0.001

    if x > 0:
        # roll PID
        pid_roll = PID(
            Error=x_dis, Kp=PID_p, Ki=PID_i, Kd=0, max_angle=15, a=0.2)
        roll_angle = pid_roll.PID()
    #########yaw_angle#########
    if angle is None:
        yaw_angle = current_yaw
    else:
        if angle > 0:
            theta = 90 - angle + alpha
            yaw_angle = current_yaw-theta
            if (yaw_angle > 180):
                yaw_angle = -180 + (yaw_angle-180)
            if (yaw_angle < -180):
                yaw_angle = 180 + (yaw_angle+180)
            print("set:"+str(yaw_angle))
            print("yaw right")
        elif angle < 0:
            theta = 90 + angle
            yaw_angle = current_yaw+theta
            if (yaw_angle > 180):
                yaw_angle = -180 + (yaw_angle-180)
            if (yaw_angle < -180):
                yaw_angle = 180 + (yaw_angle+180)
            print("set:"+str(yaw_angle))
            print("yaw left")
        # else:
        #     yaw_angle = current_yaw

        if x == 0 and angle == 0:
            print("I don't see the line")
            pitch_angle = stay_pitch_angle
        if (angle > 80 or angle < -80) and x_dis < 15:
            print("Pitch Forward")
            pitch_angle = move_pitch_angle
        else:
            print("Pitch Forward")
            pitch_angle = stay_pitch_angle
    return pitch_angle, roll_angle, yaw_angle, status, thrust


def landing(x, y, current_alt, thrust=0.4):
    ######## init ########
    global pitch_angle
    global roll_angle
    global yaw_angle
    global status
    x_dis = center_x-x
    y_dis = center_y-y
    # 隨高度降低p值
    if current_alt < 0.5:
        PID_p = 0.35
    else:
        PID_p = 0.3

    if x > 0:
        # roll PID
        pid_roll = PID(
            Error=x_dis, Kp=PID_p, Ki=0, Kd=0, max_angle=15, a=0.2)
        roll_angle = pid_roll.PID()
        pid_pitch = PID(
            Error=y_dis, Kp=PID_p, Ki=0, Kd=0, max_angle=15, a=0.2)
        pitch_angle = pid_pitch.PID()
    if current_alt <= 0.31:
        thrust = 0
    return pitch_angle, roll_angle, thrust

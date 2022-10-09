# takeoff
# takeoff follow
# stay
# move forward
# landing
from PID import PID
import time

start_EM_land_time = time.time()
target_Altitude_reach = False
X = 160
Y = 120
center_x = int(X/2)
center_y = int(Y/2)


def takeoff(current_alt, setAltitude, setThrust=0.52, EM_land_time=10):
    global start_EM_land_time
    global target_Altitude_reach
    if (time.time() - start_EM_land_time > EM_land_time) and target_Altitude_reach == False:
        print("take off timeout: EM_landing!!!!!")
        status = "EM_landing!!!!!"
        thrust = 0
    if current_alt >= setAltitude:
        thrust = 0.5
        target_Altitude_reach = True
    else:
        thrust = setThrust
    return thrust


def stay(x, y, angle, current_yaw, thrust=0.5):
    x_dis = center_x-x
    y_dis = center_y-y
    pid_roll = PID(
        Error=x_dis, Kp=0.3, Ki=0, Kd=0, max_angle=15, a=0.2)
    roll_angle = pid_roll.PID()
    pid_pitch = PID(
        Error=y_dis, Kp=0.3, Ki=0, Kd=0, max_angle=15, a=0.2)
    pitch_angle = pid_pitch.PID()
    #########yaw_angle#########

    return pitch_angle, roll_angle, yaw_angle, thrust


def move_forward(x, angle, current_yaw, duration):
    x_dis = center_x-x
    pid_roll = PID(
        Error=x_dis, Kp=0.3, Ki=0, Kd=0, max_angle=15, a=0.2)
    roll_angle = pid_roll.PID()
    #########yaw_angle#########

    return


def landing(x_dis, y_dis, thrust=0):

    return

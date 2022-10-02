# add PID control to follow.py
from curses import KEY_PPAGE
from msilib.schema import Error
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math


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


def arm():
    print("Basic pre-arm checks")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
    print("armed")


def arm_and_takeoff_nogps(aTargetAltitude, DEFAULT_TAKEOFF_THRUST=0.55, SMOOTH_TAKEOFF_THRUST=0.55, limit_time=10, default_yaw=True):
    if default_yaw is True:
        yawangle = math.degrees(vehicle.attitude.yaw)
    else:
        yawangle = 0
    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    start = time.time()
    while True:
        current_altitude = vehicle.rangefinder.distance
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if time.time() - start > limit_time:
            print("take off timeout")
            print("change mode to landing")
            vehicle.mode = VehicleMode("LAND")
            time.sleep(1)
            while True:
                time.sleep(1)
                print("vehicle emergency landing: open controller")
        if current_altitude >= aTargetAltitude:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.7:
            thrust = SMOOTH_TAKEOFF_THRUST
            print("thrust set to SMOOTH")
        set_attitude(yaw_angle=yawangle, thrust=thrust)
        time.sleep(0.2)


def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)


def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, duration=0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)

    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)

    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         None, 0, True,
                         thrust)


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def distanceCalculate(p1, p2):
    """p1 and p2 in format (x1,y1) and (x2,y2) tuples"""
    dis = ((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2) ** 0.5
    return dis


def PID(Error=0, Kp=0.8, Ki=0, Kd=0, max_angle=15, a=0.2):
    total_Err = total_Err + Error
    output = -(Kp*Error + Ki*total_Err + Kd * (Error - last_Err))
    last_Err = Error
    pid_angle = output*a
    if pid_angle > max_angle:
        pid_angle = max_angle
    if pid_angle < -max_angle:
        pid_angle = -max_angle
    return pid_angle


##############主程式##############
time.sleep(2)
print("takeoff")
yawangle = math.degrees(vehicle.attitude.yaw)
arm_and_takeoff_nogps(aTargetAltitude=0.5, DEFAULT_TAKEOFF_THRUST=0.55,
                      SMOOTH_TAKEOFF_THRUST=0.52, limit_time=10, default_yaw=True)
# 起飛後直行一段時間
set_attitude(yaw_angle=yawangle, pitch_angle=-5, thrust=0.5, duration=2)

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
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
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
<<<<<<< HEAD
            x_distance = center[0]-cx
            roll_angle = PID(Error=x_distance, Kp=0.8, Ki=0,
                             Kd=0, max_angle=15, a=0.2)
            #pitch_angle=PID(Error = x_distance, Kp=0.8, Ki=0, Kd=0,max_angle=15,a=0.2)
            # Error = x_distance
            # total_Err = total_Err +Error
            # output = -(Kp*Error + Ki*total_Err + Kd* (Error - last_Err))
            # last_Error = Error
            # u = output
            # roll_angle= u*0.2
            # if roll_angle > 15: roll_angle =15
            # if roll_angle < -15: roll_angle =-15
    # yaw調整(yaw_angle)1絕對調整
            if angle > 0:
=======
            x_distance=center[0]-cx
            Error = x_distance
            total_Err = total_Err +Error
            output = -(Kp*Error + Ki*total_Err + Kd* (Error - last_Err))
            last_Error = Error
            u = output
            roll_angle= u*0.2
            if roll_angle > 15: roll_angle =15
            if roll_angle < -15: roll_angle =-15
            # if x_distance > 0 :
            #     roll_angle = -5
            #     print("Roll left")
            #     WriteText(frame2, "Roll left", 2)
            # elif x_distance < 0 :
            #     roll_angle = 5
            #     print("Roll right")
            #     WriteText(frame2, "Roll right", 2)
            # else:
            #     pitch_angle = -5
            #     print("Pitch Forward")
            #     WriteText(frame2, "Pitch Forward", 2)
    #yaw調整(yaw_angle)1絕對調整
            if angle > 0 :
>>>>>>> parent of ef674b3 (Update follow.py)
                theta = 90 - angle
                yaw_angle = yawangle-theta
                print("current_yaw:"+str(math.degrees(vehicle.attitude.yaw)))
                WriteText(frame2, "current_yaw:" +
                          str(math.degrees(vehicle.attitude.yaw)), 4)
                print("set:"+str(yawangle-theta))
                WriteText(frame2, "set:"+str(yawangle-theta), 3)
                print("yaw right")
                WriteText(frame2, "yaw right", 5)
            elif angle <= 0:
                theta = 90 + angle
                yaw_angle = yawangle+theta
                print("current_yaw:"+str(math.degrees(vehicle.attitude.yaw)))
                WriteText(frame2, "current_yaw:" +
                          str(math.degrees(vehicle.attitude.yaw)), 4)
                print("set:"+str(yawangle+theta))
                WriteText(frame2, "set:"+str(yawangle+theta), 3)
                print("yaw left")
                WriteText(frame2, "yaw left", 5)
            else:
                print("Pitch Forward")
                WriteText(frame2, "Pitch Forward", 5)
                print("current_yaw:"+str(math.degrees(vehicle.attitude.yaw)))
                WriteText(frame2, "current_yaw:" +
                          str(math.degrees(vehicle.attitude.yaw)), 4)
    # pitch(直走)觸發條件
            if theta > -15 and theta < 15 and x_distance > -30 and x_distance < 30:
                pitch_angle = -5
            else:
                pitch_angle = 0

            ###########################送出set_altitude 指令###########################
            set_attitude(pitch_angle=pitch_angle, yaw_angle=yaw_angle,
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

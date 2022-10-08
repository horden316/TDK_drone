import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import math
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 指定影像編碼方式
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (480,  360))
blank_width = 480
blank_height = 360
text_yspace = 15


def log(frame=(100, 0, 0), lane_mask=(100, 0, 0), red_mask=(100, 0, 0), drop_mask=(100, 0, 0), h_mask=(100, 0, 0), ex_frame=(100, 0, 0),
        show=True, alt=0.0, pitch=0.0, roll=0.0, yaw=0.0,
        t_alt=0.0, t_pitch=0.0, t_roll=0.0, t_yaw=0.0,
        lane_xy=(0.0, 0.0), lane_angle=0.0, lane_dis=0.0,
        target="None", target_xy=(0.0, 0.0), status="None", section=0,):
    h, w, _ = frame.shape
    back_frame = np.zeros((blank_height, blank_width, 3), np.uint8)
    # lane_mask = cv2.merge((lane_mask, lane_mask, lane_mask))
    # red_mask = cv2.merge((red_mask, red_mask, red_mask))
    # drop_mask = cv2.merge((drop_mask, drop_mask, drop_mask))
    # h_mask = cv2.merge((h_mask, h_mask, h_mask))
    # ex_frame = cv2.merge((ex_frame, ex_frame, ex_frame))
    back_frame[0:h, 0:w] = frame
    back_frame[0:h, 160:160+w] = lane_mask
    back_frame[0:h, 320:320+w] = red_mask
    back_frame[120:120+h, 0:w] = drop_mask
    back_frame[120:120+h, 160:160+w] = h_mask
    back_frame[120:120+h, 320:320+w] = ex_frame
    cv2.putText(back_frame, "alt:"+str(alt)+"/"+str(t_alt), (0, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "pitch:"+str(pitch)+"/"+str(pitch), (0, 260+text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "roll:"+str(roll)+"/"+str(roll), (0, 260+2*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "yaw:"+str(yaw)+"/"+str(yaw), (0, 260+3*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (201, 194, 9), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "Detection:", (0, 260+4*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "Lane:"+"(x,y)="+str(lane_xy)+"  angle="+str(lane_angle), (0, 260+5*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "Target:"+str(target)+"(x,y)="+str(target_xy), (0, 260+6*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "Status:"+str(status), (240, 260),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150, 255), 1, cv2.LINE_AA)
    cv2.putText(back_frame, "Section:"+str(section), (240, 260+4*text_yspace),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150, 255), 1, cv2.LINE_AA)
    if show == True:
        cv2.imshow("Frame", back_frame)


#####################testing code#####################
if __name__ == "__main__":
    X = 160
    Y = 120
    cap = cv2.VideoCapture(0)
    cap.set(3, X)
    cap.set(4, Y)
    while True:
        ret, frame = cap.read()
        low_b = np.uint8([255, 255, 255])
        high_b = np.uint8([50, 50, 50])
        mask = cv2.inRange(frame, high_b, low_b)
        remask = cv2.bitwise_not(mask)
        remask = cv2.merge((remask, remask, remask))
        log(frame=frame, lane_mask=remask, red_mask=remask,
            drop_mask=remask, h_mask=remask)
        if cv2.waitKey(1) & 0xFF == ord(' '):
            break
    cap.release()
    cv2.destroyAllWindows()

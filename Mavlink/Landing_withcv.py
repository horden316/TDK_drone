from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math
#連線至無人機
connection_string = 'tcp:192.168.4.1:1234'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=115200)

#####################函式#####################
def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)
    
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)

def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
        x_m,          # x	float	X Position of the landing target on MAV_FRAME
        y_m,          # y	float	Y Position of the landing target on MAV_FRAME
        z_m,          # z	float	Z Position of the landing target on MAV_FRAME
        (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
        2,          # type of landing target: 2 = Fiducial marker
        1,          # position_valid boolean
    )
    print (msg)
    vehicle.send_mavlink(msg)

    
def send_land_message_v1(x_rad=0, y_rad=0, dist_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
    )
    print (msg)
    vehicle.send_mavlink(msg)
  
        
# Define function to send distance_message mavlink message for mavlink based rangefinder, must be >10hz
# http://mavlink.org/messages/common#DISTANCE_SENSOR
def send_distance_message( dist):
    msg = vehicle.message_factory.distance_sensor_encode(
        0,          # time since system boot, not used
        1,          # min distance cm
        10000,      # max distance cm
        dist,       # current distance, must be int
        0,          # type = laser?
        0,          # onboard id, not used
        mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
        0           # covariance, not used
    )
    vehicle.send_mavlink(msg)     

def set_land_target(x_cm,y_cm,freq):#in meters
    z_cm = vehicle.rangefinder.distance*100
    angle_x, angle_y = marker_position_to_angle(x_cm,y_cm, z_cm)
    if time.time() >= time_start + 1.0/freq:
            time_start = time.time()
            send_land_message_v2(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
    print ("Target found x = %5.0f cm  y = %5.0f cm -> angle_x = %5f  angle_y = %5f"%(x_cm, y_cm, angle_x, angle_y))
#####################主程式#####################
while True:
    ############cv detection code############
    
    land_trg = True
    if land_trg: 
        print("Target found: send_land_target")
        set_land_target(x_cm=,y_cm=,freq=15)








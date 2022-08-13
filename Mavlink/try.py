#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 10 15:19:24 2022

@author: horden
"""
import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/cu.usbmodem141101")

#the_connection = mavutil.mavlink_connection("tcp:192.168.4.1:6789")

#the_connection.reboot_autopilot()


the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))


the_connection.arducopter_arm()

#測試ARM的
print("Waiting for the vehicle to arm")
the_connection.motors_armed_wait()
print('Armed!')

#Takeoff
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 1, 0, 0, 0, 0, 0, 0)

#33 for GLOBAL_POSITION_INT
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 33, 0, 0, 0, 0, 0, 0)





'''


#command_long_send(target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7) 他寫在message #76的格式

#Takeoff指令
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


#ARM
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


#測試ARM的
print("Waiting for the vehicle to arm")
the_connection.motors_armed_wait()
print('Armed!')




while True:
    msg = the_connection.recv_match(
        type='LOCAL_POSITION_NED', blocking=True)
    print(msg)

#用GPS取得的高度（但其實裡面很多跟高度無關的值？）
while True:
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    print(msg)



# Request all parameters
the_connection.mav.param_request_list_send(
    the_connection.target_system, the_connection.target_component
)
while True:
    time.sleep(0.01)
    try:
        message = the_connection.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
        print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
    except Exception as error:
        print(error)




#把整個recv_match讀到的.to_dict()後篩出yaw
while True:
    try:
        infordict = the_connection.recv_match().to_dict()
        print(infordict["yaw"])

    except:
        pass
    


#看起來不能這樣玩
while True:
    yaw=the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    yaw = the_connection.messages['MAV_SYS_STATUS_SENSOR_YAW_POSITION'].alt
    print(yaw)



try:
    heightref = the_connection.messages['MAV_ODID_HEIGHT_REF'].alt
    print(heightref)
except:
    print('No USB connected')


try:
    conn = the_connection.messages['AV_POWER_STATUS_USB_CONNECTED'].alt
    print(conn)
except:
    print('No USB connected')


try: 
    altitude = the_connection.messages['GPS_RAW_INT'].alt
    timestamp = the_connection.time_since('GPS_RAW_INT')
except:
    print('No GPS_RAW_INT message received')
    
    


'''
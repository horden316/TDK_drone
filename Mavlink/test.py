#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 10 15:19:24 2022

@author: horden
"""
import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("com3")
#the_connection = mavutil.mavlink_connection('tcp:192.168.4.1:1234')

#the_connection.reboot_autopilot()

#mavutil.mavfile.wait_heartbeat()
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
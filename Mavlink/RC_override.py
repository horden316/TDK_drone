#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: horden
"""
import time
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/cu.usbmodem142401")

#the_connection = mavutil.mavlink_connection("tcp:192.168.4.1:6789")

#the_connection.reboot_autopilot()


the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))

#ARM
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)


def set_rc_channel_pwm(channel_id, pwm):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)] #開一個18個欄位，值預設都是65535的list
    rc_channel_values[channel_id - 1] = pwm
    the_connection.mav.rc_channels_override_send(
        the_connection.target_system,                # target_system
        the_connection.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

StartTime=time.time()
while (time.time()-StartTime<5):
    set_rc_channel_pwm(3, 1600) #(channel多少,PWM)
    print(str(time.time())+"THROTTLE1600")

StartTime2=time.time()
while (time.time()-StartTime2<10):
    set_rc_channel_pwm(3, 1600) #(channel多少,PWM)
    set_rc_channel_pwm(5, 1001) #(channel多少,PWM)
    print(str(time.time())+"Stable")

StartTime3=time.time()
while (time.time()-StartTime3<15):
    set_rc_channel_pwm(3, 1600) #(channel多少,PWM)
    set_rc_channel_pwm(5, 2001) #(channel多少,PWM)
    print(str(time.time())
    +"Land")
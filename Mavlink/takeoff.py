from pymavlink import mavutil
import RPi.GPIO as GPIO

channel1=1000
channel2=1000
channel3=1000
channel4=1000

the_connection = mavutil.mavlink_connection("/dev/cu.usbmodem01")
px4flow = mavutil.mavlink_connection("/dev/tty.usbmodem144401")



the_connection.wait_heartbeat()
print("board:Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))

px4flow.wait_heartbeat()
print("px4flow:Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))



the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

while px4flow.recv_match(type='OPTICAL_FLOW', blocking=True).ground_distance<1:
    channel1+=1;
else:
    channel1-=1;
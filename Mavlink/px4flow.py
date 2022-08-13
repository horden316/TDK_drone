from pymavlink import mavutil

the_connection = mavutil.mavlink_connection("/dev/tty.usbmodem142401")


the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))

while True:
    msg = the_connection.recv_match(type='OPTICAL_FLOW', blocking=True)
    #print("ground_distance: "+ str(msg.ground_distance))
    print(msg)
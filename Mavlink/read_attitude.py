from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = 'tcp:192.168.4.1:1234'
sitl = None

# Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=115200)

while True:
    print("Pitch: %f \n"%vehicle.attitude.pitch)
    print("Pitch: %f \n"%vehicle.attitude.pitch)
    print("Pitch: %f \n"%vehicle.attitude.pitch)
    print("Altitude: %f \n" %vehicle.rangefinder.distance)
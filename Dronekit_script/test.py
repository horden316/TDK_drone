from dronekit import connect, VehicleMode, LocationGLobalRelative,APIException
import time
import socket
import exceptions

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    args = parser.parse_args

    connection_string = args.connect
    baud_rate=115200
    
    vehicle =connect(connection_string,baud=baud_rate,wait_ready=True)
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable..")
        time.sleep(1)
    print("now armable")
    print("")
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for drone to become armed..")
        time.sleep(1)
    print("Vehicle is now armed")
    print("props spinning")
    return None

    vehicle =connectMyCopter()
    arm()
    print("End of script")


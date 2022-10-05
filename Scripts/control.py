from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil  # Needed for command message definitions
import time
import math


class control():
    def __init__(self):
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200)

    def arm(self):
        print("Basic pre-arm checks")
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)
        print("armed")

    def arm_and_takeoff_nogps(self, aTargetAltitude, DEFAULT_TAKEOFF_THRUST=0.55, SMOOTH_TAKEOFF_THRUST=0.55, limit_time=10, default_yaw=True):
        if default_yaw is True:
            yawangle = math.degrees(self.vehicle.attitude.yaw)
        else:
            yawangle = 0
        print("Arming motors")
        # Copter should arm in GUIDED_NOGPS mode
        self.vehicle.mode = VehicleMode("GUIDED_NOGPS")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            self.vehicle.armed = True
            time.sleep(1)

        print("Taking off!")

        thrust = DEFAULT_TAKEOFF_THRUST
        start = time.time()
        while True:
            current_altitude = self.vehicle.rangefinder.distance
            print(" Altitude: %f  Desired: %f" %
                  (current_altitude, aTargetAltitude))
            if time.time() - start > limit_time:
                print("take off timeout")
                print("change mode to landing")
                self.vehicle.mode = VehicleMode("LAND")
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
            self.set_attitude(yaw_angle=yawangle, thrust=thrust)
            time.sleep(0.2)

    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
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

    def send_attitude_target(self, roll_angle=0.0, pitch_angle=0.0,
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
            yaw_angle = self.vehicle.attitude.yaw
        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # Target system
            1,  # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll_angle, pitch_angle,
                               yaw_angle),  # Quaternion
            0,  # Body roll rate in radian
            0,  # Body pitch rate in radian
            math.radians(yaw_rate),  # Body yaw rate in radian/second
            thrust  # Thrust
        )
        self.vehicle.send_mavlink(msg)

    def set_attitude(self, roll_angle=0.0, pitch_angle=0.0,
                     yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                     thrust=0.5, duration=0):
        """
        Note that from AC3.3 the message should be re-sent more often than every
        second, as an ATTITUDE_TARGET order has a timeout of 1s.
        In AC3.2.1 and earlier the specified attitude persists until it is canceled.
        The code below should work on either version.
        Sending the message multiple times is the recommended way.
        """
        self.send_attitude_target(roll_angle, pitch_angle,
                                  yaw_angle, yaw_rate, False,
                                  thrust)

        start = time.time()
        while time.time() - start < duration:
            self.send_attitude_target(roll_angle, pitch_angle,
                                      yaw_angle, yaw_rate, False,
                                      thrust)
            time.sleep(0.1)

        # Reset attitude, or it will persist for 1s more due to the timeout
        self.send_attitude_target(0, 0,
                                  None, 0, True,
                                  thrust)


class PID():
    def __init__(self, Error=0, Kp=0.8, Ki=0, Kd=0, max_angle=15, a=0.2):
        self.Error = Error
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_angle = max_angle
        self.a = a
        self.total_Err = 0
        self.last_Err = 0

    def PID(self):
        self.total_Err = self.total_Err + self.Error
        output = -(self.Kp*self.Error + self.Ki*self.total_Err +
                   self.Kd * (self.Error - self.last_Err))
        self.last_Err = self.Error
        pid_angle = output*self.a
        if pid_angle > self.max_angle:
            pid_angle = self.max_angle
        if pid_angle < -self.max_angle:
            pid_angle = -self.max_angle
        return pid_angle

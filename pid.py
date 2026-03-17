class PID:
    def __init__(self, Kp, Ki=0, Kd=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def regulate(self, error, dt):
        res = error * self.Kp
        return res

class PID:
    def __init__(self, ref, Kp, Ki=0, Kd=0):
        self.ref = ref
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.sum = 0
        self.prev = 0

    def regulate(self, error, dt):
        self.sum += error * dt
        P = error * self.Kp
        I = self.sum * self.Ki
        D = (error - self.prev) / dt * self.Kd
        res = P + I + D
        self.prev = error
        return res

    def reset(self):
        self.sum = 0
        self.prev = 0

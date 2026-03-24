class PID:
    def __init__(self, r0, T, Ti, Td):
        self.q0 = -r0 * (1 + T / (2 * Ti) + Td / T)
        self.q1 = r0 * (1 - T / (2 * Ti) + 2 * Td / T)
        self.q2 = -r0 * Td / T
        self.prev = 0
        self.prev_prev = 0
        self.prev_u = 0

    def regulate(self, cur, ref):
        res = self.prev_u - (self.q0 + self.q1 + self.q2) * ref + \
            self.q0 * cur + self.q1 * self.prev + self.q2 * self.prev_prev
        self.prev_prev = self.prev
        self.prev = cur
        self.prev_u = res
        return res

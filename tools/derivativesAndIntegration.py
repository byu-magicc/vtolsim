
#implements the dirty derivative class, to get the dirty derivative for a particular function
class DirtyDerivative:
    def __init__(self, Ts, sigma):
        self.Ts = Ts
        self.sigma = sigma
        self.a1 = (2.0 * sigma - Ts) / (2.0 * sigma + Ts)
        self.a2 = 2.0 / (2.0 * sigma+ Ts)
        self.initialized = False

    def update(self, z):
        if self.initialized is False:
            self.z_dot = 0 * z
            self.z_delay_1 = z
            self.initialized = True
        else:
            self.z_dot = self.a1 * self.z_dot \
                         + self.a2 * (z - self.z_delay_1)
            self.z_delay_1 = z
        return self.z_dot
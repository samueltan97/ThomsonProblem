import numpy as np

class Particle:
    def __init__(self, dt, mass=1, charge=1, force=[np.zeros(3)]):
        self.mass = mass
        self.charge = charge
        self.force = force  # net force on the particle
        self.dt = dt

        self.pos = None
        self.vel = [np.zeros(np.shape(force))]
        self.accel = [np.zeros(np.shape(force))]

    def update(self):
        # self.accel = np.append(self.accel, self.force[-1] / self.mass)
        self.accel.append(self.force[-1] / self.mass)

        # self.vel = np.append(self.vel, self.vel[-1] + self.accel[-1] * self.dt)
        self.vel.append(self.vel[-1] + self.accel[-1] * self.dt)

        # self.pos = np.append(self.pos, self.pos[-1]+self.vel[-1]*self.dt)
        self.pos.append(self.pos[-1]+self.vel[-1]*self.dt)


import numpy as np

class Particle:
    def __init__(self, dt, mass=1, charge=1, force=[np.zeros(3)]):
        self.mass = mass
        self.charge = charge
        self.force = force  # net force on the particle
        self.dt = dt
        self.pos = [np.zeros(3)]
        self.vel = [np.zeros(3)]
        self.accel = [np.zeros(3)]

    def update(self):
        self.accel = np.vstack((self.accel, self.force[-1] / self.mass))
        self.vel = np.vstack((self.vel, self.vel[-1] + self.accel[-1] * self.dt))
        new_pos = self.pos[-1] + self.vel[-1] * self.dt
        p = (new_pos/np.linalg.norm(new_pos))*1.01
        self.pos = np.vstack((self.pos, p))

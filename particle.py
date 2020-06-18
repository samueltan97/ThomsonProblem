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

    def velocity_constraint(self, vel, radius):
        vel = vel - np.dot(vel, radius)*radius/np.linalg.norm(radius)
        return vel

    def position_constraint(self, pos):
        pos = 1.01 * pos / (np.linalg.norm(pos))
        return pos

    def update(self):

        self.accel = np.vstack((self.accel, self.force[-1] / self.mass))
        radius = self.pos[-1]

        new_vel = self.vel[-1] + self.accel[-1]*self.dt
        new_vel = self.velocity_constraint(new_vel, radius)
        self.vel = np.vstack((self.vel, new_vel))

        new_pos = self.pos[-1] + (self.vel[-1] * self.dt)
        new_pos = self.position_constraint(new_pos)
        self.pos = np.vstack((self.pos, new_pos))

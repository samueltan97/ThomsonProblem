import numpy as np

class Particle:
    def __init__(self, timesteps, mass=1, charge=1, force=np.zeros(timesteps.size), dt=timesteps.size / 1000):
        self.mass = mass
        self.charge = charge
        self.timesteps = timesteps
        self.force = force
        self.dt = dt

        # TODO instead of fixing the size of the numpy arrays, we can keep it open-ended and
        # TODO just append the values to the end of it. We might have to create x, y, and z for each though
        self.pos = np.zeros((self.timesteps.size, 3))
        self.vel = np.zeros((self.timesteps.size, 3))
        self.accel = np.zeros((self.timesteps.size, 3))
        self.force = np.zeros((self.timesteps.size, 3))  # net force on the particle

    def update(self):
        self.accel - self.force / self.mass
        self.vel = self.accel * self.dt
        self.pos = self.vel * self.dt
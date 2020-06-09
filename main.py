from threading import Thread
from time import sleep
import numpy as np
import sys
from particle import Particle
class MainCycle:

    def __init__(self, particle_count, delta_t):
        self.particle_count = int(particle_count)
        self.delta_t = float(delta_t)
        self.make_particle_list()

    def call_at_interval(self, period, callback, args):
        while True:
            sleep(period)
            callback(*args)

    def set_interval(self, period, callback, *args):
        Thread(target=self.call_at_interval, args=(period, callback, args)).start()

    def make_particle_list(self):  # makes list of N particles
        particle_list = []
        for i in range(int(self.particle_count)):
            particle_list.append(Particle(self.delta_t))
        self.particle_list = particle_list

    # TODO revise the positions that we set up
    def set_positions(self):  # sets INITIAL positions of particles as (1,0,0) , (2,0,0) ...
        for i in range(len(self.particle_list)):
            self.particle_list[i].pos = [np.array([i/10, 0, 0])]
        print("first position", self.particle_list[0].pos)


    # TODO I changed the way it iterates by updating every force to make it bidirectional. Less iterations ftw
    def calc_forces(self, particle_list):  # calcs forces between particles
        for i in range(len(particle_list)):
            for j in range(i+1, len(particle_list)):
                sep = particle_list[i].pos[-1] - particle_list[j].pos[-1]
                particle_list[i].force = np.append(particle_list[i].force, particle_list[i].force[-1] + sep / (np.linalg.norm(sep)) ** 3)
                particle_list[j].force = np.append(particle_list[j].force, particle_list[i].force[-1] - sep / (np.linalg.norm(sep)) ** 3)

        print(particle_list[0].force[-1], particle_list[1].force[-1])

    def iterate_cycle(self, particle_count):
        self.calc_forces(self.particle_list)
        for i in range(particle_count):
            self.particle_list[i].update()

    def start_cycle(self):
        self.set_positions()
        self.set_interval(self.delta_t, self.iterate_cycle, self.particle_count)


if __name__ == "__main__":
    # first arg will be number of particles and second arg will be delta T in seconds
    MainCycle(sys.argv[1], sys.argv[2]).start_cycle()
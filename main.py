from threading import Thread
from time import sleep
import numpy as np
import sys
from particle import Particle
from mayavi import mlab

phi = np.linspace(0, 2*np.pi, 100)
theta = np.linspace(0, np.pi, 100)

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
        mlab.gcf().scene.parallel_projection = False
        mlab.show()

    def make_particle_list(self):  # makes list of N particles
        particle_list = []
        for i in range(int(self.particle_count)):
            particle_list.append(Particle(self.delta_t))
        self.particle_list = particle_list

    # TODO revise the positions that we set up
    def set_positions(self):  # sets INITIAL positions of particles as (1,0,0) , (2,0,0) ...
        for i in range(len(self.particle_list)):
            set_pos = [np.random.rand(3)]
            self.particle_list[i].pos = (set_pos/np.linalg.norm(set_pos))*1.01
        #print(np.linalg.norm(self.particle_list[0].pos), "  ", np.linalg.norm(self.particle_list[1].pos))


    def plot_sphere(self):
        x = 1 * np.outer(np.cos(phi), np.sin(theta))
        y = 1 * np.outer(np.sin(phi), np.sin(theta))
        z = 1 * np.outer(np.ones(np.size(phi)), np.cos(theta))
        mlab.mesh(x, y, z, colormap="Spectral")

    #TODO: fix initial positions
    def plot_particles(self):
        particle_plots = []
        for i in range(len(self.particle_list)):
            x = 0.05 * np.outer(np.cos(phi), np.sin(theta)) + self.particle_list[i].pos[0][0]
            y = 0.05 * np.outer(np.sin(phi), np.sin(theta)) + self.particle_list[i].pos[0][1]
            z = 0.05 * np.outer(np.ones(np.size(phi)), np.cos(theta)) + self.particle_list[i].pos[0][2]
            particle_plots.append(mlab.mesh(x, y, z, colormap="autumn"))
        self.particle_plots = particle_plots

    # TODO I changed the way it iterates by updating every force to make it bidirectional. Less iterations ftw
    def calc_forces(self, particle_list):  # calcs forces between particles
        for i in range(len(particle_list)):
            for j in range(i+1, len(particle_list)):
                sep = particle_list[i].pos[-1] - particle_list[j].pos[-1]
                i_radius = particle_list[i].pos[-1]
                j_radius = particle_list[j].pos[-1]
                i_force = sep / (np.linalg.norm(sep)) ** 3 - np.dot(sep/(np.linalg.norm(sep) ** 3), i_radius)*(i_radius/np.linalg.norm(i_radius))
                j_force = -sep / (np.linalg.norm(sep)) ** 3 - np.dot(-sep/(np.linalg.norm(sep) ** 3), j_radius)*(j_radius/np.linalg.norm(j_radius))
                particle_list[i].force = np.vstack((particle_list[i].force, i_force))
                particle_list[j].force = np.vstack((particle_list[j].force, j_force))

    def update_plot(self):
        for i in range(len(self.particle_list)):
            x = 0.05 * np.outer(np.cos(phi), np.sin(theta)) + self.particle_list[i].pos[-1][0]
            y = 0.05 * np.outer(np.sin(phi), np.sin(theta)) + self.particle_list[i].pos[-1][1]
            z = 0.05 * np.outer(np.ones(np.size(phi)), np.cos(theta)) + self.particle_list[i].pos[-1][2]
            self.particle_plots[i].mlab_source.trait_set(x=x, y=y, z=z)
        #print(np.linalg.norm(self.particle_list[0].pos), "  ", np.linalg.norm(self.particle_list[1].pos))

    def iterate_cycle(self, particle_count):
        self.calc_forces(self.particle_list)
        for i in range(particle_count):
            self.particle_list[i].update()
        #print(self.particle_list[0].pos[-1], "  ", self.particle_list[1].pos[-1])
        print(np.linalg.norm(self.particle_list[0].pos[-1]), "  ", np.linalg.norm(self.particle_list[1].pos[-1]))
        self.update_plot()

    def start_cycle(self):
        self.set_positions()
        self.plot_sphere()
        self.plot_particles()
        self.set_interval(self.delta_t, self.iterate_cycle, self.particle_count)


if __name__ == "__main__":
    # first arg will be number of particles and second arg will be delta T in seconds
    MainCycle(sys.argv[1], sys.argv[2]).start_cycle()

# -*- coding: utf-8 -*-

# pragma: no cover
__author__ = 'Benjamin Alt, Felix Schneider and Jacqueline Rutschke'
#https://github.com/Scaatis/hbpprak_perception/
# not exactly what Benjamin and Felix did but very close to it

from hbp_nrp_cle.brainsim import simulator as sim
import numpy as np


resolution = 17
n_motors = 4  # down, up, left, right

sensors = sim.Population(resolution * resolution, cellclass=sim.IF_curr_exp())
down, up, left, right = [sim.Population(1, cellclass=sim.IF_curr_exp()) for _ in range(n_motors)]

indices = np.arange(resolution * resolution).reshape((resolution, resolution))
ones = np.ones(shape=((resolution // 2) * resolution,1))

# upper half = eight rows from the top
upper_half = sim.PopulationView(sensors, indices[:resolution // 2].flatten())
# lower half = eight rows from bottom
lower_half = sim.PopulationView(sensors, indices[resolution - resolution//2:].flatten())
# left half = eight columns from left
left_half = sim.PopulationView(sensors, indices[:, :resolution // 2].flatten())
# right half = eight colums from right
right_half = sim.PopulationView(sensors, indices[:, resolution - resolution//2:].flatten())
# center nine =  nine center squares
center_nine = sim.PopulationView(sensors, indices[(resolution//2) - 1 : (resolution//2) + 2, (resolution//2) - 1 : (resolution//2) + 2].flatten())

pro_down = sim.Projection(lower_half, down, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_up = sim.Projection(upper_half, up, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_left = sim.Projection(left_half, left, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))
pro_right = sim.Projection(right_half, right, sim.AllToAllConnector(),
                      sim.StaticSynapse(weight=ones))

circuit = sensors + down + up 
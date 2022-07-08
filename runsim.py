import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
import time

from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim
import controller
np.random.seed(8)
'''
# 3D boxes   lx, ly, lz, hx, hy, hz
obstacles = [[-5, 25, 0, 20, 35, 60],
             [30, 25, 0, 55, 35, 100],
             [45, 35, 0, 55, 60, 60],
             [45, 75, 0, 55, 85, 100],
             [-5, 65, 0, 30, 70, 100],
             [70, 50, 0, 80, 80, 100]]
'''

obstacles = [[0.3 * 100, 0.3 * 100, 0 * 100, 0.5 * 100, 0.5 * 100, 1 * 100],
             [0.45 * 100, 0.75 * 100, 0 * 100, 0.65 * 100, 0.95 * 100, 0.5 * 100],
             [0.05 * 100, 0.5 * 100, 0 * 100, 0.25 * 100, 0.7 * 100, 1 * 100],
             [0.6 * 100, 0.45 * 100, 0 * 100, 0.8 * 100, 0.6 * 100, 1 * 100]]


# limits on map dimensions
bounds = np.array([0,100])
# create map with obstacles
mapobs = Map(obstacles, bounds, dim = 3)

#plan a path from start to goal
start = np.array([0,0,0])
goal = np.array([100,100,100])

rrt = RRTStar(start = start, goal = goal,
              Map = mapobs, max_iter = 500,
              goal_sample_rate = 0.1)

waypoints, min_cost = rrt.plan()


#scale the waypoints to real dimensions
waypoints = 0.01*waypoints
#print(waypoints)

#Generate trajectory through waypoints
traj = trajGenerator(waypoints, max_vel = 10, gamma = 1e6)

#initialise simulation with given controller and trajectory
Tmax = traj.TS[-1]
des_state = traj.get_des_state
sim = QuadSim(controller,des_state,Tmax)

#create a figure
fig = plt.figure()
ax = Axes3D.Axes3D(fig)
ax.set_xlim((0,1))
ax.set_ylim((0,1))
ax.set_zlim((0,1))

#plot the waypoints and obstacles
rrt.draw_path(ax, waypoints)
mapobs.plotobs(ax, scale = 0.01)

print(obstacles)
#time.sleep(30)
#run simulation
sim.run(ax)

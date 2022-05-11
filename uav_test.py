from random import random
import numpy as np
import random
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque

#from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim
import controller

def draw_path(ax,path):
        '''draw the path if available'''
        if path is None:
            print("path not available")
        else:
            ax.plot(*np.array(path).T, '-', color = (0.9, 0.2, 0.5, 0.8), zorder = 5)

for i in range(8):
    #create a figure
    fig = plt.figure()
    ax = Axes3D.Axes3D(fig)
    ax.set_xlim((0,1))
    ax.set_ylim((0,1))
    ax.set_zlim((0,1))
    #scale the waypoints to real dimensions
    #waypoints = 0.02*waypoints
    start = np.array([0,0,0])
    #goal = np.array([1,1,0])
    goal = start
    for i in range(5):
        tem_point = np.array([random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)])
        goal = np.vstack((goal, tem_point))
    land = np.array([0,0,0])
    goal = np.vstack((goal,land))
    waypoints = goal
    #print(waypoints)

    #Generate trajectory through waypoints
    traj = trajGenerator(waypoints, max_vel = 10, gamma = 1e6)
    #print(f"traj is {traj}")

    #initialise simulation with given controller and trajectory
    Tmax = traj.TS[-1]
    des_state = traj.get_des_state
 
    sim = QuadSim(controller,des_state,Tmax)

    #plot the waypoints and obstacles
    # rrt.draw_path(ax, waypoints)
    #mapobs.plotobs(ax, scale = 0.02)

    #run simulation
    draw_path(ax,waypoints)
    sim.run(ax)

    
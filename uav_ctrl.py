from random import random
from tkinter import HORIZONTAL
import numpy as np
import random
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque

#from PathPlanning import RRTStar, Map
from TrajGen import trajGenerator, Helix_waypoints, Circle_waypoints
from Quadrotor import QuadSim, QuadCtrl, Quadrotor
import controller

def draw_path(ax,path):
        '''draw the path if available'''
        if path is None:
            print("path not available")
        else:
            ax.plot(*np.array(path).T, '-', color = (0.9, 0.2, 0.5, 0.8), zorder = 5)

USE_PLOT = True
FLIGHT_HORIZENTAL = True

for i in range(8):
    if USE_PLOT == True:
        
        #create a figure
        fig = plt.figure()
        ax = Axes3D.Axes3D(fig)
        ax.set_xlim((0,1))
        ax.set_ylim((0,1))
        ax.set_zlim((0,1))
        '''
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((0,1))
            ax.set_ylim((0,1))
            ax.set_zlim((0,1))
        '''
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        lines = ax.get_lines()[-4:]   


    #deternimne flight waypoints        
    start = np.array([0,0,0])
    goal = start

    if FLIGHT_HORIZENTAL == False:
        for i in range(5):
            tem_point = np.array([random.uniform(0,1), random.uniform(0,1), random.uniform(0,1)])
            goal = np.vstack((goal, tem_point))
    else:
        for i in range(5):
            tem_point = np.array([random.uniform(0,1), random.uniform(0,1)])
            tem_point = np.hstack((tem_point,1))
            goal = np.vstack((goal, tem_point))
            
    land = np.array([0,0,0])
    goal = np.vstack((goal,land))
    waypoints = goal


    if USE_PLOT == True:
        draw_path(ax,waypoints)

    #Generate trajectory through waypoints
    traj = trajGenerator(waypoints, max_vel = 10, gamma = 1e6)

    #initialise simulation with given controller and trajectory
    Tmax = traj.TS[-1]
    get_des_state = traj.get_des_state
    
    #uav modeling
    t = 0
    control_frequency = 200
    dt = 1/control_frequency
    animation_frequency = 50     
    control_iterations = int(control_frequency / animation_frequency)

    pos = None
    attitude = [0,0,0]
    if pos is None: pos = get_des_state(0).pos
    Quad = Quadrotor(pos, attitude)

    pos_history = deque(maxlen=100)  
    animation_rate = 1/animation_frequency

    while t < Tmax + 0:
        for _ in range(control_iterations):
            des_state = get_des_state(t)
            state = Quad.get_state()

            quad_ctrl = QuadCtrl(controller,Tmax,animation_frequency,
                 control_frequency,t,dt,state,des_state)
            U,M = quad_ctrl.Step()
            Quad.update(dt, U, M)
            #print(f"uav pos is {state.pos}")
            t += dt

        if USE_PLOT == True:    
            frame = Quad.world_frame()
            lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]

            for line, line_data in zip(lines[:3], lines_data):
                x, y, z = line_data
                line.set_data(x, y)
                line.set_3d_properties(z)

            pos_history.append(frame[:,4])
            history = np.array(pos_history)
            lines[-1].set_data(history[:,0], history[:,1])
            lines[-1].set_3d_properties(history[:,-1])  

            plt.pause(animation_rate)  

    if USE_PLOT == True:
        plt.close()

    #run simulation
    #draw_path(ax,waypoints)
    #quad_ctrl.run(ax)
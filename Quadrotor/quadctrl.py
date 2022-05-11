import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque
from .quadrotor import Quadrotor

class QuadCtrl:
    def __init__(self,controller,Tmax,animation_frequency,
                 control_frequency, t, dt, state, des_state):

        self.t = t
        self.Tmax = Tmax
        self.dt = dt
        self.animation_rate = 1/animation_frequency
        self.control_iterations = int(control_frequency / animation_frequency)

        self.state = state
        self.des_state = des_state
        self.controller = controller
        
        self.pos_history = deque(maxlen=100)

    def Step(self):
        if(self.t >= self.Tmax):
            U, M = self.controller.run_hover(self.state, self.des_state,self.dt)
        else:
            U, M = self.controller.run(self.state, self.des_state)
        
        return U,M 

    '''
    def control_loop(self):
        for _ in range(self.control_iterations):
            self.Step()
            
        #return self.Quadrotor.world_frame()

    def run(self,ax = None,save = False):
        #self.init_plot(ax)
        while self.t < self.Tmax + 0:
            #frame = self.control_loop()
            self.control_loop()
            #self.update_plot(frame)
            #plt.pause(self.animation_rate)
        #plt.close()


    def init_plot(self,ax = None):
        if ax is None:
            fig = plt.figure()
            ax = Axes3D.Axes3D(fig)
            ax.set_xlim((0,1))
            ax.set_ylim((0,1))
            ax.set_zlim((0,1))
        ax.plot([], [], [], '-', c='red',zorder = 10)
        ax.plot([], [], [], '-', c='blue',zorder = 10)
        ax.plot([], [], [], '-', c='green', marker='o', markevery=2,zorder = 10)
        ax.plot([], [], [], '.', c='green', markersize=2,zorder = 10)
        self.lines = ax.get_lines()[-4:]

    def update_plot(self,frame):

        lines_data = [frame[:,[0,2]], frame[:,[1,3]], frame[:,[4,5]]]

        for line, line_data in zip(self.lines[:3], lines_data):
            x, y, z = line_data
            line.set_data(x, y)
            line.set_3d_properties(z)

        self.pos_history.append(frame[:,4])
        history = np.array(self.pos_history)
        self.lines[-1].set_data(history[:,0], history[:,1])
        self.lines[-1].set_3d_properties(history[:,-1])
    '''
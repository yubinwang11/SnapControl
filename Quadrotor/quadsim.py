import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as Axes3D
from collections import deque
from .quadrotor import Quadrotor

class QuadSim:
    def __init__(self,controller,des_state,Tmax,
                 pos = None, attitude = [0,0,0],
                 animation_frequency = 50,
                 control_frequency = 200):

        self.t = 0
        self.Tmax = Tmax
        self.dt = 1/control_frequency
        self.animation_rate = 1/animation_frequency
        self.control_iterations = int(control_frequency / animation_frequency)

        self.des_state = des_state
        self.controller = controller
        if pos is None: pos = des_state(0).pos
        self.Quadrotor = Quadrotor(pos, attitude)

        self.pos_history = deque(maxlen=100)

        self.total_energy_consumption = np.float(0)
        self.total_flight_time = 0
        self.energy_consump = None

    def Step(self):
        des_state = self.des_state(self.t)
        state = self.Quadrotor.get_state()

        power_consump = self.Quadrotor.power_consumption()
        #print(power_consump)

        if(self.t >= self.Tmax):
            U, M = self.controller.run_hover(state, des_state,self.dt)
        else:
            U, M = self.controller.run(state, des_state)
        self.Quadrotor.update(self.dt, U, M)
        #print(f"uav pos is {state.pos}")
        self.t += self.dt

        if power_consump < 100000:
            self.energy_consump = power_consump * self.dt       
            print(self.energy_consump)
        
        if self.energy_consump < 10000:
            pass
        else:
            self.total_energy_consumption += self.energy_consump
        print(self.total_energy_consumption)

        self.total_flight_time += self.dt


    def control_loop(self):
        for _ in range(self.control_iterations):
            self.Step() 
            
        return self.Quadrotor.world_frame()

    def run(self,ax = None,save = False):
        self.init_plot(ax)
        while self.t < self.Tmax + 0:
            frame = self.control_loop()
            self.update_plot(frame)
            plt.pause(self.animation_rate)

        print(f"total energy consumption is {self.total_energy_consumption}, and total flight time is {self.total_flight_time}")
        plt.close()

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

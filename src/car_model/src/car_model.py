"""
Simulation of a car model
"""
import numpy as np


class CarModel(object):
    """
    Car model simulation class. In this model, we control the
    steering speed and the speed of the robot
    """

    def __init__(self, x=0, y=0, theta=0, dt=0.1):
        """
        A car is defined by its pose and orientation
        :param dt: euler simulation step
        """
        self.X = np.array([x, y, theta])
        self.dt = dt

    def sim_step(self, speed, steering):
        """
        One euler simulation step
        """
        u = np.array([speed, steering])
        x, y, theta = self.X
        self.X = np.array([x + speed*self.dt*np.cos(theta),
                           y + speed*self.dt*np.sin(theta),
                           theta + self.dt*steering])

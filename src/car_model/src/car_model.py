"""
Simulation of a car model
"""
import numpy as np


class CarModel(object):
    """
    Car model simulation class. In this model, we control the
    steering speed and the speed of the robot
    """

    def __init__(self, x=0, y=0, theta=0, dt=0.1, noisy=False):
        """
        A car is defined by its pose and orientation
        :param dt: euler simulation step
        """
        self.X = np.array([x, y, theta])
        self.dt = dt
        self.noisy = noisy
        if noisy:
            self.steering_noise = 0.01
            self.speed_noise = 0.1

    def sim_step(self, speed, steering):
        """
        One euler simulation step
        """
        if self.noisy:
            speed = np.random.normal(speed, self.speed_noise)
            steering = np.random.normal(steering, self.steering_noise)
        u = np.array([speed, steering])
        x, y, theta = self.X
        self.X = np.array([x + speed*self.dt*np.cos(theta),
                           y + speed*self.dt*np.sin(theta),
                           theta + self.dt*steering])

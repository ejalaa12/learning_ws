#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, Twist
from car_model import CarModel


class CarNode():
    def __init__(self):
        rospy.init_node('car_sim')
        rospy.Subscriber('twist2d', Twist, self.update_command)
        dt = rospy.get_param('~dt', default=0.05)
        noisy = rospy.get_param('~noisy', default=False)
        self.rate = rospy.Rate(1./dt)
        self.car = CarModel(dt=dt)
        self.speed, self.steering = 0, 0
        self.last_time = rospy.Time.now()
        self.pose_pub = rospy.Publisher('car_pose', Pose2D, queue_size=1)

    def update_command(self, msg):
        self.last_time = rospy.Time.now()
        self.speed = msg.linear.x
        self.steering = msg.angular.z

    def check_command_validity(self):
        """
        Check if the command is not too old, else resets it to 0
        """
        if rospy.Time.now() - self.last_time > rospy.Duration(2):
            self.speed = 0
            self.steering = 0

    def work(self):
        while not rospy.is_shutdown():
            self.check_command_validity()
            self.car.sim_step(speed=self.speed, steering=self.steering)
            self.pose_pub.publish(Pose2D(*self.car.X))
            self.rate.sleep()


if __name__ == '__main__':
    node = CarNode()
    node.work()

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from tf.transformations import quaternion_from_euler

rospy.init_node('conv')
pub = rospy.Publisher('poseStamped', PoseStamped, queue_size=1)

def pub_pose(p):
    """
    type p: PoseStamped
    """
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = p.x
    msg.pose.position.y = p.y
    orientation = quaternion_from_euler(0, 0, p.theta)
    msg.pose.orientation.x = orientation[0]
    msg.pose.orientation.y = orientation[1]
    msg.pose.orientation.z = orientation[2]
    msg.pose.orientation.w = orientation[3]
    pub.publish(msg)


rospy.Subscriber('pose2d', Pose2D, pub_pose)

rospy.spin()

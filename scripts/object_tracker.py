#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from tortoisebot_object_follower.msg import object_pose

velocity_msg = Twist()
ob_pose = object_pose()

vel_pub = None
pose_pub = None


def move(linear,angular):
	global vel_pub,velocity_msg
	velocity_msg.linear.x = linear
	velocity_msg.angular.z = angular
	vel_pub.publish(velocity_msg)


def laser_callback(msg):
    global ob_pose,pose_pub
    ob_pose.z = msg.ranges[288:431]
    pose_pub.publish(ob_pose)


def object_pose_update(msg):
    global ob_pose
    if msg:
        ob_pose.x = msg.x
        ob_pose.y = msg.y


def main():
    global ob_pose, vel_pub, pose_pub
    
    rospy.init_node('ebot_controller')

    # initialize publisher to publish to /cmd_vel topic
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # subscribe to /odom and /ebot/laser/scan
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber("/object_pose", object_pose, object_pose_update)
    pose_pub = rospy.Publisher('/object_pose', object_pose, queue_size=10)

    rate = rospy.Rate(10)
    move(0,0)

    while not rospy.is_shutdown():
        move(0,0)
        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tortoisebot_object_follower.msg import object_pose
import numpy as np

velocity_msg = Twist()
ob_pose = object_pose()

laser_val = 10

# initialize publisher to publish to /cmd_vel topic
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
pose_pub = rospy.Publisher('/object_pose', object_pose, queue_size=10)


def move(linear,angular):
	global vel_pub,velocity_msg
	velocity_msg.linear.x = linear
	velocity_msg.angular.z = angular
	vel_pub.publish(velocity_msg)


def laser_callback(msg):
    global laser_val
    laser_val = min(min(msg.ranges[288:431]), 10)
    #laser_val = (0.99*ir)+0.2
    

def object_pose_update(msg):
    global ob_pose
    ob_pose.x = msg.x
    ob_pose.y = msg.y
    ob_pose.r = msg.r

def calibrate():
    global ob_pose,laser_val
    print('calibrating...')
    while(1):
        rospy.sleep(1)
        if ob_pose.x == -1 or laser_val > 3:
            print('object not found')
            print(laser_val)
        else:
            break
    const = ob_pose.r * laser_val
    print(const)
    print('calibration done')
    return const

def calculate(const):
    global ob_pose
    max_vel = 0.2
    max_theta = 2.35
    ob_pose.z = const/ob_pose.r
    if abs(ob_pose.z - 0.3) > max_vel:
        linear = max_vel * np.sign(ob_pose.z - 0.3)
    else:
        linear = ob_pose.z-0.3

    if abs(-0.005*(ob_pose.x-400)) > max_theta:
        linear = max_theta * np.sign(-0.005*(ob_pose.x-400))
    else:
        angular = -0.005*(ob_pose.x-400)
    return linear, angular

def main():
    global ob_pose,pose_pub
    
    rospy.init_node('ebot_controller')

    # subscribe to /odom and /ebot/laser/scan
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    rospy.Subscriber("/object_pose", object_pose, object_pose_update)
    print('object_tracker online')
    rate = rospy.Rate(10)
    move(0,0)
    ob_pose.z = 0
    ob_pose.x = -1
    rospy.sleep(2)
    const = calibrate()
    print('tracking started')
    while not rospy.is_shutdown():
        linear, angular = calculate(const)
        pose_pub.publish(ob_pose)
        print("distance error = {}".format(ob_pose.z - 0.35))
        if ob_pose.x == -1:
            print('object_not_found')
            move(0,0)
        else:
            move(linear, angular)


        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

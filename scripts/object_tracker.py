#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tortoisebot_object_follower.msg import object_pose

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
    ir = min(min(msg.ranges[288:431]), 10)
    laser_val = (0.99*ir)+0.2
    

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
        if ob_pose.x == -1 and laser_val > 3:
            print('object not found')
        else:
            break
    const = ob_pose.r * laser_val
    print(const)
    print('calibration done')
    return const

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
    rospy.sleep(2)
    const = calibrate()
    print('tracking started')
    while not rospy.is_shutdown():
        ob_pose.z = const/ob_pose.r
        pose_pub.publish(ob_pose)
        print("distance = {}".format(ob_pose.z))
        if ob_pose.x == -1:
            print('object_not_found')
            move(0,0)
        else:
            move((ob_pose.z-0.35), -0.004*(ob_pose.x-415))


        rate.sleep()




if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

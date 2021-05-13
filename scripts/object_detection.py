#!/usr/bin/env python

import rospy
import cv2
# from std_msgs.msg import string 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import sys

bridge = CvBridge()

class Object_tracker:
    def __init__(self):
        image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def image_callback(self, ros_image):
        print('image recieved')
        try:
            cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Camera_output", cv_image)
        cv2.waitKey(1) 


def main():
    rospy.init_node('Object_follower')
    object = Object_tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
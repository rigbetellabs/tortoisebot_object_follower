#!/usr/bin/env python

import numpy as np
import rospy
import cv2
from tortoisebot_object_follower.msg import object_pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class Object_tracker:
    def __init__(self):
        rospy.Subscriber("/object_pose", object_pose, self.object_pose_update)
        self.pose_pub = rospy.Publisher('/object_pose', object_pose, queue_size=10)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.yellowLower =(30, 100, 50)
        self.yellowUpper = (60, 255, 255)
        self.distance = 0

    def object_pose_update(self, msg):
        self.distance = msg.x

    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.ball_detection(cv_image)
    
    def filter_color(self, rgb_image, lower_bound_color, upper_bound_color):
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv image",hsv_image)
        masked_image = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
        return masked_image

    def getContours(self, binary_image):
        contours, hierarchy = cv2.findContours(binary_image.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def get_contour_center(contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return cx, cy

    def draw_ball_contour(self, binary_image, rgb_image, contours):
        black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
        
        for c in contours:
            area = cv2.countorArea(c)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            if (area>3000):
                cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                cx, cy = self.get_contour_center(c)
                cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
                cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
                
        cv2.imshow("Object_detector",rgb_image)
        cv2.imshow("Black_image",black_image)

    def ball_detection(self, image):
        rgb_image = image
        binary_image_mask = self.filter_color( rgb_image, self.yellowLower, self.yellowUpper)
        contours = self.getContours(binary_image_mask)
        self.draw_ball_contour(binary_image_mask, rgb_image,contours)




def main():
    rospy.init_node('Object_detection')
    object = Object_tracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down')
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
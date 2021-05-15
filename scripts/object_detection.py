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
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.ob_pose = object_pose()
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.yellowLower =(110,100,100)
        self.yellowUpper = (130,255,255)
        self.reset_ob_val()

    def reset_ob_val(self):
        self.ob_pose.x = -1
        self.ob_pose.y = -1
        self.ob_pose.z = -1
        self.ob_pose.r = -1

    def object_pose_update(self, msg):
        self.ob_pose.z = msg.x

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
        if contours:
            return contours
        else:
            return 0

    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return cx, cy

    def draw_ball_contour(self, binary_image, rgb_image, contours):
        #print(contours)
        black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
        
        if contours == 0:
            cv2.putText(black_image,'object not found',(20,15), self.font, 0.81,(255,255,255),1)

        else:
            for c in contours:
                area = cv2.contourArea(c)

                if (area>500):
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                    self.ob_pose.x, self.ob_pose.y = self.get_contour_center(c)
                    self.ob_pose.r = radius
                    self.pose_pub.publish(self.ob_pose)
                    cv2.circle(rgb_image, (int(x),int(y)),int(radius),(0,0,255),1)
                    cv2.circle(black_image, (self.ob_pose.x,self.ob_pose.y),(int)(radius),(0,0,255),1)

            cv2.putText(black_image,'ob_x = {}'.format(self.ob_pose.x),(15,20), self.font, 0.7,(255,255,255),1)
            cv2.putText(black_image,'ob_y = {}'.format(self.ob_pose.y),(15,60), self.font, 0.7,(255,255,255),1)
            cv2.putText(black_image,'ob_z = {}'.format(self.ob_pose.z),(15,100), self.font, 0.7,(255,255,255),1)
            cv2.putText(black_image,'ob_r = {}'.format(int(self.ob_pose.r)),(15,140), self.font, 0.7,(255,255,255),1)
        cv2.imshow("Object_detector",rgb_image)
        cv2.imshow("Black_image",black_image)
        cv2.waitKey(1)

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
    del(object)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
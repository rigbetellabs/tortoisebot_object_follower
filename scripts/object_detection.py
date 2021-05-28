#!/usr/bin/env python

import numpy as np
import rospy
import cv2
from tortoisebot_object_follower.msg import object_pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#A class to detect object, dispay the detected object and send object data to a topic for tracking
class Object_tracker:
    def __init__(self):
        rospy.Subscriber("/object_pose", object_pose, self.object_pose_update)
        self.pose_pub = rospy.Publisher('/object_pose', object_pose, queue_size=10)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        self.ob_pose = object_pose()
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.yellowLower =(90,50,50)            # Lower Limit for color detection
        self.yellowUpper = (150,255,255)        # Upper Limit for color detection
        self.ob_pose.z = -1
        self.reset_ob_val()

    def reset_ob_val(self):
        self.ob_pose.x = -1
        self.ob_pose.y = -1
        self.ob_pose.r = -1

    def object_pose_update(self, msg):
        self.ob_pose.z = msg.z


    '''Function -   recive image from camera and convert image for image processing and calls ball_detection()
       Arguments -  image from camera 
    '''
    def image_callback(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.ball_detection(cv_image)
    

    '''Function -   Convert the image to HSV and then to binary to get only the bounded colors
       Arguments -  RGB image, upper and lower bound of color
       Returns -    Binary masked image
    ''' 
    def filter_color(self, rgb_image, lower_bound_color, upper_bound_color):
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv image",hsv_image)
        masked_image = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)
        return masked_image

    
    '''Function -   Get all the contours(closed edges) from the color bounder binary image
       Arguments -  Binary image
       Returns -    All the contours if any/0 if no contours are found
    '''
    def getContours(self, binary_image):
        _, contours, hierarchy = cv2.findContours(binary_image.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return contours
        else:
            return 0

    '''Function -   Get center of contour
       Arguments -  contour
       Returns -    X and Y coordinates of the center
    '''
    def get_contour_center(self, contour):
        M = cv2.moments(contour)
        cx=-1
        cy=-1
        if (M['m00']!=0):
            cx= int(M['m10']/M['m00'])
            cy= int(M['m01']/M['m00'])
        return cx, cy

    def dispay_window(self, winname, img, x, y):
        cv2.namedWindow(winname,cv2.WINDOW_NORMAL)        # Create a named window
        cv2.moveWindow(winname, x, y)   # Move it to (x,y)
        cv2.resizeWindow(winname, 580, 580)
        cv2.imshow(winname,img)
        


    '''Function -   Get all the contours(closed edges) from the color bounder binary image
       Arguments -  Binary image, RGB image, contours detected in the binary image
       Returns -    All the contours if any/0 if no contours are found
    '''
    def draw_ball_contour(self, binary_image, rgb_image, contours):
        #print(contours)
        black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
        cv2.line(black_image, (400,0), (400,1000), (0,255,0),2)
        if contours == 0:
            cv2.putText(black_image,'Object Not Detected',(20,15), self.font, 0.81,(255,255,255),1)

        else:
            area = 0
            c = 0
            for i in contours:
                ar = cv2.contourArea(i)
                if ar > area:
                    area = ar
                    c = i 

            if (area>10):
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
                self.ob_pose.x, self.ob_pose.y = self.get_contour_center(c)
                self.ob_pose.r = radius

                cv2.circle(rgb_image, (int(x),int(y)),int(radius),(0,0,255),2)
                cv2.circle(black_image, (self.ob_pose.x,self.ob_pose.y),(int)(radius),(255,0,0),-1)
            
                cv2.putText(black_image,'POSITION ERROR',(15,20), self.font, 0.8,(0,0,255),1)
                if self.ob_pose.x>430:
                    cv2.putText(black_image,'RIGHT : {}'.format(self.ob_pose.x-400),(15,60), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'TURN LEFT',(310,40), self.font, 1.5,(0,255,0),2)
                    cv2.arrowedLine(rgb_image, (290,20), (230,20),(0,255,0),3)
                elif self.ob_pose.x<400:
                    cv2.putText(black_image,'LEFT : {}'.format(400-self.ob_pose.x),(15,60), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'TURN RIGHT',(310,40), self.font, 1.5,(0,255,0),2)
                    cv2.arrowedLine(rgb_image, (600,20), (660,20),(0,255,0),3)
                else:
                    cv2.putText(black_image,'CENTER',(15,60), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'CENTER',(310,40), self.font, 1.5,(0,255,0),2)

                if self.ob_pose.z > 0.36:
                    cv2.putText(black_image,'Distance Error : {}'.format(self.ob_pose.z - 0.3),(15,120), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'GO Forward',(310,750), self.font, 1.5,(0,0,255),2)
                elif self.ob_pose.z < 0.24:
                    cv2.putText(black_image,'Distance Error : {}'.format(self.ob_pose.z - 0.3),(15,120), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'Go Back',(310,750), self.font, 1.5,(0,0,255),2)
                else:
                    cv2.putText(black_image,'STOP',(15,120), self.font, 0.8,(255,255,255),2)
                    cv2.putText(rgb_image,'STOP',(310,750), self.font, 1.5,(0,0,255),2)

                cv2.putText(black_image,'Object Area : {}'.format(int(area)),(15,160), self.font, 0.8,(255,255,255),2)
            else :
                cv2.putText(black_image,'object too small',(20,15), self.font, 0.81,(255,255,255),1)

        self.pose_pub.publish(self.ob_pose)
        self.dispay_window("Position Detector", black_image, 0,0)
        self.dispay_window("Detected_objects", rgb_image, 0,500)
        # cv2.imshow("Object_detector",rgb_image)
        # cv2.imshow("Black_image",black_image)
        cv2.waitKey(1)


    '''Function -   Calls different functions for image processing
       Arguments -  RGB image
    '''
    def ball_detection(self, image):
        rgb_image = image
        self.reset_ob_val()
        binary_image_mask = self.filter_color( rgb_image, self.yellowLower, self.yellowUpper)
        contours = self.getContours(binary_image_mask)
        self.draw_ball_contour(binary_image_mask, rgb_image,contours)




def main():
    rospy.init_node('Object_detection')
    object = Object_tracker()
    rospy.spin()

if __name__ == '__main__':
    main()
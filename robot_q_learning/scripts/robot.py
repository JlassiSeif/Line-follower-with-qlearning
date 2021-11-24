#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class robot :
    def __init__(self) :
        self.vel = Twist()
        self.cmdPub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.imgPub = rospy.Publisher('/image/thresh',Image,queue_size=1)
        self.bridge = CvBridge()
        self.CameraSub = rospy.Subscriber('/camera/image_raw',Image,self.CameraCallback)
        self.yaw = 0.0
        self.noContour = True
        self.cx = 0
        self.array = []
        self.distanceFromCenter=0;



    def CameraCallback(self, data) :
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        (height,width,channels) = cv_image.shape
        crop_img = cv_image[int(height/2)+100:height-100, 0:width]
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray,(5,5),0)
        ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
        contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0 :
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            self.noContour = False
            if not M['m00']== 0 :
                self.cx = int(M['m10']/M['m00'])
        else : 
            self.noContour = True

        # pubImage = self.bridge.cv2_to_imgmsg(thresh)
        # self.imgPub.publish(pubImage)
        self.distanceFromCenter=width/2-self.cx
    
    def move(self, action, speed):
        if speed == 0 :
            val = 0.2
        elif speed == 1 :
            val = 0.5
        elif speed == 2 :
            val = 0.7
        else :
            val = 0.3
        if( action == 0 ):
            self.vel.linear.x = 0.25
            self.vel.angular.z = 0
            self.cmdPub.publish(self.vel)
        if( action == 1):
            self.vel.linear.x = 0.25
            self.vel.angular.z = -val
            self.cmdPub.publish(self.vel)
        if( action == 2):
            self.vel.linear.x = 0.25
            self.vel.angular.z = val
            self.cmdPub.publish(self.vel)
        if(action == 9):
            self.vel=Twist()
            self.cmdPub.publish(self.vel)

    def index_f(self, data) :
        if -400 <= data < -280 :
            return 0
        elif -280 <= data < -200 :
            return 1
        elif  -200 <= data < -120:
            return 2
        elif -120 <= data < -40 :
            return 3
        elif -40 <= data < 40 :
            return 4
        if 40 <= data < 120 :
            return 5
        elif  120 <= data < 200 :
            return 6
        elif 200 <= data < 280 :
            return 7
        else :
            return 8
    def get_noContour(self):
        return self.noContour
    
    
    def computeState(self):
        stateIndex=self.index_f(self.distanceFromCenter)
        return stateIndex



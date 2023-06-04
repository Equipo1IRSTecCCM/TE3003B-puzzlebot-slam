#!/usr/bin/env python
import cv2
import numpy as np
import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class line_follower:
    def __init__(self):
        self.img = np.array([])
        self.bridge = CvBridge()
        self.dt = 0.1
        rospy.init_node("line_follower")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        self.twist_publisher = rospy.Publisher('/img_processing/cmd_vel', Twist, queue_size=10)
        self.debug_msg = rospy.Publisher('/ojos',Image,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)

    def source_callback(self,msg):
        try:
            self.w = msg.width
            self.h = msg.height
            self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except:
            pass

    
    def timer_callback(self,time):
        vel = 0.1
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("la vida es trsiteza")
            pass
        try:
            if continuar:
                
                img = imagen_resize[int(imagen_resize.shape[0]*5/6):int(imagen_resize.shape[0])-1,:]#int(imagen_resize.shape[1]*4/20):int(imagen_resize.shape[1]*14/20)-1]
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

                lower = np.array([100, 100, 0], np.uint8)
                upper = np.array([125, 185, 255], np.uint8)

                img = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((1,1),np.uint8)

                img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
                img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
                M = cv2.moments(img)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                
                d = int(img.shape[1]/2) - cX
                cv2.putText(img,str(d),(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,0,0),1) 
                kp = 0.0005
                msg = Twist()
                msg.linear.x = vel
                msg.linear.y = 0
                msg.linear.z = 0
                msg.angular.x = 0
                msg.angular.y = 0
                msg.angular.z = kp*d
                self.twist_publisher.publish(msg)
                msg_img = Image()
                msg_img = self.bridge.cv2_to_imgmsg(img)
                self.debug_msg.publish(msg_img)
        except:
            msg = Twist()
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.twist_publisher.publish(msg)
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #publicar
        self.twist_publisher.publish(msg)
        print("Muerte y destruccion o shutdown")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    lineator = line_follower()
    try:
        lineator.run()
    except:
        pass

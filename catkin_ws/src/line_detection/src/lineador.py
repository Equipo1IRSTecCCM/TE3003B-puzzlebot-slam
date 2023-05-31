#!/usr/bin/env python
import cv2
import numpy as np

import rospy
import time
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from csv import reader

class line_follower:
    def __init__(self):
        self.img = np.array([])
        self.ignorarTimer = 38
        self.bridge = CvBridge()
        self.dt = 0.1
        self.max_v = 0.3
        self.estancado = False
        self.red_density = 0
        self.green_density = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.antDist = []
        self.doing_pp = False
        self.sem_verde = False
        self.max_w = 0.15
        self.enRecta = False
        self.gd = 0
        self.rd = 0
        self.X,self.Y,self.Za,self.Zl = self.read()
        rospy.init_node("line_follower")

        rospy.Subscriber('/video_source/raw',Image,self.source_callback)
        self.twist_publisher = rospy.Publisher('/img_processing/cmd_vel', Twist, queue_size=10)
        self.debug_msg = rospy.Publisher('/ojos',Image,queue_size=10)
        self.t1 = rospy.Timer(rospy.Duration(self.dt),self.timer_callback)
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop)
        
    def g_callback(self,msg):
        self.gd = msg.data
    def r_callback(self,msg):
        self.rd = msg.data
    def odom_callback(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta 
    def source_callback(self,msg):
        try:
            self.w = msg.width
            self.h = msg.height
            self.img = self.bridge.imgmsg_to_cv2(msg,'bgr8')
        except:
            pass

    def getDistance(self,x1,y1,x2,y2):
        return np.sqrt((x1-x2)**2+(y1-y2)**2)
    def getAngle(self,x1,y1,x2,y2):
        return np.arctan2(y1-y2,x1-x2)
    
    def skeletonize(self,img):
        size = np.size(img)
        skel = np.zeros(img.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3,3))
        while True:
            #Step 2: Open the image
            open = cv2.morphologyEx(img, cv2.MORPH_OPEN, element)
            #Step 3: Substract open from the original image
            temp = cv2.subtract(img, open)
            #Step 4: Erode the original image and refine the skeleton
            eroded = cv2.erode(img, element)
            skel = cv2.bitwise_or(skel,temp)
            img = eroded.copy()
            # Step 5: If there are no white pixels left ie.. the image has been completely eroded, quit the loop
            if cv2.countNonZero(img)==0:
                break
        return skel
    def evaluarIdx(self,x,v):
        """
        Find the index of v on x (or the closest value)
        
        Parameters
        ----------
        x :     numpy.array(float[])
            The array
        v :     float
            The searched value
        """
        idx = np.where(x==v)
        #In case it's empty
        if idx[0].shape == (0,):
            closest = 0 
            for i in x:
                if abs(v-i) < abs(v-closest):
                    closest = i
            idx = np.where(x==closest)
        #print(x)
        return idx[0][0]
    def getValues(self,p,d):
        idx_l = self.evaluarIdx(self.Y,p)
        idx_a = self.evaluarIdx(self.X,d)
        val_a = self.Za[idx_l][idx_a]
        val_l = self.Zl[idx_l][idx_a]
        return val_a,val_l
    def read(self):
        p = "/home/super-diego/TE3003B-puzzlebot-slam/catkin_ws/src/line_detection/src/"
        with open(p + 'az.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Za = np.array([list(map(float, sublist)) for sublist in list_z])

        with open(p + 'lz.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        Zl = np.array([list(map(float, sublist)) for sublist in list_z])
        
        with open(p + 'x.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        x_temp = [list(map(float, sublist)) for sublist in list_z]
        X = np.array(x_temp[0])

        with open(p +'y.csv', 'r') as read_obj:
            csv_reader = reader(read_obj)
            list_z = list(csv_reader)
            
        y_temp = np.array([list(map(float, sublist)) for sublist in list_z])
        y_temp = np.transpose(y_temp)
        Y = np.array(y_temp[0])
        return X, Y, Za, Zl
    def timer_callback(self,time):
        omega = 0
        vel = 0.1
        negro = np.zeros((5,5),np.uint8)
        continuar = False
        try:
            imagen_resize = cv2.resize(self.img,None,fx=0.3,fy=0.3)
            continuar = True
        except:
            print("la vida es trsiteza")
            pass
        try:
            if continuar:
                img = cv2.cvtColor(imagen_resize,cv2.COLOR_BGR2GRAY)
                img_gaus = cv2.GaussianBlur(img,(3,3),cv2.BORDER_DEFAULT)
                img_gaus = cv2.GaussianBlur(img_gaus,(3,3),cv2.BORDER_DEFAULT)
                img_gaus = cv2.GaussianBlur(img_gaus,(3,3),cv2.BORDER_DEFAULT)
                img_gaus = cv2.GaussianBlur(img_gaus,(3,3),cv2.BORDER_DEFAULT)
                img = img_gaus[int(img_gaus.shape[0]*5/6):int(img_gaus.shape[0])-1,int(img_gaus.shape[1]*4/20):int(img_gaus.shape[1]*14/20)-1]
                img = img.astype(float) * 1.0
                img = img.astype(np.uint8)
                
                _,img = cv2.threshold(img, 120, 255, cv2.THRESH_BINARY)
                img = cv2.bitwise_not(img)
                
                skel = self.skeletonize(img)
                M = cv2.moments(img)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                # cv2.circle(img,(cX,cY),5,(0,0,0),-1)
                
                d = int(img.shape[1]/2) - cX
                cv2.putText(img,str(d),(cX,cY),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,0,0),1) 
                kp = 0.0001
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
    def getDensitySmall(self,img):
        return 1 - np.sum(img) / (img.shape[0] * img.shape[1]) / 255
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

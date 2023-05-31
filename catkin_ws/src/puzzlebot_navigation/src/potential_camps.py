#!/usr/bin/env python3
# Campos potenciales





import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import quaternion_matrix, quaternion_multiply




class Laser():
    u"""Class that handles laser information"""

    def __init__(self):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb (self, msg):
        # Laser scan callback function
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the laser value"""
        return self._scan_data

class pot_fields:
    def __init__(self, obstacle_distance = 1.0, max_lin = 0.2, min_lin = 0.05, max_ang = 0.3, min_ang = 0.1, gain_rep = 0.004, gain_att = 2.0, prefix = ""):
        rospy.init_node('evasion_notebook_node') 
        _ = rospy.wait_for_message('scan', LaserScan)
        self.vel_pub = rospy.Publisher('/pot_fields/cmd_vel', Twist, queue_size=10)
        self.odom_sub= rospy.Subscriber("/odom",Odometry,self.odom_cb)
        self.obj_sub = rospy.Subscriber('/brain/map_objective',PointStamped,self.obj_cb)
        # self.cl_sub = rospy.Subscriber('/clicked_point',PointStamped,self.obj_cb)
        self.laser = Laser()
        if prefix != "":
            self.prefix = "/" + prefix
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.x_obj = 0.
        self.y_obj = 0.0
        self.trans = np.array([0,0,0])
        self.rot = np.array([0,0,0,1])
        self.obstacle_distance = obstacle_distance
        self.max_lin = max_lin
        self.min_lin = min_lin
        self.min_ang = min_ang
        self.max_ang = max_ang
        self.gain_rep = gain_rep
        self.gain_att = gain_att
        self.listener = tf.TransformListener()

        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.stop)

    def stop(self):
        v = Twist()
        self.vel_pub.publish(v)

    def odom_cb(self,msg): 
        translation_matrix = [[1, 0, 0, self.trans[0]],
                                [0, 1, 0, self.trans[1]],
                                [0, 0, 1, self.trans[2]],
                                [0, 0, 0, 1]]
        rotation_matrix = quaternion_matrix(self.rot)

        pose_in_odom = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        point_in_map = np.dot(transformation_matrix,
                         [pose_in_odom[0], pose_in_odom[1], pose_in_odom[2], 1])
        orientation_in_map = quaternion_multiply(self.rot,
                                            [pose_in_odom[3], pose_in_odom[4], pose_in_odom[5], pose_in_odom[6]])
        self.x = point_in_map[0]
        self.y = point_in_map[1]
        quaternion = (
        orientation_in_map[0],
        orientation_in_map[1],
        orientation_in_map[2],
        orientation_in_map[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th = euler[2]

    def obj_cb(self,msg):
        self.x_obj = msg.point.x
        self.y_obj = msg.point.y



    def get_Rep_Force(self):
        try:
            data = self.laser.get_data()
            lec = np.asarray(data.ranges)
            lec[np.isinf(lec)] = 13.5
            deltaang = data.angle_increment
            laserdegs = np.arange(data.angle_min,data.angle_max,deltaang)
            Fx = 0.0
            Fy = 0.0
            for i,deg in enumerate(laserdegs):
                if (lec[i] < self.obstacle_distance): ###TUNABLE
                    Fx += (1/lec[i])**2 * np.cos(deg)
                    Fy += (1/lec[i])**2 * np.sin(deg)
            Fth = np.arctan2(Fy,(Fx))+np.pi
            Fmag = np.linalg.norm((Fx,Fy))
            return Fx, Fy, Fmag, Fth
        except:
            return 0, 0, 0, 0
        
    


    def get_Att_Force(self):
        xy, xycl = np.array((self.x,self.y)), np.array((self.x_obj,self.y_obj))
        euclD = np.linalg.norm(xy-xycl)
        Fatrx =(-self.x + self.x_obj)/euclD
        Fatry =(-self.y + self.y_obj)/euclD      
        Fatrth = np.arctan2(Fatry, Fatrx) 
        Fatrth = Fatrth-self.th
        Fmagat = np.linalg.norm((Fatrx,Fatry))
        return  Fatrx, Fatry , Fmagat, Fatrth, euclD


    def get_Speed(self,Ftotx,Ftoty,Ftotth):
        speed=Twist()
        if(abs(Ftotth) < self.min_ang) :
            speed.linear.x = self.max_lin
            speed.angular.z = 0

        else:
            sign = abs(Ftotth)/Ftotth         
            if (abs(Ftotth) < np.pi/2):
                speed.linear.x= self.min_lin
                speed.angular.z = self.max_ang * sign / 2
            
            else:                    
                speed.linear.x = 0.0
                speed.angular.z = self.max_ang * sign

        return speed
    def run(self):


        while not rospy.is_shutdown():
            try:
                (self.trans, self.rot) = self.listener.lookupTransform('/map', '{}/odom'.format(self.prefix), rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            xy, xycl = np.array((self.x,self.y)), np.array((self.x_obj,self.y_obj))
            euclD=np.linalg.norm(xy-xycl)


            _,_,Fmag,Fth = self.get_Rep_Force()
            _,_,Fmagat,Fatrth, euclD = self.get_Att_Force()

            Ftotx = -Fmag * np.cos(Fth) * self.gain_rep + Fmagat * np.cos(Fatrth) * self.gain_att
            Ftoty = -Fmag * np.sin(Fth) * self.gain_rep + Fmagat * np.sin(Fatrth) * self.gain_att
            Ftotth = np.arctan2(Ftoty,Ftotx)
            ###ANGLE NORMALIZATION
            if ( Ftotth> np.pi ):     
                Ftotth = -np.pi - (Ftotth-np.pi)
            if (Ftotth < -np.pi): 
                Ftotth = (Ftotth + 2 * np.pi)
            ####
            speed = self.get_Speed(Ftotx,Ftoty,Ftotth)

            radioRobot = 0.1
            if euclD < radioRobot * 2: 
                speed.angular.z = 0.0
                speed.linear.x = 0.0
                print("im here", euclD,(euclD < radioRobot))

            self.vel_pub.publish(speed)

            
            self.rate.sleep()
if __name__ == "__main__":
    try:
        p = rospy.get_param('puzzlebot_odom/prefix_robot')
    except:
        p = ""
    pot = pot_fields(obstacle_distance=0.5, prefix=p)
    pot.run()
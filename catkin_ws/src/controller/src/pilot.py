#!/usr/bin/env python3

#Arbitro
import math
import numpy as np
import rospy
import smach
import smach_ros
import tf
import time
from tf.transformations import quaternion_matrix, quaternion_multiply
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Int16
from std_msgs.msg import Bool

class pilot:
    def __init__(self, x = 0.0, y = 0.0, th = 0.0, prefix = ""):

        rospy.init_node("Toretto")
        rospy.on_shutdown(self.stop)
        #Referee
        self.state = 0
        
        #Robot ns
        if prefix != "":
            prefix = "/" + prefix
        self.prefix = prefix
        print("Prefix: {}".format(prefix))
        #Objective generation
        if prefix == "" or prefix == "/he":
            self.objectives = [[1.5,1.1],[2.5,1.1],[3.5,1.5],[5.0,1.7],[9.6,1.9]]#,[10.1,1.9]]
        elif prefix == "/fb":
            #Cambiar el último punto por las medidas justo al frente de la línea
            self.objectives = [[1.5,1.1],[2.5,1.1],[3.5,1.3],[5.0,1.5],[9.6,1.5]]
        elif prefix == "/mm":
            self.objectives = [[1.5,1.1],[2.5,2.0],[5.0,2.0]]
        print("Obj: {}".format(self.objectives))
        self.obj_idx = 0
        self.obj_num = len(self.objectives)
        self.obj_pub = rospy.Publisher('{}/brain/map_objective'.format(self.prefix), PointStamped, queue_size=10)

        #Cmd publisher
        self.cmd_pub = rospy.Publisher('{}/cmd_vel'.format(self.prefix), Twist, queue_size=10)

        #Odometry
        self.x = x
        self.y = y
        self.th = th

        self.listener = tf.TransformListener()
        self.odom_sub= rospy.Subscriber("{}/odom".format(self.prefix),Odometry,self.odom_cb)

        #Pot fields navigation
        self.nav_cmd = Twist()

        self.nav_sub = rospy.Subscriber("{}/pot_fields/cmd_vel".format(self.prefix), Twist, self.nav_cb)

        #Wall follower
        self.wal_cmd = Twist()

        self.wal_sub = rospy.Subscriber("{}/wall/cmd_vel".format(self.prefix), Twist, self.wal_cb)
        #Line follower
        self.lin_cmd = Twist()

        self.lin_sub = rospy.Subscriber("{}/img_processing/cmd_vel".format(self.prefix), Twist, self.lin_cb)

        #Arriving flag
        self.arrived = False
        self.std_vel = [0,0,0,0,0,0,0,0,0,0]
        self.std_idx = 0

        self.arr_pub = rospy.Publisher("{}/arrived".format(self.prefix), Bool, queue_size=10)
        #Publish the first point
        time.sleep(1)
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = self.objectives[self.obj_idx][0]
        point.point.y = self.objectives[self.obj_idx][1]
        self.obj_pub.publish(point)

    def odom_cb(self,msg):
        try:
            (self.trans, self.rot) = self.listener.lookupTransform('/map', '{}/odom'.format(self.prefix), rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.trans = [0,0,0]
            self.rot = [0,0,0,1]
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
    
    def nav_cb(self,msg):
        self.nav_cmd = msg

    def wal_cb(self,msg):
        self.wal_cmd = msg
    
    def lin_cb(self, msg):
        self.lin_cmd = msg
    
    def run(self):
        rate = rospy.Rate(10)
        th_obj = 0.0
        th_tol = 0.05
        past_x = self.x
        past_y = self.y
        while not rospy.is_shutdown():
            vel = np.sqrt((past_x - self.x)**2 + (past_y - self.y))
            self.std_vel[self.std_idx] = vel
            std = np.mean(self.std_vel)
            self.std_idx += 1
            if self.std_idx >= len(self.std_vel):
                self.std_idx = 0
            #Navigating
            if self.state == 0:
                x_obj = self.objectives[self.obj_idx][0]
                y_obj = self.objectives[self.obj_idx][1]
                dist = np.sqrt((self.x - x_obj)**2 + (self.y - y_obj)**2)
                print("Dist: {}".format(dist))
                if dist < 0.3:
                    self.obj_idx += 1
                    if self.obj_idx < self.obj_num:
                        point = PointStamped()
                        point.header.stamp = rospy.Time.now()
                        point.header.frame_id = "map"
                        point.point.x = self.objectives[self.obj_idx][0]
                        point.point.y = self.objectives[self.obj_idx][1]
                        self.obj_pub.publish(point)
                    else:
                        self.state = 1
                        print("Change to state 1")
                self.cmd_pub.publish(self.nav_cmd)
            #Wall follower
            elif self.state == 1:
                print("Following wall")
                if self.x < self.objectives[self.obj_num - 1][0] + 0.2:
                    self.cmd_pub.publish(self.wal_cmd)
                else:
                    self.state = 2
                    print("Change to state 2")
            #Allign
            elif self.state == 2:
                th_err = th_obj - self.th
                print("thr_err: {}".format(th_err))
                if th_err < th_tol:
                    self.state = 3
                    print("Change to state 3")
                else:
                    kp = 0.5
                    cmd = Twist()
                    cmd.angular.z = th_err * kp
                    self.cmd_pub.publish(cmd)
            #Follow line
            elif self.state == 3:
                print("Following line")
                if self.lin_cmd.linear.x == 0.0:
                    pass
                #Send go to arm when the last 10 speeds were 0
                if abs(std) < 0.05:
                    self.arrived = True
                self.cmd_pub.publish(self.lin_cmd)
            

            past_x = self.x
            past_y = self.y
            self.arr_pub.publish(self.arrived)
            rate.sleep()
    def stop(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        #Stop robot
        self.cmd_pub.publish(msg)
if __name__ == "__main__":
    try:
        p = rospy.get_param('puzzlebot_odom/prefix_robot')
    except:
        p = ""
    try:
        x = float(rospy.get_param('puzzlebot_odom/x'))
    except:
        x = 0.0
    try:
        y = float(rospy.get_param('puzzlebot_odom/y'))
    except:
        y = 0.0
    try:
        t = float(rospy.get_param('puzzlebot_odom/t'))
    except:
        t = 0.0
    p = pilot(prefix = p, x = x, y = y, th = t)
    p.run()
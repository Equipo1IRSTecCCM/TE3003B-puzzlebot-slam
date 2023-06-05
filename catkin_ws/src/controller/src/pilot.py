#!/usr/bin/env python3
'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Decides which type of navigation it uses
@authors Diego Reyna Reyes
@authors Francisco Emiliano Rocha Pineda
@authors Samantha Barrón Martinez
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''

import numpy as np
import rospy
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
    '''
    Starts the pilot class
    @param x: Intial x position
    @param y: Intial y position
    @param th: Intial yaw position
    @param prefix: Namespace of the robot
    '''
    def __init__(self, x = 0.0, y = 0.0, th = 0.0, prefix = ""):
        #Start node
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
            self.objectives = [[3.5,1.1],[9.6,2.3]]
        elif prefix == "/fb":
            self.objectives = [[11.1,1.1]]
        elif prefix == "/mm":
            self.objectives = [[1.5,1.1],[2.5,2.0],[5.0,-0.5]]
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
    '''
    Callback for the /odom topic
    @param msg: Odometry message
    '''
    def odom_cb(self,msg):
        # Get tf
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
        # Transform pose
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
    '''
    Callback for the /pot_fields/cmd_vel topic
    @param msg: Twist message
    '''
    def nav_cb(self,msg):
        self.nav_cmd = msg
    '''
    Callback for the /wall/cmd_vel topic
    @param msg: Twist message
    '''
    def wal_cb(self,msg):
        self.wal_cmd = msg
    
    '''
    Callback for the /img_processing/cmd_vel topic
    @param msg: Twist message
    '''
    def lin_cb(self, msg):
        self.lin_cmd = msg
    '''
    Selects the algorithm to use
    '''
    def run(self):
        # Define variables
        rate = rospy.Rate(10)
        th_obj = 0.0
        th_tol = 0.05
        past_x = self.x
        past_y = self.y
        while not rospy.is_shutdown():
            # Calculate speed
            vel = np.sqrt((past_x - self.x)**2 + (past_y - self.y))
            self.std_vel[self.std_idx] = vel
            std = np.mean(self.std_vel)
            self.std_idx += 1
            if self.std_idx >= len(self.std_vel):
                self.std_idx = 0
            # Navigating
            if self.state == 0:
                print(f'state: {self.state}')
                x_obj = self.objectives[self.obj_idx][0]
                y_obj = self.objectives[self.obj_idx][1]
                dist = np.sqrt((self.x - x_obj)**2 + (self.y - y_obj)**2)
                print("Dist: {}".format(dist))
                # Send next point
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
                # When advanced 0.2m change behaviour
                if self.x < self.objectives[self.obj_num - 1][0] + 0.2:
                    self.cmd_pub.publish(self.wal_cmd)
                else:
                    self.state = 2
                    print("Change to state 2")
            #Allign
            elif self.state == 2:
                th_err = th_obj - self.th
                print("thr_err: {}".format(th_err))
                # Allign to yaw = 0
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
                # Follow the line
                if self.lin_cmd.linear.x == 0.0:
                    pass
                #Send go to arm when the last 10 speeds were 0
                if abs(std) < 0.05:
                    self.arrived = True
                self.cmd_pub.publish(self.lin_cmd)
            
            # Update before next iteration
            past_x = self.x
            past_y = self.y
            self.arr_pub.publish(self.arrived)
            rate.sleep()
    '''
    Stops the robot when spin stops
    '''
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
    # Get the parameters
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
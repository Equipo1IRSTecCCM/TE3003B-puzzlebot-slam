#!/usr/bin/env python3
'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Uses Dead reckoning and the kinematic model 
@author Diego Reyna Reyes
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32, Float64
from math import cos, sin
import numpy as np

L = 0.191
R = 0.05



class init_publisher:
    '''
    Starts the init_pub class for map merge
    @param x: Intial x position
    @param y: Intial y position
    @param th: Intial yaw position
    @param prefix: Namespace of the robot
    '''
    def __init__(self,x = 0.0, y = 0.0, th = 0.0, prefix = ""):
        self.x = x
        self.y = y
        self.z = 0.0
        self.th = th
        self.prefix = prefix

        self.x_pub = rospy.Publisher("{}/map_merge/init_pose_x".format(self.prefix),Float64, queue_size=10)
        self.y_pub = rospy.Publisher("{}/map_merge/init_pose_y".format(self.prefix),Float64, queue_size=10)
        self.z_pub = rospy.Publisher("{}/map_merge/init_pose_z".format(self.prefix),Float64, queue_size=10)
        self.th_pub = rospy.Publisher("{}/map_merge/init_pose_yaw".format(self.prefix),Float64, queue_size=10)
    '''
    Publishes the intial position
    '''
    def update(self):
        self.x_pub.publish(self.x)
        self.y_pub.publish(self.y)
        self.z_pub.publish(self.z)
        self.th_pub.publish(self.th)
class k_model:
    '''
    Starts the init_pub class for map merge
    @param x: Intial x position
    @param y: Intial y position
    @param th: Intial yaw position
    @param prefix: Namespace of the robot
    '''
    def __init__(self, prefix = "", x = 0.0, y = 0.0, th = 0.0):
        self.x = x
        self.y = y
        self.th = th
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0
        if prefix != "":
            prefix = "/" + prefix
        self.prefix = prefix

        self.init_pos = init_publisher(x=x,y=y,th=th,prefix=prefix)

        rospy.init_node('puzzlebot_deadReckoning')
        self.pub_odom = rospy.Publisher('{}/odom'.format(self.prefix), Odometry, queue_size=10)
        self.wl_sub = rospy.Subscriber('{}/wl'.format(self.prefix), Float32, self.wr_cb)
        self.wr_sub = rospy.Subscriber('{}/wr'.format(self.prefix), Float32, self.wl_cb)

    
    '''
    Callback for the /wr topic
    @param msg: Float32 message
    '''
    def wr_cb(self, msg):
        self.wr = msg.data
    '''
    Callback for the /wl topic
    @param msg: Float32 message
    '''
    def wl_cb(self, msg):
        self.wl = msg.data
    
    def run(self):
        try:
            # Create the variables
            dt = 0.1
            past_t = rospy.Time.now()
            self.init_pos.update()
            rate = rospy.Rate(7)
            rate.sleep()
            while not rospy.is_shutdown():
                self.init_pos.update()
                # Use kinematic model
                now_t = rospy.Time.now()
                dt = (now_t - past_t).to_sec()
                self.w = -R * (self.wr - self.wl) / L
                self.v = R * (self.wr + self.wl) * 0.5
                
                self.th += self.w * dt
                self.x += self.v * np.cos(self.th) * dt
                self.y += self.v * np.sin(self.th) * dt
                # Create message
                o = Odometry()
                o.header.frame_id = "{}/odom".format(self.prefix)
                o.child_frame_id = "{}/base_footprint".format(self.prefix)
                o.header.stamp = rospy.Time.now()
                o.pose.pose.position.x = self.x
                o.pose.pose.position.y = self.y
                quat = Quaternion(*quaternion_from_euler(0,0,self.th))
                o.pose.pose.orientation = quat
                o.twist.twist.linear.x = self.v
                o.twist.twist.angular.z = self.w
                self.pub_odom.publish(o)
                past_t = now_t
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == "__main__":
    # Read parameters
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
    model = k_model(p, x = x, y = y, th = t)
    model.run()




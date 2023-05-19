#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
L = 0.191
R = 0.05


class tf_model:
    def __init__(self, prefix = ""):
        self.x = 0.0
        self.y = 0.0
        self.quat = Quaternion(*quaternion_from_euler(0,0,0))
        self.R = R
        self.prefix = prefix
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber("/scan_lag",LaserScan, self.scan_cb)
        self.pub_scan = rospy.Publisher("/scan",LaserScan,queue_size=10)
        #self.publish_static()
        self.tf_broadcaster = tf.TransformBroadcaster()
    def scan_cb(self,msg):
        m = LaserScan()
        m = msg
        m.header.stamp = rospy.Time.now()
        self.pub_scan.publish(m)
    
    def pose_cb(self, msg):
        self.x = msg.pose.position.x 
        self.y = msg.pose.position.y 
        self.quat = msg.pose.orientation
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y 
        self.quat = msg.pose.pose.orientation
    
    def run(self):
        dt = 0.1
        rate = rospy.Rate(1/dt)
        while not rospy.is_shutdown():
        
            pose_trans = TransformStamped()
            pose_trans.header.stamp = rospy.Time.now()
            pose_trans.header.frame_id = "odom"
            pose_trans.child_frame_id = self.prefix + "base_footprint"

            pose_trans.transform.translation.x = self.x
            pose_trans.transform.translation.y = self.y
            pose_trans.transform.translation.z = 0
            pose_trans.transform.rotation = self.quat
            self.tf_broadcaster.sendTransformMessage(pose_trans)

            pose_trans = TransformStamped()
            pose_trans.header.stamp = rospy.Time.now()
            pose_trans.header.frame_id = "base_footprint"
            pose_trans.child_frame_id = self.prefix + "base_link"

            pose_trans.transform.translation.x = 0
            pose_trans.transform.translation.y = 0
            pose_trans.transform.translation.z = self.R
            pose_trans.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransformMessage(pose_trans)
            #Gazebo
            gaz_trans = TransformStamped()
            gaz_trans.header.stamp = rospy.Time.now()
            gaz_trans.header.frame_id = "map"
            gaz_trans.child_frame_id = self.prefix + "odom"

            gaz_trans.transform.translation.x = 0
            gaz_trans.transform.translation.y = 0
            gaz_trans.transform.translation.z = 0
            gaz_trans.transform.rotation.w = 1.0
            #self.tf_broadcaster.sendTransformMessage(gaz_trans)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('puzzlebot_tf')
    model = tf_model()
    model.run()
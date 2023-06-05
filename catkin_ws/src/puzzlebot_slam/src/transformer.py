#!/usr/bin/env python3
'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Publoshes transforms
@author Diego Reyna Reyes
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''
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
    '''
    Starts the init_pub class for map merge
    @param prefix: Namespace of the robot
    '''
    def __init__(self, prefix = ""):
        self.x = 0.0
        self.y = 0.0
        self.quat = Quaternion(*quaternion_from_euler(0,0,0))
        self.R = R
        if prefix != "":
            prefix = "/" + prefix
        self.prefix = prefix
        self.sub_odom = rospy.Subscriber('{}/odom'.format(self.prefix), Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber("{}/scan_lag".format(self.prefix),LaserScan, self.scan_cb)
        self.pub_scan = rospy.Publisher("{}/scan".format(self.prefix),LaserScan,queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
    '''
    Republishes the scan with the actual time stamp
    '''   
    def scan_cb(self,msg):
        m = LaserScan()
        m = msg
        m.header.stamp = rospy.Time.now()
        self.pub_scan.publish(m)
    
    '''
    Callback for the /odom topic
    @param msg: Odometry message
    '''
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y 
        self.quat = msg.pose.pose.orientation
    
    def run(self):
        dt = 0.1
        rate = rospy.Rate(1/dt)
        while not rospy.is_shutdown():
            # Create tf
            pose_trans = TransformStamped()
            pose_trans.header.stamp = rospy.Time.now()
            pose_trans.header.frame_id = self.prefix + "/odom"
            pose_trans.child_frame_id = self.prefix + "/base_footprint"

            pose_trans.transform.translation.x = self.x
            pose_trans.transform.translation.y = self.y
            pose_trans.transform.translation.z = 0
            pose_trans.transform.rotation = self.quat
            self.tf_broadcaster.sendTransformMessage(pose_trans)

            pose_trans = TransformStamped()
            pose_trans.header.stamp = rospy.Time.now()
            pose_trans.header.frame_id = self.prefix + "/base_footprint"
            pose_trans.child_frame_id = self.prefix + "/base_link"

            pose_trans.transform.translation.x = 0
            pose_trans.transform.translation.y = 0
            pose_trans.transform.translation.z = self.R
            pose_trans.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransformMessage(pose_trans)
            rate.sleep()

if __name__ == "__main__":
    try:
        p = rospy.get_param('puzzlebot_tf/prefix_robot')
    except:
        p = ""
    rospy.init_node('puzzlebot_tf')
    model = tf_model(p)
    model.run()
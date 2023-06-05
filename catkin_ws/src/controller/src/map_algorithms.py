#!/usr/bin/env python3
'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Creates navigation goals based on the map
@author Diego Reyna Reyes
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''
import numpy as np
import cv2
import rospy
import tf
import matplotlib.pyplot as plt
import heapq
import random
from djikstra import find_shortest_path
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_multiply
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import Int16, Float32, Bool

class mapping():
    '''
    Starts the map analyzer class
    @param prefix: Namespace of the robot
    '''
    def __init__(self, prefix = ""):
        # Map messages
        self.occupancy_grid = None
        self.info = None
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        # Objective
        self.x_obj = 0.0
        self.y_obj = 0.0
        self.new_obj = False
        # Path
        self.path = []
        self.path_num = 0
        if prefix != "":
            prefix = "/" + prefix
        self.prefix = prefix
        

        self.listener = tf.TransformListener()

        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.pub_count = rospy.Publisher("/brain/map_count", Int16, queue_size=10)
        self.pub_debug = rospy.Publisher("brain/debug", Float32, queue_size=10)
        self.pub_goal = rospy.Publisher('/brain/map_objective', PointStamped, queue_size=10)
        self.cl_sub = rospy.Subscriber('/clicked_point',PointStamped,self.obj_cb)
        self.ob_sub = rospy.Subscriber('/brain/main_objective', PointStamped, self.obj_cb)
        self.pub_obj = rospy.Publisher('/brain/final_obj', PointStamped, queue_size=10)
    '''
    Callback for the /odom topic
    @param msg: Odometry message
    '''
    def callback_odom(self,msg):
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
        # Transform pose
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
    '''
    Callback for receiving objectives
    @param msg: PointStamped message
    '''
    def obj_cb(self,msg):
        self.x_obj = msg.point.x
        self.y_obj = msg.point.y
        self.path_num = len(self.path)
        self.new_obj = True
    '''
    Callback for the /map topic
    @param msg: Occupancy Grid message
    '''
    def callback_map(self, msg):
        self.info = msg.info
        # Convert the data to a numpy array
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    '''
    Creates a path using dijikstra
    '''
    def getPath(self):
        x_map = int((self.x-self.info.origin.position.x) / self.info.resolution)
        y_map = int((self.y-self.info.origin.position.y) / self.info.resolution)

        xo_map = int((self.x_obj-self.info.origin.position.x) / self.info.resolution)
        yo_map = int((self.y_obj-self.info.origin.position.y) / self.info.resolution)
        
        map = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1])).copy()
        print(np.unique(map))
        map[map == -1] = 100
        map[map == 100] = 100
        map[map == 0] = 1
        print(np.unique(map))
        map = map.astype(np.uint8)
        start = (y_map, x_map)
        end = (yo_map, xo_map)
        return find_shortest_path(map, start, end)
    '''
    Creates the points that the robot should follow
    '''
    def create_path(self):
        p = self.getPath()
        p.reverse()
        print(p)
        self.path = []
        self.path_num = 0
        for i,node in enumerate(p):
            if i % 5 == 0:
                # Convert to world coordinates
                point = PointStamped()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = "map"
                point.point.x = node[1]*self.info.resolution + self.info.origin.position.x
                point.point.y = node[0]*self.info.resolution + self.info.origin.position.y
                self.path.append(point)
        node = p[-1]
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = node[1]*self.info.resolution + self.info.origin.position.x
        point.point.y = node[0]*self.info.resolution + self.info.origin.position.y
        self.path.append(point)
    '''
    Keeps following the path
    '''
    def follow_path(self):
        dist = np.sqrt((self.x - self.x_obj)**2 + (self.y - self.y_obj)**2)
        if len(self.path) == 0 or dist < 0.1 or self.path_num == len(self.path):
            if self.new_obj:
                self.create_path()
                print(self.path)
                self.new_obj = False
        if len(self.path) > 0 and self.path_num < len(self.path):
            x_act = self.path[self.path_num].point.x
            y_act = self.path[self.path_num].point.y
            dist = np.sqrt((self.x - x_act)**2 + (self.y - y_act)**2)
            
            self.pub_goal.publish(self.path[self.path_num])
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.point.x = self.x_obj
            point.point.y = self.y_obj
            # Publish
            self.pub_obj.publish(point)
            if dist < 0.4:
                self.path_num += 1
    '''
    Goes to the closest empty space to the objective
    '''
    def go_to(self):
        print(self.x_obj, self.y_obj)
        xo_map = int((self.x_obj-self.info.origin.position.x) / self.info.resolution)
        yo_map = int((self.y_obj-self.info.origin.position.y) / self.info.resolution)
        self.position = (yo_map,xo_map)
        map = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1])).copy()
        
        # Get the closest one inside of the ones in border
        indices = np.array(np.where(map == 0))
        distances = np.sqrt((indices[0] - self.position[0])**2 + (indices[1] - self.position[1])**2)

        idx = np.argmin(distances)
        [y_obj, x_obj] = indices[:,idx]

        x_obj = x_obj*self.info.resolution + self.info.origin.position.x
        y_obj = y_obj*self.info.resolution + self.info.origin.position.y
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = x_obj
        point.point.y = y_obj
        print(x_obj, y_obj)
        # Publish
        self.pub_goal.publish(point)
        self.new_obj = False
if __name__ == "__main__":
    rospy.init_node("Map_analyser")
    try:
        p = rospy.get_param('puzzlebot_odom/prefix_robot')
    except:
        p = ""
    
    m = mapping()
    rate = rospy.Rate(10)
    print("Innit")
    while not rospy.is_shutdown():
        m.go_to()

        rate.sleep()
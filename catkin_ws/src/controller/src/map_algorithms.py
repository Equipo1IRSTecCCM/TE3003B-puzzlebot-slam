#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
import tf
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import Int16, Float32

class mapping():
    def __init__(self):
        self.occupancy_grid = None
        self.info = None
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.x_obj = 0.0
        self.y_obj = 0.0
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.pub_count = rospy.Publisher("/brain/map_count", Int16, queue_size=10)
        self.pub_debug = rospy.Publisher("brain/debug", Float32, queue_size=10)
        self.pub_goal = rospy.Publisher('/brain/map_objective', PointStamped, queue_size=10)
        self.pub_pose = rospy.Publisher('/move_base_simple/goal', PoseStamped,queue_size=10)
    def callback_odom(self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.th=euler[2]
    def callback_map(self, msg):
        self.info = msg.info
        # Convert the data to a numpy array
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    def getFinish(self):
        try:
            img = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1],1)).copy()
            img2 = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1],1)).copy()
        except:
            return
        #Get all the borders
        img[img == 100] = 255
        img[img == 0] = 100
        img[img == -1] = 50
        #Get walls
        img2[img2 == 0] = 0
        img2[img2 == 100] = 50
        img2[img2 == -1] = 0
        
        img = img.astype(np.uint8)
        front = cv2.Canny(img,50,150)

        img2 = img2.astype(np.uint8)
        front_wo = cv2.Canny(img2,50,150)
        #Combine so we get the border of free spaces next to the unknown
        front_wo = cv2.blur(front_wo, (10,10))
        front_wo[front_wo > 10] = 255
        front_wo[front_wo <= 10] = 0
        border = cv2.bitwise_and(front,cv2.bitwise_not(front_wo))
        #Get number of pixels that are still missing to explore
        count = border[border > 0].shape[0]
        # if count < 5:
        #     point = PointStamped()
        #     point.header.stamp = rospy.Time.now()
        #     point.header.frame_id = "map"
        #     point.point.x = self.x
        #     point.point.y = self.y
        #     #Publish
        #     self.pub_goal.publish(point)
        self.pub_count.publish(count)
        dist = np.sqrt((self.x_obj - self.x)**2 + (self.y_obj - self.y)**2)
        if dist < 1:
            self.getObjective(border)
        else:
            #Convert to world coordinates
            point = PointStamped()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = "map"
            point.point.x = self.x_obj
            point.point.y = self.y_obj
            #Publish
            self.pub_goal.publish(point)
    def getObjective(self, border):
        #Get the pixel of the map we're on
        x_map = int((self.x-self.info.origin.position.x) / self.info.resolution)
        y_map = int((self.y-self.info.origin.position.y) / self.info.resolution)
        self.position = (y_map,x_map)

        #Get the closest one inside of the ones in border
        indices = np.array(np.where(border == 255))
        distances = np.sqrt((indices[0] - self.position[0])**2 + (indices[1] - self.position[1])**2)
        if len(distances) == 0:
            self.x_obj = self.x
            self.y_obj = self.y
            return
        idx = np.argmax(distances)
        [self.y_obj, self.x_obj] = indices[:,idx]
        #Get the orientation
        direction = cv2.Laplacian(border,cv2.CV_64F)
        d = np.arctan(direction[self.y_obj,self.x_obj])
        self.pub_debug.publish(d)
        quat = Quaternion(*quaternion_from_euler(0,0,self.th))
        self.pub_debug.publish(d)
       
        #Create msgs
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        #Convert to world coordinates
        pose.pose.position.x = self.x_obj*self.info.resolution + self.info.origin.position.x
        pose.pose.position.y = self.y_obj*self.info.resolution + self.info.origin.position.y
        # pose.pose.orientation.x = quat[0]
        # pose.pose.orientation.y = quat[1]
        # pose.pose.orientation.z = quat[2]
        pose.pose.orientation = quat
        #Publish
        self.pub_pose.publish(pose)
        
        self.x_obj = pose.pose.position.x
        self.y_obj = pose.pose.position.y
        

if __name__ == "__main__":
    rospy.init_node("Map_analyser")
    map_msg = rospy.wait_for_message('/map', OccupancyGrid)
    m = mapping()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.getFinish()
        rate.sleep()
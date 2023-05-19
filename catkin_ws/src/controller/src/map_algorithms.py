#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16

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
        self.pub_count = rospy.Publisher("/brain/map_count",Int16, queue_size=10)
        self.pub_goal = rospy.Publisher('/brain/map_objective',PointStamped, queue_size=10)
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
        self.pub_count.publish(count)
        dist = np.sqrt((self.x_obj - self.x)**2 + (self.y_obj - self.y)**2)
        if dist < 0.5:
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
        idx = np.argmax(distances)
        [self.y_obj, self.x_obj] = indices[:,idx]

        #Convert to world coordinates
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "map"
        point.point.x = self.x_obj*self.info.resolution + self.info.origin.position.x
        point.point.y = self.y_obj*self.info.resolution + self.info.origin.position.y
        #Publish
        self.pub_goal.publish(point)
        self.x_obj = point.point.x
        self.y_obj = point.point.y
        

if __name__ == "__main__":
    rospy.init_node("Map_analyser")
    map_msg = rospy.wait_for_message('/map', OccupancyGrid)
    m = mapping()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.getFinish()
        rate.sleep()
#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
import smach
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int16

class mapping():
    def __init__(self):
        self.occupancy_grid = None
        self.info = None
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.pub_count = rospy.Subscriber("/brain/map_count",Int16, queue_size=10)

    def callback_map(self, msg):
        self.info = msg.info
        # Convert the data to a numpy array
        self.occupancy_grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    def getFinish(self):
        try:
            img = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1],1)).copy()
            img2 = np.reshape(self.occupancy_grid,(self.occupancy_grid.shape[0],self.occupancy_grid.shape[1],1)).copy()

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
            
        except:
            return
        
if __name__ == "__main__":
    rospy.init_node("Map_analyser")
    m = mapping()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        m.getFinish()
        rate.sleep()
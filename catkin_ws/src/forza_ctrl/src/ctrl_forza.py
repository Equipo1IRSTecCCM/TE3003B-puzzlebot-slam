#!/usr/bin/env python3
'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Allows driving the puzzlebot with an xbox controller
@author Diego Reyna Reyes
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np

class ctrl:
    '''
    Starts the ctrl class
    '''
    def __init__(self) -> None:
        self.go = False
        self.linear = 0.0
        self.angular = 0.0
        rospy.init_node("ForzaCtrl")
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.first_r = True
        self.first_l = True
    '''
    Read the control input
    '''  
    def joy_cb(self, msg):
        # Trigger button
        self.go = msg.buttons[0] == 1.0
        self.linear = 0.0
        self.angular = 0.0
        # Read the bumpers and create linear and angular speed
        if self.go:
            r_t = msg.axes[5]
            if self.first_r:
                if r_t == 0.0:
                    r_t = 0.0
                else:
                    self.first_r = False
            else:
                if r_t == 1.0:
                    r_t = 0.0
                else:
                    r_t -= 1.0
                    r_t = -r_t
                    r_t /= 2.0
            l_t = msg.axes[2]
            if self.first_l:
                if l_t == 0.0:
                    l_t = 0.0
                else:
                    self.first_l = False
            else:
                if l_t == 1.0:
                    l_t = 0.0
                else:
                    l_t -= 1.0
                    l_t = -l_t
                    l_t /= 2.0         

            self.linear = r_t - l_t

            self.angular = msg.axes[0]
            self.angular *= 0.5
        else:
            self.linear = 0.0
            self.angular = 0.0
    '''
    Send the command
    '''  
    def run(self):
        rate = rospy.Rate(10)
        increase = 0.05
        l = 0.0
        while not rospy.is_shutdown():
            # Create a proportional control
            t = Twist()
            if abs(l - self.linear) > increase:
                if l < self.linear:
                    if l < 0:
                        l += increase * 2
                    else:
                        l += increase
                else:
                    l -= increase * 3
            if self.linear == 0 and abs(l) < 0.07:
                l = 0.0
            t.linear.x = l
            adjst_a = abs(l / 1.0) * 3.0
            if adjst_a < 1:
                adjst_a = 1.0
            t.angular.z = self.angular * adjst_a 

            self.pub_cmd.publish(t)
            rate.sleep()
if __name__ == "__main__":
    c = ctrl()
    c.run()


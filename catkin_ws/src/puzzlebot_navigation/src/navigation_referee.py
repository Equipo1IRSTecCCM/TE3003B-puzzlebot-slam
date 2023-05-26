#!/usr/bin/env python3

#Arbitro
import math
import numpy as np
import rospy
import smach
import smach_ros
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Int16
from std_msgs.msg import Bool
from variables import *


rospy.init_node('CesarArturoRamos') 
base_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
c2e = []
e2c = []
n_checks = 5
x = 0.0
y = 0.0
th = 0.0
obj_x = 0.0
obj_y = 0.0
count_map = 0
map_done = False
finished = False


def newOdom (msg):
    global x
    global y
    global th
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    th=euler[2]
sub_odom = rospy.Subscriber("/odom",Odometry,newOdom)




class Laser():
    u"""Class that handles laser information"""

    def __init__(self):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb (self, msg):
        # Laser scan callback function
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the laser value"""
        return self._scan_data
laser = Laser()  #instanciamos una clase 
scan_msg = rospy.wait_for_message('/scan', LaserScan)




def update_c2e(data):
    global c2e
    if len(c2e) >= n_checks:
        c2e[:n_checks-1,:] = c2e[1:n_checks,:]
        c2e[-1] = data
    else:
        c2e.append(data)
        if len(c2e) == n_checks:
            c2e = np.array(c2e)
def get_c2e():
    global c2e
    if len(c2e) >= n_checks:
        return (np.std(c2e[:,0]) + np.std(c2e[:,1]))/2
    else:
        return np.inf
def get_e2c():
    global e2c
    if len(e2c) >= n_checks:
        return (np.std(e2c[:,0]) + np.std(e2c[:,1]))/2
    else:
        return np.inf
def update_e2c(data):
    global e2c
    if len(e2c) >= n_checks:
        e2c[:n_checks-1,:] = e2c[1:n_checks,:]
        e2c[-1] = data
    else:
        e2c.append(data)
        if len(e2c) == n_checks:
            e2c = np.array(e2c)


go = False
def get_go(msg):
    global go
    go = msg.data


std_dev = 0.0
def get_std(msg):
    global std_dev
    std_dev = msg.data
    


class TwistSub:
    def __init__(self):
        self.data = Twist()
    def callback(self, msg):
        self.data = msg



def get_obj(msg):
    global obj_x
    global obj_y

    obj_x = msg.point.x
    obj_y = msg.point.y

def get_count(msg):
    global count_map
    global map_done
    count_map = msg.data
    if not map_done and count_map < 30:
        map_done = True

def get_fh(msg):
    global finished
    finished = finished or msg.data

def get_fm(msg):
    global finished
    finished = finished or msg.data
    
x = 0
y = 0
th = 0



thr = 0.01
sub_go = rospy.Subscriber('nav/go',Bool,get_go)
# sub_obj = rospy.Subscriber('/objective',PointStamped,get_obj)
sub_std = rospy.Subscriber('/nav/std',Float32,get_std)
pub_restart = rospy.Publisher('/nav/restart',Bool,queue_size=10)
sub_finish_he = rospy.Subscriber('/he/finish',Bool,get_fh)
sub_finish_mm = rospy.Subscriber('/mm/finish',Bool,get_fm)



rate = rospy.Rate(20)


class Campos(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['estancado','todoOk'])
        self.cmd_camp = TwistSub()
        self.sub_camp= rospy.Subscriber("/cmd_vel_camp",Twist,self.cmd_camp.callback)
    def execute(self, userdata):
        global go
        global std_dev
        global base_vel_pub
        global x
        global y
        global c2e
        global e2c
        global rate
        global finished
        print('Executing state Campos')
        if not finished:
            base_vel_pub.publish(self.cmd_camp.data)
        rate.sleep()
        if go:
            if std_dev < thr:
                pub_restart.publish(True)
                update_c2e(np.array([x,y]))
                std_c2e = get_c2e()
                std_e2c = get_e2c()
                return 'estancado'
        return 'todoOk'




class Evasion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['liberado','todoOk'])
        self.cmd_eva = TwistSub()
        self.sub_eva = rospy.Subscriber("/cmd_vel_eva",Twist,self.cmd_eva.callback)

    def execute(self, userdata):
        print('Executing state Evasion')
        global rate
        global go
        global std_dev
        global base_vel_pub
        global x
        global y
        global c2e
        global e2c
        global finished
        if not finished:
            base_vel_pub.publish(self.cmd_eva.data)
        rate.sleep()
        if go:
            if std_dev > thr * 1.5:
                pub_restart.publish(True)
                update_e2c(np.array([x,y]))
                std_c2e = get_c2e()
                std_e2c = get_e2c()
                return 'liberado'
        return 'todoOk'

#QUe cuando /brain/map_count < 30 dejar de publicar
#En coolab_slam.launch incluir rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map world



def main():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Camp', Campos(), 
                               transitions={'estancado':'Evasion', 
                                            'todoOk':'Camp'})
        smach.StateMachine.add('Evasion', Evasion(), 
                               transitions={'liberado':'Camp', 
                                            'todoOk':'Evasion'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    sis.stop()

if __name__ == '__main__':
    main()



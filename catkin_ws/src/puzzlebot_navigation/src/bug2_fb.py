#!/usr/bin/env python3

'''
TE3003B - Integración de robótica y sistemas inteligentes
CRALIOS - Collaborative Robots Assembly Line for Irregular Objects using SLAM
Follows a wall using the LIDAR
@author Jorge Gerardo Iglesias Ortiz
@date 4/06/2023
Mexico City, Mexico
ITESM CCM
'''
import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from smach import State, StateMachine
import smach_ros
from tf.transformations import euler_from_quaternion

class ControlPID:
    def __init__(self, rate, kp = 1, ki = 0, kd = 0):
        self.rate = rate
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0
        self.ea = 0
        self.pred = 0

    def update(self, current, goal):
        ep = goal - current
        self.i = self.i + self.ki*ep*self.rate
        ed = (ep - self.ea)/self.rate
        self.ea = ep
        self.pred = current + ep*self.kp + self.i + ed*self.kd

pos = Twist()
c_vel = Twist()
pub = rospy.Publisher('/wall/cmd_vel',Twist,queue_size=10)
rospy.init_node('rodeo')
rate_dur = 10
rate = rospy.Rate(rate_dur)
rate_dur = 1/rate_dur

nfb = False
ilb = False
sl = 0
turnPID = ControlPID(rate_dur,kp = 0.3)
fowaPID = ControlPID(rate_dur,kp = 1,kd = 0.005)

def smallest_angle_diff(t,s):
    a = t - s
    a -= 2*np.pi if a > np.pi else -2*np.pi if a < -np.pi else 0
    return a

def dis(src, goal):
    return np.sqrt((goal[0]-src[0])**2 + (goal[1]-src[1])**2)

def callbackScan(msg):
    global nfb
    global ilb
    global sl

    #Get angles array
    scan = np.array(msg.ranges)
    ang = msg.angle_min - np.pi
    andf = msg.angle_max - np.pi
    inc = msg.angle_increment
    angles = np.arange(ang,andf+inc,inc)

    #Get only front detected points
    scan_f = scan[np.r_[0:90]]
    scan_f = scan_f[~np.isnan(scan_f)]
    scan_f = scan_f[np.isfinite(scan_f)]

    #Get only left detected points
    anglesl = angles[np.r_[-180:-90]]
    scan_l = scan[np.r_[-180:-90]]
    anglesl = anglesl[~np.isnan(scan_l)]
    scan_l = scan_l[~np.isnan(scan_l)]
    anglesl = anglesl[np.isfinite(scan_l)]
    scan_l = scan_l[np.isfinite(scan_l)]

    #Get booleans if obstacle in front and wall in its left
    nfb = not scan_f[scan_f < 0.3].shape[0] > 0 and not scan_l[scan_l < 0.25].shape[0] > 0
    #Y el 0.65 cambiar
    ilb = scan_l[scan_l < 0.35].shape[0] > 0

    #Calculate slope with regression and give it as error
    if scan_l[scan_l < 1.25].shape[0] >= 13:
        px = scan_l*np.cos(anglesl)
        py = scan_l*np.sin(anglesl)
        sl = np.sum((px - np.average(px))*(py -np.average(py)))/np.sum((px - np.average(px))**2)

def callbackGoalPoint(msg):
    global puntos
    puntos.append([msg.pose.position.x, msg.pose.position.y])

class Turn_so(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global rate_dur
        global nfb
        global ilb
        global c_vel
        
        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, -np.pi/16)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/16 else (np.pi/16)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.1 else 0.1*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()
            
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

class AdvanceTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        global c_vel

        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, np.pi/16)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/16 else (np.pi/16)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0.1)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.1 else 0.1*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()
            
        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

class Advance_so(State):
    def __init__(self):
        State.__init__(self, outcomes=['a','at','t'])
    def execute(self, ud):
        global pub
        global rate
        global nfb
        global ilb
        global sl
        global c_vel

        global turnPID
        global fowaPID
        
        vel = Twist()
        turnPID.update(c_vel.angular.z, sl)
        vel.angular.z = turnPID.pred
        vel.angular.z = vel.angular.z if abs(vel.angular.z) <= np.pi/16 else (np.pi/16)*np.sign(vel.angular.z)
        fowaPID.update(c_vel.linear.x, 0.05)
        vel.linear.x = fowaPID.pred
        vel.linear.x = vel.linear.x if abs(vel.linear.x) <= 0.05 else 0.05*np.sign(vel.linear.x)
        pub.publish(vel)
        rate.sleep()

        if not nfb:
            return "t"
        elif ilb:
            return "a"
        else:
            return "at"

def main():
    
    scanS = rospy.Subscriber('/scan',LaserScan,callbackScan)
    Gpoints = rospy.Subscriber('/move_base_simple/goal',PoseStamped,callbackGoalPoint)
    
    sm = StateMachine(outcomes=['succeeded'])
    sm.userdata.sm_input = 0

    with sm:

        StateMachine.add('TURN_so', Turn_so(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so'})
        StateMachine.add('ADVANCE_so', Advance_so(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so'})
        StateMachine.add('ADVANCETURN', AdvanceTurn(), transitions={'t':'TURN_so','at':'ADVANCETURN','a':'ADVANCE_so'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    rospy.sleep(1)
    sis.start()
    
    outcome = sm.execute()

    sis.stop()
    
    rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass



#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from AIControl import AIControl
import math

class PingerMission(object):
    def __init__(self):
        print 'pinger init'
        ### service hydrophone ###
        # rospy.wait_for_service('hydro')
        # self.srvPing = rospy.ServiceProxy('hydro',hydro_info)
        # print 'set up service complete'
        self.aicontrol = AIControl()
        # self.data=hydro_msg()
        self.hy = hydro_msg()
        self.check = Bool
        ### subscribe hydrophone ###
        rospy.Subscriber ('/hydro', hydro_msg, self.listening)
        rospy.Subscriber ('/controller/is_at_fix_orientation', Bool, self.stop_turn)

    def listening(self, data):
        self.hy = data

    def stop_turn(self, data):
        self.check = data
    
    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        return real

    def diff(sefl, now, want):
        if abs(want - now) > math.pi:
            return abs(want - now) - 2*math.pi
        else:
            return abs(want - now)

    def ping_check(self):
        print "listen hydrophone"
        goal = False
        goal_yaw = -99999
        rospy.sleep(5)
        
        state = self.aicontrol.get_pose()
        my_yaw = state[5]
        real_degree = self.convert(self.hy.azi)
        print self.hy.azi
        print real_degree

        self.aicontrol.turn_yaw_relative (real_degree)
        goal_yaw = real_degree/180 * math.pi + my_yaw
       
        while goal != True:

            print 'listen pinger'
            
            state = self.aicontrol.get_pose()
            my_yaw = state[5]
            print '*****************' 

            # if abs(self.diff(my_yaw,goal_yaw)) < 0.15:
            if self.check:
                real_degree = self.convert(self.hy.azi)
                print self.hy.azi
                print 'real_degree'
                print real_degree
                
                if real_degree > -10 and real_degree < 10:
                    self.aicontrol.drive_x (1)  ### control distance by azi || elv
                    print 'drive'
                    rospy.sleep(5)
                else:
                    goal_yaw = real_degree/180 * math.pi + my_yaw
                    print abs(self.diff(my_yaw, goal_yaw)) 
                    self.aicontrol.turn_yaw_relative (real_degree)
                    print 'turn'
                print'***********************'
            else:
                print 'yung mai tung'
                print my_yaw
                print goal_yaw
                
            print self.hy
            rospy.sleep (5)

            if self.hy.elv < 35:
                goal = True

            ##### add stop state 
               
    def run(self):

        self.aicontrol.stop(2)
        print 'stop to listen pinger'
        self.ping_check()
        print 'above pinger'
        self.aicontrol.drive_z(-0.05)
        print 'pinger mission complete'

if __name__=='__main__':
    print 'hydrophone'
    # rospy.init_node('ping_ai')
    ping=PingerMission()
    ping.run()

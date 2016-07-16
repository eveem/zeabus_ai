#!/usr/bin/env python

import rospy
from std_msgs.msg import String
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

        ### subscribe hydrophone ###
        rospy.Subscriber ('/hydro', hydro_msg, self.listening)
        
    def listening(self, data):
        print data

    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        # real = azi
        # if(real > 0):
        #     real -= 45
        # else :
        #     real +=
        return real

    def ping_check(self):
        print "listen hydrophone"
        goal = False
        goal_yaw = -99999
        
        state = self.aicontrol.get_pose()
        my_yaw = state[5]
        # pinger = self.srvPing(True)
        # pinger = pinger.hydro
        
        real_degree = self.convert(pinger.azi)
        print pinger.azi
        print real_degree
        self.aicontrol.turn_yaw_relative (real_degree)
        
        while goal != True:
            print 'listen pinger'
            
            state = self.aicontrol.get_pose()
            my_yaw = state[5]

            if abs(my_yaw - goal_yaw) > 0.05:
                pinger = self.srvPing(True)
                pinger = pinger.hydro
                real_degree = self.convert(pinger.azi)
                print pinger.azi
                print real_degree
                goal_yaw = real_degree/180 * math.pi + my_yaw
                
                if real_degree > -10 and real_degree < 10:
                    self.aicontrol.drive_x (1)
                    rospy.sleep(2)
                else:
                    self.aicontrol.turn_yaw_relative (real_degree)
                print'***********************'
            else:
                print 'yung mai tung'
                print my_yaw
                print goal_yaw
            # print self.data.elv
            print pinger

            # if pinger.stop:

                # if real_degree > -3 and real_degree < 3:
                    # while True:
                    # self.aicontrol.drive ([1,0,0,0,0,0])
                    # rospy.sleep(6)
                        # if real_degree > 175 and real_degree < -175:
                        #     print 'stop because azi'
                        #     self.aicontrol.stop(20)
                        #     goal = True
                # else:
                    
                    # rospy.sleep(0.5)
                    # self.aicontrol.stop(4)
            # else:
            #     self.aicontrol.stop(6)
            #     self.aicontrol.drive ([0.2,0,0,0,0,0])
            #     rospy.sleep(0.5)
            
            # if pinger.distance == -999:
            #     self.aicontrol.drive ([0.2,0,0,0,0,0])
            #     rospy.sleep(0.2)

            # if pinger.elv > -10 and pinger.elv < 10:
            #     print 'stop because elv'
            #     self.aicontrol.stop(20)
            #     goal = True
               
    def run(self):
        # self.aicontrol.stop()
        print 'stop to listen pinger'
        self.ping_check()
        print 'above pinger'
        self.aicontrol.drive_z(0)
        print 'pinger mission complete'

if __name__=='__main__':
    print 'hydrophone'
    rospy.init_node('ping_ai')
    ping=PingerMission()
    ping.run()

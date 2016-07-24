#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from zeabus_vision_bin.srv import Bin_Srv
from zeabus_vision_bin.msg import Bin_Msg
from AIControl import AIControl
from binn import BinnMission
from sett import SettMission
import math

class PingerMission(object):
    def __init__(self):
        print 'pinger init'
        self.aicontrol = AIControl()
        self.hy = hydro_msg()
        self.check = Bool
        ### subscribe hydrophone ###
        rospy.Subscriber ('/hydro', hydro_msg, self.listening)

        #### BINN
        ## subscribe vision
        srv_name = 'bin_srv'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect_binn = rospy.ServiceProxy(srv_name, Bin_Srv)

        #### SETT
        ## subscribe vision
        sett_srv = 'setcourse_srv'
        rospy.wait_for_service(sett_srv)
        print 'service starts top srv'
        self.detect_sett = rospy.ServiceProxy(sett_srv, SetCourse_Srv)

        binn_mission = BinnMission()
        sett_mission = SettMission()

    def listening(self, data): #### call back
        self.hy = data

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
        self.aicontrol.stop (5)

        real_degree = self.convert(self.hy.azi)
        print self.hy.azi
        print real_degree

        # dis = 3
        dis = 0.5
        self.aicontrol.turn_yaw_relative (real_degree)

        goal = False

        while goal != True:
            print 'listen pinger'
            state = self.aicontrol.get_pose()
            my_yaw = state[5]
            print '*****************'

            if self.aicontrol.stop_turn():
                real_degree = self.convert(self.hy.azi)
                print self.hy.azi
                print 'real_degree'
                print real_degree
                print 'stop status'
                print self.aicontrol.stop_turn()
                if real_degree > -10 and real_degree < 10:
                    self.aicontrol.drive_x (dis)  ### control distance by azi || elv
                    print 'drive'
                    rospy.sleep(1)
                else:
                    self.aicontrol.turn_yaw_relative (real_degree)
                    print 'turn'
                print'***********************'
            else:
                print 'yung mai tung'

            print self.hy
            rospy.sleep (5)

            if self.hy.elv < 50:
                dis = 0.2
            if self.hy.elv < 25:
                goal = True
            ##### add stop state

    def find_binn(self):
        count = 10

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            binn_data = self.detect_binn(String('bin'),String('white'))
            binn_data = binn_data.data
            print binn_data

            if len(binn_data.appear) == 1 or len(binn_data.appear) == 2:
                print 'found binn'
                return True
            else:
                print 'not found'
                count -= 1
                self.aicontrol.drive_x (0.2)
            rospy.sleep(0.25)

        return False

    def run(self):
        self.aicontrol.stop(2)
        self.aicontrol.drive_z (-1.7)
        print 'stop to listen pinger'
        self.ping_check()
        print 'above pinger'

        count = 20
        domiss = False

        if self.find_binn():
            print 'will do binn'
            self.do_binn()
        else:
            rospy.sleep(20)
            if self.hy.elv > 25:
                self.ping_check()
                self.find_binn()
                self.do_binn()
            else:
                self.aicontrol.stop(10)
                print 'FINISH !!'
        print 'pinger mission complete'

    def do_binn(self):
        binn_mission.run(0)
        sett_mission.run()

if __name__=='__main__':
    print 'hydrophone'
    # rospy.init_node('ping_ai')
    ping=PingerMission()
    ping.run()

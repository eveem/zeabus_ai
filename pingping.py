#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from zeabus_hydrophone.srv import hydro_info
from zeabus_hydrophone.msg import hydro_msg
from zeabus_vision_bin.srv import Bin_Srv
from zeabus_vision_bin.msg import Bin_Msg
from zeabus_vision_setcourse.srv import SetCourse_Srv
from zeabus_vision_setcourse.msg import SetCourse_Msg
from AIControl import AIControl
from binn import BinnMission
from sett import SettMission
import depth as const
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
        print 'service starts binn'
        self.detect_binn = rospy.ServiceProxy(srv_name, Bin_Srv)

        #### SETT
        ## subscribe vision
        sett_srv = 'setcourse_srv'
        rospy.wait_for_service(sett_srv)
        print 'service starts top srv'
        self.detect_sett = rospy.ServiceProxy(sett_srv, SetCourse_Srv)

        self.got_data = False

    def reset(self):
        reset_hy = rospy.Publisher('/hydro_status', Bool, queue_size = 1)
        while reset_hy.get_num_connections() == 0:
            rospy.sleep (1)
        reset_hy.publish (True)

    def listening(self, data): #### call back
        self.got_data = True
        self.hy = data

    def convert(self, azi):
        print 'covert degree to turn'
        azi = -azi
        real = azi - 45
        if real < -180:
            real = real + 360
        return real

    def check_data(self):
        return self.got_data

    def ping_check(self):
        self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
        print "listen hydrophone"  
        self.aicontrol.stop (3)
        self.reset()
        self.aicontrol.stop (3)

        real_degree = self.convert(self.hy.azi)
        print self.hy.azi
        print real_degree

        dis = 3
        self.aicontrol.turn_yaw_relative (real_degree)

        goal = False
        count = 50

        while goal != True and not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            # if not self.got_data:
            #     self.aicontrol.drive_x (0.2) 
            print 'listen pinger'
            state = self.aicontrol.get_pose()
            my_yaw = state[5]
            print '*****************'

            if self.hy.distance != -999:

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
                        dis -= 0.2
                        if dis <= 0:
                            dis = 0.5
                        rospy.sleep(1)
                    else:
                        # if real_degree > 30:
                        #     real_degree = 30
                        # elif real_degree < -30:
                        #     real_degree = -30
                        self.aicontrol.turn_yaw_relative (real_degree)
                        print 'turn'
                    print'***********************'
                else:
                    print 'yung mai tung'

                print self.hy
                rospy.sleep (5)

                if self.hy.elv < 40:
                    dis = 0.5
                if self.hy.stop:
                    print self.hy.elv
                    goal = True
                ##### add stop state
            count -= 1
        return goal

    def find_binn(self):
        count = 10
        self.aicontrol.drive_z (const.BIN_DETECTING_DEPTH)
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            binn_data = self.detect_binn(String('bin'),String('white'))
            binn_data = binn_data.data
            print binn_data

            if len(binn_data.appear) == 1 or len(binn_data.appear) == 2:
                print 'found binn'
                self.do_binn()
                return True
            else:
                print 'not found'
                count -= 1
                self.aicontrol.drive_x (0.04)
        return False

    def run(self):
        self.aicontrol.stop(2)
        self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
        print 'stop to listen pinger'
        self.ping_check()
        print 'above pinger'

        count = 20

        if self.find_binn():
            print 'will do binn'
            self.aicontrol.stop(2)
            self.do_binn()
            self.aicontrol.turn_yaw_relative(90)
            rospy.sleep(30) #### switch pinger
            self.reset()
            self.ping_check()
            self.aicontrol.drive_z (const.PING_FLOATING_DEPTH)
            print 'FINISH !!'
        else:
            self.aicontrol.drive_x (-1)
            self.aicontrol.drive_z (const.PING_FLOATING_DEPTH)
            self.aicontrol.turn_yaw_relative (90)
            rospy.sleep(30)
            self.reset()
            if self.hy.elv > 25:
                self.aicontrol.drive_z (const.PING_DETECTING_DEPTH)
                self.ping_check()
                self.find_binn()
                self.do_binn()
            else:
                self.aicontrol.stop(2)
                print 'FINISH !!'
        print 'pinger mission complete'
        return

    def do_binn(self):
        binn_mission = BinnMission()
        binn_mission.run(0)

if __name__=='__main__':
    print 'hydrophone'
    # rospy.init_node('ping_ai')
    ping=PingerMission()
    ping.run()

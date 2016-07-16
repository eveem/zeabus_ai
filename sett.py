#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
from hardware import Hardware

class SettMission (object):

    def __init__ (self):
        print "Now do Set Course"
        #### SETT
        ## subscribe vision
        # top_srv = 'vision1'
        # rospy.wait_for_service(top_srv)
        # print 'service starts top srv'
        # self.detect_sett = rospy.ServiceProxy(top_srv, vision_srv)
        #### SETT
        self.aicontrol = AIControl()
        self.hw = Hardware()

    def find_sett (self):
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count)
            sett_data = self.sett_bouy(String('small'))
            sett_data = sett_data.data
            print sett_data
            if len(sett_data.appear) == 4:
                self.aicontrol.stop(2)
                if sett_data.y[0] > sett_data.y[1]:
                    vz = sett_data.y[0]
                    vy = sett_data.x[0]
                else:
                    vy = sett_data.x[1]
                    vz = sett_data.y[1]
                self.aicontrol.drive_y (vy)
                self.aicontrol.drive ([0,0,vz,0,0,0])

            if len(sett_data.appear) == 1:
                if sett_data.area > 500:
                    self.hw.command ()


    def run (self, search):
        for i in xrange(2):
            find_sett (search[i])

if __name__ == '__main__':
    sett_mission = SettMission()
    #command
    sett_mission.run(['N','W'])
    print "finish bin"

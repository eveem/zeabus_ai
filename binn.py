#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String,Bool
from zeabus_vision_bin.srv import Bin_Srv
from zeabus_vision_bin.msg import Bin_Msg
from zeabus_vision_setcourse.srv import SetCourse_Srv
from zeabus_vision_setcourse.msg import SetCourse_Msg
from AIControl import AIControl
from hardware import Hardware
from sett import SettMission
import depth as const

class BinnMission (object):

    def __init__ (self):
        print "Now do bin"
        #### BINN
        ## subscribe vision
        srv_name = 'bin_srv'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect_binn = rospy.ServiceProxy(srv_name, Bin_Srv)
        #### BINN

        #### SETT
        ## subscribe vision
        sett_srv = 'setcourse_srv'
        rospy.wait_for_service(sett_srv)
        print 'service starts top srv'
        self.detect_sett = rospy.ServiceProxy(sett_srv, SetCourse_Srv)
        #### SETT

        self.aicontrol = AIControl()
        self.hw = Hardware()

    def getdata (self):
        binn_data = self.detect_binn(String('bin'),String('white'))
        binn_data = binn_data.data
        return binn_data

    def run (self, cover): # if cover = 1, uncover = 0
        print 'Go to bin'
        count = 50
        self.aicontrol.drive_z (const.BIN_DETECTIING_DEPTH)
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            print 'in while'
            binn_data = self.getdata()

            i = -1

            if len(binn_data.appear) == 2:
                if not binn_data.cover[0]:
                    i = 0
                else:
                    i = 1
            elif len(binn_data.appear) == 1 and binn_data.cover[0]:
                i = 0
            else:
                self.aicontrol.stop(1)
                print 'not found'
                count -= 1

            if i != -1:
                if self.aicontrol.is_center ([binn_data.x[i],binn_data.y[i]],-0.05,0.05,-0.05,0.05) and -3 < binn_data.angle[i] < 3:
                    print 'Center'
                    print binn_data.x[i]
                    print binn_data.y[i]
                    self.aicontrol.drive_z (const.BIN_DROP_DEPTH)   ##### DEPTH !!!
                    rospy.sleep (1)
                    break
                elif self.aicontrol.is_center ([binn_data.x[i],binn_data.y[i]],-0.05,0.05,-0.05,0.05):
                    self.aicontrol.turn_yaw_relative (binn_data.angle[i])
                    rospy.sleep (1)
                else :
                    print 'Not Center'
                    vx = binn_data.x[i]
                    vy = binn_data.y[i]
                    self.aicontrol.drive ([vx,vy,0,0,0,0])
                    rospy.sleep(0.5)

            rospy.sleep(0.25)

        '''
        if cover == 1:
            # self.hw.command ('gripper', 'grab')   ### grab
            self.aicontrol.drive_z (-2)           ### up -> open binn ###
            self.aicontrol.drive ([0,1,0,0,0,0])  ### move -> drop cover ###
            rospy.sleep(0.1)
            # self.hw.command ('gripper', 'leave')  ### leave cover alone
            self.aicontrol.drive ([0,-1,0,0,0,0]) ### move back to above bin ###
            rospy.sleep(0.1)
            self.aicontrol.drive_z (-2.8)
        '''

        ## drop x2 times
        # self.hw.command('drop_left', 'drop')
        self.aicontrol.stop (5)
        print 'drop laew eiei'
        rospy.sleep(1)
        # self.hw.command('drop_right', 'close')
        rospy.sleep(1)
        print 'drop marker yet'
        print 'bin complete'

        self.find_sett()
        sett_mission = SettMission()
        sett_mission.run ()

    def find_sett (self):
        self.aicontrol.drive_z (const.SET_DETECTING_DEPTH) #### CHANGE ME !!!
        count = 80
        print 'set course'
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            sett_data = self.detect_sett(String('setcourse'),String('big'))
            sett_data = sett_data.data
            print sett_data
            
            if len(sett_data.appear) == 1:
                print 'FOUND SETT !!'
                break
            else:
                self.aicontrol.turn_yaw_relative(-5)
                rospy.sleep(2)
                self.aicontrol.stop(1)
        return

if __name__ == '__main__':
    binn_mission = BinnMission()
    #command
    #binn_mission.run(## 0 or 1##)
    print "finish bin"

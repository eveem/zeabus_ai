#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_bin.srv import Bin_Srv
from zeabus_vision_bin.msg import Bin_Msg
from AIControl import AIControl
from hardware import Hardware

class BinnMission (object):

    def __init__ (self):
        print "Now do bin"
        ## subscribe vision
        srv_name = 'bin_srv'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect_binn = rospy.ServiceProxy(srv_name, Bin_Srv)
        ## old vision
        self.aicontrol = AIControl()
        # self.hw = Hardware()
        self.aicontrol.drive ([1,0,0,0,0,0])
        rospy.sleep(0.1)

    def getdata (self):
        binn_data = self.detect_binn(String('bin'),String('white'))
        binn_data = binn_data.data
        return binn_data

    def run (self, cover): # if cover = 1, uncover = 0
        print 'Go to bin'
        count = 100

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            print 'in while'
            binn_data = self.getdata()

            if binn_data.appear :
                print 'found'
                if self.aicontrol.is_center ([binn_data.x,binn_data.y],-0.1,0.1,-0.1,0.1):
                    print 'Center'
                    # self.aicontrol.drive_z (-2.8)   ##### DEPTH !!!
                    self.aicontrol.turn_yaw_relative(binn_data.angle)
                else :
                    print 'Not Center'
                    vx = binn_data.x
                    vy = binn_data.y
                    self.aicontrol.drive ([vx,vy,0,0,0,0])
            else :
                print 'not found'
                count -= 1
                degree = [90, -180, -90, 180]
                
                for i in range(4):
                    self.aicontrol.turn_yaw_relative (degree[i])
                    self.aicontrol.stop(2)
                    rospy.sleep (5)
                    object_data = self.getdata()
                    if object_data.appear :
                        break

            rospy.sleep(0.25)

        if cover == 1:
            # self.hw.command ('gripper', 'grab')   ### grab
            self.aicontrol.drive_z (-2)           ### up -> open binn ###
            self.aicontrol.drive ([0,1,0,0,0,0])  ### move -> drop cover ###
            rospy.sleep(0.1)    
            # self.hw.command ('gripper', 'leave')  ### leave cover alone
            self.aicontrol.drive ([0,-1,0,0,0,0]) ### move back to above bin ###
            rospy.sleep(0.1)
            self.aicontrol.drive_z (-2.8)

        ## drop x2 times
        # self.hw.command('drop_left', 'drop')
        rospy.sleep(0.5)
        # self.hw.command('drop_right', 'close')
        rospy.sleep(0.5)
        print 'drop marker yet'
        print 'bin complete'

if __name__ == '__main__':
    binn_mission = BinnMission()
    #command
    #binn_mission.run(## 0 or 1##)
    print "finish bin"

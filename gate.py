#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl
import depth as const
import direction as tis

class GateMission (object):

    def __init__ (self):
        print "Now do gate"
        #### PATH
        ## subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts bot srv'
        self.detect_path = rospy.ServiceProxy(bot_srv, vision_srv)
        #### PATH

        #### GATE
        ## subscribe vision
        # gate_srv = 'vision1'
        # rospy.wait_for_service(gate_srv)
        # print 'service starts gate srv'
        # self.detect_gate = rospy.ServiceProxy(gate_srv, vision_srv)
        #### GATE

        self.aicontrol = AIControl()

    def run (self, quarter):
        print 'drive z'
        self.aicontrol.drive_z (const.GATE_PASS_DEPTH)
        self.aicontrol.drive_z (const.GATE_PASS_DEPTH)
        self.aicontrol.drive_z (const.GATE_PASS_DEPTH)


        # if quarter == 'D' and quarter == 'A':
        #     mul = 1
        # else:
        #     mul = -1

        # self.aicontrol.drive_y (2*mul)
        # self.aicontrol.drive_x (5)
        # self.aicontrol.turn_yaw_relative (15*mul)
        # while not self.aicontrol.stop_turn():
        #     rospy.sleep(0.1)
        # self.aicontrol.drive_x (5)

        # self.aicontrol.drive_x (8)
        # self.aicontrol.drive_z (const.PATH_DETECTING_DEPTH)

        self.aicontrol.turn_yaw_absolute (tis.GATE_DIRECTION)
        rospy.sleep (2)
        print 'tuer yaw absolute complete'
        self.aicontrol.drive_x (8)

        # count = 50
        # while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
        #     print 'GATE DATA'
        #     gate_data = self.detect_gate(String('gate'), String('orange'))
        #     gate_data = gate_data.data
        #     print gate_data
        #     print '********'
        #     if gate_data.appear:
        #         print 'found gate'
        #         if self.aicontrol.is_center([gate_data.x,gate_data.y],-0.05,0.05,-0.05,0.05):
        #             self.aicontrol.drive_x (3)
        #             print 'forward'
        #             break
        #         else:
        #             self.aicontrol.drive ([0,gate_data.x,gate_data.y,0,0,0])
        #             rospy.sleep (1)
        #     else:
        #         print 'not found'
        #         self.aicontrol.drive_x (0.2)
        #         count -= 1

        found = False
        count = 12
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

            print 'PATH DATA'
            path_data = self.detect_path(String('path1'), String('orange'))
            path_data = path_data.data
            print path_data
            print '--------'

            if path_data.appear :
                print 'found path'
                self.aicontrol.stop(2)
                found = True
                break
            else:
                print 'not found'
                self.aicontrol.drive_x (0.5)
                count -= 1

        if self.aicontrol.is_fail(count):
            # self.aicontrol.turn_yaw_absolute (tis.PATH_ONE_DIRECTION)
            self.aicontrol.drive_x (1)
            self.aicontrol.drive_y (1)
            path_data = self.detect_path(String('path1'),String('orange'))
            path_data = path_data.data
            print path_data
            if path_data.appear:
                found = True
            if not found:
                self.aicontrol.drive_y(-2)
                path_data = self.detect_path(String('path1'),String('orange'))
                path_data = path_data.data
                print path_data
                if path_data.appear:
                    found = True

        if self.aicontrol.is_fail(count):
            return found
        else:
            return found


if __name__ == '__main__':
    print 'start gate'
    gate_mission = GateMission()
    #command
    gate_mission.run_without_vision()
    print "finish gate"

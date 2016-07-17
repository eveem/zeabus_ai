#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl

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
        # top_srv = 'vision1'
        # rospy.wait_for_service(top_srv)
        # print 'service starts top srv'
        # self.detect_gate = rospy.ServiceProxy(top_srv, vision_srv)
        #### GATE

        self.aicontrol = AIControl()

    def run (self):
        self.aicontrol.drive_z (-1)
        move = 10
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            # gate_data = self.detect_gate(String('gate'), String('orange'))
            # gate_data = gate_data.data
            # print gate_data
            # print '********'

            print 'PATH DATA'
            path_data = self.detect_path(String('path1'), String('orange'))
            path_data = path_data.data
            print path_data
            print '--------'
            '''
            if gate_data.appear :
                if self.aicontrol.is_center([gate_data.x,gate_data.y],-0.2,0.2,-0.2,0.2):
                    print 'center'
                    self.aicontrol.drive_x (1)
                    self.aicontrol.drive ([1,0,0,0,0,0])
                    rospy.sleep (4)
                    print 'forward'
                    break
                else :
                    print 'not center'
                    self.aicontrol.drive ([gate_data.x,gate_data.y,0,0,0,0])
                    rospy.sleep(0.25)
            '''
            if path_data.appear :
                print 'found path'
                self.aicontrol.stop(2)
                break
            else:
                print 'not found'
                if move != 0:
                    self.aicontrol.drive_x (0.2)
                    rospy.sleep (0.5)
                    move -= 1
                print 'cmd_vel gogo'
        return

if __name__ == '__main__':
    print 'start gate'
    gate_mission = GateMission()
    #command
    gate_mission.run_without_vision()
    print "finish gate"

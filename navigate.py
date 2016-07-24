#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv
from zeabus_vision_srv.msg import vision_msg
from AIControl import AIControl

class NavigateMission (object):

    def __init__ (self):
        print "Now do Navigation Channel"
        #### NAVIGATE
        ## subscribe vision
        nav_srv = 'vision1'
        rospy.wait_for_service(nav_srv)
        print 'service starts navigate srv'
        self.detect_nav = rospy.ServiceProxy(nav_srv, vision_srv)

        ### subscribe vision
        bot_srv = 'vision2'
        rospy.wait_for_service(bot_srv)
        print 'service starts nav bot'
        self.bot_nav = rospy.ServiceProxy(bot_srv, vision_srv)

        self.aicontrol = AIControl()

    def run (self):
        self.aicontrol.drive_z (-1.1)
        print 'run in navigate'
        count = 100

        # while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
        #     print 'in while'
        #     nav_data = self.detect_nav(String('navigate'),String('yellow'))
        #     nav_data = nav_data.data
        #     print nav_data

        #     # bot_data = self.bot_nav(String('navigate'),String('yellow'))
        #     # bot_data = bot_data.data
        #     # print bot_data

        #     if nav_data.appear :
        #         print 'found'
        #         vy = nav_data.x
        #         vz = nav_data.y
        #         print nav_data

        #         if nav_data.area > 500 : ### near ###
        #             print 'near'
        #             bc = 0.06
        #         else : ### far ###
        #             print 'far'
        #             bc = 0.1

        #         if self.aicontrol.is_center([nav_data.x,0],-bc,bc,-bc,bc) :
        #             # self.aicontrol.drive ([0.5,0,0,0,0,0])
        #             print 'center'
        #             self.aicontrol.drive_z (-0.5)
        #             self.aicontrol.drive_x (0.05)
        #             bot_data = self.bot_nav(String('navigate'),String('yellow'))
        #             bot_data = bot_data.data
        #             print bot_data
        #             while not bot_data.appear: 
        #                 self.aicontrol.drive_x (0.05)
        #                 bot_data = self.bot_nav(String('navigate'),String('yellow'))
        #                 bot_data = bot_data.data
        #                 print bot_data
        #             print 'will stop'
        #             # self.aicontrol.turn_yaw_relative (bot_data.angle)
        #             self.aicontrol.stop(1)
        #             self.aicontrol.drive_x (-0.3)
        #             self.aicontrol.drive_z (-1.25) #### CHANGE ME !! ####
        #             self.aicontrol.stop(1)
        #             print 'stop wait to roll'
        #             break
        #         else :
        #             self.aicontrol.drive ([0,vy,0,0,0,0])
        #             print 'not center'
        #     else :
        #         print 'not found'
        #         # self.aicontrol.drive ([0.2,0,0,0,0,0])
        #         rospy.sleep(1)
        #         count -= 1
        #     rospy.sleep (0.25)
        ### end while ###

        print 'see portal'
        self.aicontrol.stop(3)
        self.aicontrol.drive_z (-1.15)
        self.aicontrol.drive_x (-1)
        print 'drive z'
        print 'drive style'
        ### style ###
        self.aicontrol.roll(2)
        self.aicontrol.stop (3)
        print 'finish navigation channel'
        return

if __name__ == '__main__':
    navigate_mission = NavigateMission()
    navigate_mission.run()
    print "finish Navigation Channel"

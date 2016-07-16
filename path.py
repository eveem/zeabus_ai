#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import vision_srv,Boom_Srv
from zeabus_vision_srv.msg import vision_msg,Boom_Msg
from AIControl import AIControl

class PathMission (object):

    def __init__ (self):
        print "Mission : Path"
        #### PATH
        ## subscribe vision
        path_srv = 'vision2'
        rospy.wait_for_service(path_srv)
        print 'service starts path srv'
        self.detect_path = rospy.ServiceProxy(path_srv, vision_srv)
        #### PATH

        #### BOUY
        ## subscribe vision
        # bouy_srv = 'find_obj'
        # rospy.wait_for_service(bouy_srv)
        # print 'servive starts bouy srv'
        # self.detect_bouy = rospy.ServiceProxy(bouy_srv, Boom_Srv)
        #### BOUY

        #### NAV
        ## subscribe vision
        nav_srv = 'vision1'
        rospy.wait_for_service(nav_srv)
        print 'servive starts nav srv'
        self.detect_nav = rospy.ServiceProxy(nav_srv, vision_srv)
        #### NAV

        self.aicontrol = AIControl()
        self.angle=0

    def goto_path (self, n_path):

        self.aicontrol.drive_z(-1)
        print 'Go to Path'
        count = 100

        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            path_data = self.detect_path(String(n_path),String('orange'))
            path_data = path_data.data
            print 'PATH DATA'
            print path_data

            if path_data.appear :
                print 'found'

                if self.aicontrol.is_center([path_data.x,path_data.y],-0.2,0.2,-0.2,0.2):
                    self.angle = path_data.angle
                    print self.angle
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    self.aicontrol.drive ([path_data.x/2,path_data.y/2,0,0,0,0])
            else :
                print 'not found'
                # self.aicontrol.drive ([0,0,0.5,0,0,0])
                self.aicontrol.stop(0.25)
                count -= 1

        if self.aicontrol.is_fail(count) :
            print 'Find Path Fail'
            return False

        print 'Find Path Complete'
        return True

    def run(self, obj):
        print 'Start Path Mission'
        if obj == 'bouy':
            n_path = 'path1'
        elif obj == 'navigate':
            n_path = 'path2'
        if(self.goto_path(n_path)):
            print 'turn_yaw'
            print self.angle
            self.aicontrol.stop(2)
            self.aicontrol.turn_yaw_relative(self.angle)
            rospy.sleep(6)
            self.aicontrol.drive_x (0.5)
            rospy.sleep(0.5) ##### change distance
            print 'drive_x after found part'
        else :
        #     self.aicontrol.goto(0.076,0.004,-1.5)
        #     if(self.goto_path()):
        #         self.aicontrol.turn_yaw_relative(self.angle)
        #         print 'Path finish'
        #     else :
        #         print 'Path Fail'
            self.aicontrol.drive ([1,0,0,0,0,0])
            rospy.sleep (2)
        # if obj == 'bouy':
        #     count = 50
        #     self.aicontrol.drive_z (-3)
        #     while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):

        #         red_bouy = self.detect_bouy(String('bouy'),String('red'))
        #         red_bouy = red_bouy.data
        #         rospy.sleep(2)
        #         green_bouy = self.detect_bouy(String('bouy'),String('green'))
        #         green_bouy = green_bouy.data
        #         rospy.sleep(2)
        #         # yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
        #         # yellow_bouy = yellow_bouy.data
        #         print 'RED BOUY DATA'
        #         print red_bouy
        #         print 'GREEN BOUY DATA'
        #         print green_bouy
        #         # print 'YELLOW BOUY DATA'
        #         # print yellow_bouy
        #         if red_bouy.appear:
        #             print 'RED FOUND BOUY'
        #             return 
        #             break
        #         elif green_bouy.appear:
        #             print 'GREEN FOUND BOUY'
        #             return
        #             break
        #         else:
        #             self.aicontrol.drive ([0.2,0,0,0,0,0])
        #             rospy.sleep (0.3)
        #             print 'forward'
        #             count -= 1
            # while not red_bouy.appear and not green.appaer:
            #     self.aicontrol.turn_yaw_relative (10)
            #     rospy.sleep (2)

        if obj == 'navigate':
            nav_data = self.detect_nav(String('navigate'),String('yellow'))
            nav_data = nav_data.data
            print 'NAVIGATE DATA'
            print nav_data
            count = 50
            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
                if nav_data.appear:
                    print 'Found Navigate'
                    return 'nav'
                    break
                else:
                    print 'not found navigate'
                    self.aicontrol.drive_x (0.2)
                    rospy.sleep (0.3)
                    count -= 1
        self.aicontrol.stop(2)
        return


if __name__ == '__main__':
    path_mission = PathMission()
    #command
    path_mission.run()

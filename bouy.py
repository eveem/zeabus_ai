#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from zeabus_vision_srv.srv import Boom_Srv,vision_srv
from zeabus_vision_srv.msg import Boom_Msg,vision_msg
from AIControl import AIControl
from hardware import Hardware

class BouyMission (object):

    def __init__ (self):
        print "Now do bouy"
        print 'eiei'
        #### BOUY
        ## subscribe vision
        bouy_srv = 'find_obj' ### P'Ink service
        rospy.wait_for_service(bouy_srv)
        print 'service starts bouy srv'
        self.detect_bouy = rospy.ServiceProxy(bouy_srv, Boom_Srv)
        #### BOUY

        #### PATH
        path_srv = 'vision2'
        rospy.wait_for_service(path_srv)
        print 'service starts path srv'
        self.detect_path = rospy.ServiceProxy(path_srv, vision_srv)
        #### PATH

        # rospy.Subscriber ('/controller/is_at_fix_position', Bool, self.posi)
        # self.fin = False
        self.aicontrol = AIControl()
        self.hw = Hardware()

    def run (self):

        red_bouy = self.detect_bouy(String('bouy'),String('red'))
        red_bouy = red_bouy.data
        rospy.sleep(2)
        green_bouy = self.detect_bouy(String('bouy'),String('green'))
        green_bouy = green_bouy.data
        rospy.sleep(2)
        bouy_color = ['red', 'green', 'yellow']
        

        if green_bouy.appear:
            move = 0
            while red_bouy.appear == False and move != 10:
                # self.aicontrol.drive_y (0.2)
                self.aicontrol.drive ([0,0.6,0,0,0,0])
                rospy.sleep (2)
                red_bouy = self.detect_bouy(String('bouy'),String('red'))
                red_bouy = red_bouy.data
                rospy.sleep(2)
                move += 1
            if red_bouy.appear:
                print 'found red bouy'
            self.aicontrol.stop (2)

        for i in xrange(2):
            print 'will hit ' + bouy_color[i]
            count = 50
            max_area = 0
            
            self.aicontrol.drive_z (-3) ############# CHANGE ME !!!!
            print 'drive z to -3 complete'

            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
                now_pose = self.aicontrol.get_pose()

                bouy_data = self.detect_bouy(String('bouy'),String(bouy_color[i]))
                bouy_data = bouy_data.data
                print bouy_data

                if bouy_data.appear:
                    if bouy_data.value > max_area :
                        max_area = bouy_data.value

                    vx = (1/bouy_data.value)*500
                    vy = bouy_data.x
                    vz = bouy_data.y

                    if bouy_data.value > 400 : ### near ###
                        print 'near'
                        bc = 0.1
                        sr = 0.3
                    else : ### far ###
                        print 'far'
                        bc = 0.2
                        sr = 0.5

                    if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
                        print bouy_data
                        if bouy_data.value > 600: ### CHANGE ME !!!
                            print 'go to bouy'
                            print 'drive_x 3 meter'
                            self.aicontrol.drive_x (3)
                            rospy.sleep(5)
                            break
                        else:
                            print 'drive_x 1 meter so far'
                            self.aicontrol.drive_x (0.5)
                            rospy.sleep(5)
                    else :
                        self.aicontrol.drive([0,vy,vz,0,0,0])
                        print 'set to center'
                        rospy.sleep (sr)

                else :
                    self.aicontrol.drive_x (0.1)
                    rospy.sleep(1)
                    self.aicontrol.stop(0.2)
                    count -= 1
            ### end while ###

            if i == 0:
                self.aicontrol.stop (1)
                print 'stop state after hit bouy'
                print 'backward'

                print 'go to set point'
                self.aicontrol.drive_x (-4)
                rospy.sleep(5)
            
                green_bouy = self.detect_bouy(String('bouy'),String('green'))
                green_bouy = green_bouy.data
                rospy.sleep(2)
                while green_bouy.appear == False and move != 10:
                    # self.aicontrol.drive_y (-0.1)
                    self.aicontrol.drive ([0,-0.6,0,0,0,0])
                    rospy.sleep(3)
                    green_bouy = self.detect_bouy(String('bouy'),String('green'))
                    green_bouy = green_bouy.data
                    rospy.sleep(2)
                    move += 1
                rospy.sleep(3)
                print 'set point'
                print 'finish ' + bouy_color[i]
            
            elif i == 1:
                # self.hw.command('gripper', 'leave')
                yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
                yellow_bouy = yellow_bouy.data
                rospy.sleep(2)
                move = 0
                while yellow_bouy.appear == False and move != 10:
                    # self.aicontrol.drive_y (-0.1)
                    self.aicontrol.drive ([0,0.3,0,0,0,0])
                    rospy.sleep(1.5)
                    yellow_bouy = self.detect_bouy(String('bouy'),String('yellow'))
                    yellow_bouy = yellow_bouy.data
                    rospy.sleep(2)
                    move += 1

                self.aicontrol.drive_x (0.7)
                # self.hw.command('gripper', 'grab')    ##### grab
                self.aicontrol.drive_z (-3.5) #### CHANGE ME !!!!
                self.aicontrol.stop (10)
                # self.hw.command('gripper', 'leave')   ##### release
                self.aicontrol.drive ([0,1,0,0,0,0])
                rospy.sleep (5)
                print 'yellow complete'
                break
            ### end for ###
        print 'finish 3 bouy'
        # self.yellow_bouy ()
        self.find_path()

    def find_path (self):
        path_data = self.detect_path(String('path1'),String('orange'))
        path_data = path_data.data
        print path_data
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            if path_data.appear:
                print 'found path'
                self.aicontrol.stop(2)
                break
            else:
                count -= 1
        return

    # def yellow_bouy (self):
    #     print 'do yellow bouy'
    #     count = 50
    #     self.hw.command ('gripper', 'off')
    #     print 'open gripper'
    #     while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :

    #         bouy_data = self.detect_bouy(String('bouy'),String('yellow'))
    #         bouy_data = bouy_data.data
    #         print bouy_data

    #         if bouy_data.appear:
    #             vx = (1/bouy_data.value)*500
    #             vy = bouy_data.x
    #             vz = bouy_data.y

    #             if bouy_data.value > 400 : ### near ###
    #                 print 'near'
    #                 bc = 0.1
    #                 sr = 0.3
    #             else : ### far ###
    #                 print 'far'
    #                 bc = 0.2
    #                 sr = 0.5

    #             if self.aicontrol.is_center([bouy_data.x,bouy_data.y],-bc,bc,-bc,bc) :
    #                 print bouy_data
    #                 if bouy_data.value > 1000: ### change area value
    #                     print 'go to bouy'
    #                     print 'drive_x 3 meter'
    #                     self.aicontrol.drive_x (4)
    #                     self.hw.command ('gripper', 'on')
    #                     self.aicontrol.drive_z (-1.5) ####### CHANGE ME !!!
    #                     self.hw.command ('gripper', 'off')
    #                     # self.aicontrol.drive_y (0.2)
    #                     self.aicontrol.drive ([0,0.2,0,0,0,0])
    #                     rospy.sleep(2)
    #                     break
    #                 else:
    #                     print 'drive_x 1 meter so far'
    #                     self.aicontrol.drive_x (1)
    #                     rospy.sleep(0.5)
    #             else :
    #                 self.aicontrol.drive([0,vy,vz,0,0,0])
    #                 print 'set to center'
    #                 rospy.sleep (sr)
    #         else :
    #             self.aicontrol.stop(0.2)
    #             count -= 1
    #     ### end while ###

    #     self.aicontrol.stop (3)
    #     print 'stop state after hit bouy'

    #     self.find_path()

if __name__ == '__main__':
    bouy_mission = BouyMission()
    #command
    bouy_mission.red_then_green()
    # bouy_mission.yellow_bouy()
    print 'RED AND GREEN BOUY COMPLETE !!'

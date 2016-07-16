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
    
        self.aicontrol = AIControl()
        '''
        self.nav_pose = Pose()
        self.nav_pose.position.x = 
        self.nav_pose.position.y = 
        self.nav_pose.position.z =
        '''

    def run (self):
        self.aicontrol.drive_z (-1)
        print 'run in navigate'
        count = 100
        
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            print 'in while'
            nav_data = self.detect_nav('navigate','yellow')
            nav_data = nav_data.data
            print nav_data
            
            if nav_data.appear :
                print 'found'
                vy = nav_data.x
                vz = nav_data.y

                if nav_data.area > 500 : ### near ###
                    print 'near'
                    # vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.25, -0.1, 0.1, 0.25)
                    # vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.40, -0.2, 0.1, 0.30)
                    bc = 0.2
                else : ### far ###
                    print 'far'
                    # vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.35, -0.1, 0.1, 0.35)
                    # vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.50, -0.2, 0.1, 0.40)
                    bc = 0.4

                if self.aicontrol.is_center([nav_data.x,nav_data.y],-bc,bc,-bc,bc) :
                    self.aicontrol.drive ([(1/nav_data.value)*500,0,0,0,0,0])
                    print 'center'
                    self.aicontrol.drive_x (1)
                    print 'will stop'
                    self.aicontrol.stop(0.5)
                    self.aicontrol.drive_z (-0.3) #### CHANGE ME !! ####
                    self.aicontrol.stop(1)
                    print 'stop wait to roll'
                    break
                    # if object_data.value > 1000: ### very near ###
                    #     self.aicontrol.drive ([1,0,0,0,0,0])
                    #     rospy.sleep(0.5)
                    #     print 'will stop'
                    #     self.aicontrol.stop()
                    #     rospy.sleep(0.5)
                    #     self.aicontrol.drive_z (1.60)
                    #     break
                    # else :
                    #     self.aicontrol.drive ([0.5,0,0,0,0,0])
                    #     print 'forward'
                else :
                    self.aicontrol.drive ([0,vy,vz,0,0,0])
                    print 'not center'
            else :
                print 'not found'
                # self.aicontrol.drive ([0.2,0,0,0,0,0])
                count -= 1
            rospy.sleep (0.25)
        ### end while ###
        
        print 'see portal'
        # self.aicontrol.drive ([0,0,0.3,0,0,0])
        # rospy.sleep(1)
        # self.aicontrol.drvie_z ()
        print 'drive style'
        ### style ###
        # self.aicontrol.roll(2) 
        self.aicontrol.stop (20)
        print 'finish navigation channel'
        return

if __name__ == '__main__':
    navigate_mission = NavigateMission()
    navigate_mission.run()
    print "finish Navigation Channel"

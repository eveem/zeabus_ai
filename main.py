#!/usr/bin/env python

import rospy
import math
# from gate import GateMission
# from path import PathMission
# from bouy import BouyMission
# from navigate import NavigateMission
from pingping import PingerMission
# from binn import BinnMission
from AIControl import AIControl
from modbus_ascii_ros.msg import Switch

# check = False

# def start (switch):
#     global check
#     if switch.motor_switch == True:
#         check = True
#     else:
#         check = False

if __name__ == '__main__':

    # global check
    rospy.init_node('main_ai')
    print "init node complete"
    # rospy.Subscriber("/switch/data", Switch, start, queue_size = 10)
    # rospy.sleep (2)
    # while 
    # print 'DO AI EIEI YEAH !!'
    #### inherit phase
    # aicontrol = AIControl()
    pingping = PingerMission()
    pingping.listening()
    # gate_mission = GateMission()
    # path_mission = PathMission()
    # bouy_mission = BouyMission()
    # navigate_mission = NavigateMission()
    # print 'inherit complete'
    # gate_mission.run()
    # print 'GATE COMPLETE !!'
    # path_mission.run('bouy')
    # print 'PATH TO BOUY COMPLETE !!'
    # bouy_mission.run()
    # print 'BOUY COMPLETE !!'
    # path_mission.run('navigate')
    # print 'PATH TO NAVIGATE COMPLETE !!'
    # navigate_mission.run()
    # print 'NAVIGATE COMPLETE !!'
    # aicontrol.stop(10)
    # print 'FINISH !'

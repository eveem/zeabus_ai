#!/usr/bin/env python

import rospy
import math
from gate import GateMission
from path import PathMission
from bouy import BouyMission
from navigate import NavigateMission
from pingping import PingerMission
# from binn import BinnMission
# from sett import SettMission
from AIControl import AIControl
from modbus_ascii_ros.msg import Switch

check = False

def start (switch):
    global check
    if switch.motor_switch == False:
        check = False
        print 'Turn off switch'
        rospy.signal_shutdown('Turn off switch')
    else:
        check = True
        # print 'Switch on'

if __name__ == '__main__':

    # global check
    # rospy.sleep (10)
    # rospy.Subscriber("/switch/data", Switch, start, queue_size = 1)
    rospy.init_node('main_ai')
    print "init node complete"

    # print check
    # rospy.sleep (2)
    print 'DO AI EIEI YEAH !!'
    #### inherit phase
    # aicontrol = AIControl()
    # rospy.spin()
    # gate_mission = GateMission()
    # path_mission = PathMission()
    # bouy_mission = BouyMission()
    # navigate_mission = NavigateMission()
    pingping = PingerMission()
    # binn_mission = BinnMission()
    # sett_mission = SettMission()
    print 'inherit complete'
    # gate_mission.run()
    # print 'GATE COMPLETE !!'
    
    
    # if check:
    # path_mission.run('bouy')
    # print 'PATH TO BOUY COMPLETE !!'
    
    # bouy_mission.run()
    # print 'BOUY COMPLETE !!'
        
    # bouy_mission.find_path()
    # print 'FIND PATH TO NAV !!'

    # path_mission.run('navigate')
    # print 'PATH TO NAVIGATE COMPLETE !!'
        
    # navigate_mission.run()
    # print 'NAVIGATE COMPLETE !!'

    pingping.run()
    print 'PINGER FIND BINN FINISH !!'

    # pingping.run('octa')
    # print 'PINGER OCTAGON FINISH !!'

    # aicontrol.stop(3)
    # print 'FINISH !'
    
    # binn_mission.run(0)
    # sett_mission.run()
    
    # bouy_mission.yellow_bouy()
    
    # rospy.spin()
    # sett = SettMission()

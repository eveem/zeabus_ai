#!/usr/bin/env python

import rospy
import math
from gate import GateMission
from navigate import NavigateMission
from pingping import PingerMission
from AIControl import AIControl
from modbus_ascii_ros.msg import Switch
#### practice
from path_practice import PathMission
from bouy_practice import BouyMission
from binn_practice import BinnMission
from sett_practice import SettMission
#### practice
import depth as const
import direction as tis

def start (switch):
    global check
    global ever_turn_on
    if switch.motor_switch == False:
        check = False
        if ever_turn_on:
            rospy.signal_shutdown('Turn off switch')
    else:
        check = True
        ever_turn_on = True

if __name__ == '__main__':
    global check
    global ever_turn_on
    ever_turn_on = False
    rospy.Subscriber("/switch/data", Switch, start, queue_size = 1)
    rospy.init_node('main_ai')
    print "init node complete"

    print 'DO AI EIEI YEAH !!'
    #### inherit phase
    aicontrol = AIControl()
    gate_mission = GateMission()
    print '1'
    path_prac_mission = PathMission()
    print '2'
    bouy_prac_mission = BouyMission()
    print '3'
    navigate_mission = NavigateMission()
    print '4'
    binn_prac_mission = BinnMission()
    print '6'
    # sett_prac_mission = SettMission()
    # print '7'
    print 'inherit complete'

    while not check and not rospy.is_shutdown():
        rospy.sleep(0.1)

    #### FULL VERSION
    # path_one_found = gate_mission.run ('B')
    # if path_one_found:
    #     print 'CAN FOLLOW PATH'
    #     path_complete = path_prac_mission.run ()
       
    # else:
    #     print 'TURN TO FIND BOUY'
    #     rospy.sleep (2)
    #     aicontrol.turn_yaw_absolute (tis.BOUY_DIRECTION)
    
    # bouy_found = path_prac_mission.find_bouy ()

    # bouy_complete = bouy_prac_mission.run ('B')
    # if not bouy_complete:
    #     aicontrol.turn_yaw_absolute (const.PATH_TWO_DIRECTION)

    # bouy_prac_mission.find_path ()
    # path_prac_mission.run ()

    # nav_condition = path_prac_mission.find_nav ()
    # navigate_mission.run (nav_condition)

    # aicontrol.turn_yaw_absolute (const.OCTAGON_DIRECTION)
    # aicontrol.drive_x (7)

    # pingping.ping_check()
    # aicontrol.drive_z (const.PING_FLOATING_DEPTH)

    path_one_found = gate_mission.run ('B')
    if path_one_found:
        print 'CAN FOLLOW PATH'
        path_complete = path_prac_mission.run()
    else:
        print 'TURN TO FIND BOUY'
        aicontrol.turn_yaw_absolute (tis.BOUY_DIRECTION)
  
    aicontrol.drive_z (const.BOUY_DETECTING_DEPTH)
    aicontrol.drive_x (4)
    aicontrol.drive_x (-1)
    aicontrol.drive_y (-1.2)

    aicontrol.drive_z (-2.3)
    aicontrol.drive_x (1)
    
    aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
    aicontrol.drive_y (1.2)

    path_two_found = bouy_prac_mission.find_path ()
    if not path_two_found:
        print 'TURN TO FIND PATH'
        aicontrol.turn_yaw_absolute (tis.NAV_DIRECTION)

    nav_condition = path_prac_mission.find_nav ()
    navigate_mission.run (nav_condition)

    aicontrol.drive_z (const.PATH_DETECTING_DEPTH)
    aicontrol.turn_yaw_absolute (tis.OCTAGON_DIRECTION)
    aicontrol.drive_x (12)

    pingping = PingerMission()
    while not pingping.check_data():
        rospy.sleep(2)
        aicontrol.drive_x (0.2)
    
    pingping.run()


    aicontrol.stop (5)
    print 'FINISH !!!'















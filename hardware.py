#!/usr/bin/env python

import rospy
from modbus_ascii_ros.srv import IOCommand

class Hardware:

    def __init__ (self):
        rospy.init_node('hardware_service', anonymous=True)
        print 'init node hardware'

    def command (self, eq, status):
        print 'eiei'
        if status == 'on':
            srv_name = '/io_and_pressure/IO_ON'
        else:
            srv_name = '/io_and_pressure/IO_OFF'
        rospy.wait_for_service(srv_name)
        srv = rospy.ServiceProxy(srv_name, IOCommand)
        print 'service complete'
        ## gripper (on)  -> grab
        ## gripper (off) -> release
        ## fire (on)  -> fire
        ## reload when on and off it
        equipment = ['gripper', 'drop_left', 'drop_right', 'fire_left', 'fire_right']
        result = srv(equipment.index(eq))

        print result

if __name__ == '__main__':
    hw = Hardware()
    print 'hardware'
    # hw.command ('gripper', 'on')
    # hw.command ('drop_left', 'on')
    # hw.command ('gripper', 'off')
    # hw.command ('gripper', 'off')
    hw.command ('drop_right', 'off')
    hw.command ('drop_left', 'off')
    hw.command ('fire_right', 'off')
    hw.command ('fire_left', 'off')

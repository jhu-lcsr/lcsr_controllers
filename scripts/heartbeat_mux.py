#!/usr/bin/env python

import sys

import rospy

import sensor_msgs.msg
import std_msgs.msg

class HeartbeatMux(object):
    def __init__(self):
        self.buttons = rospy.get_param('~buttons',[])
        
        if len(self.buttons) == 0:
            rospy.logerr("No buttons designated to send heartbeats! Please set the ~buttons parameter to a list of button indices.")
            sys.exit(-1)

        self.sub = rospy.Subscriber('joy',sensor_msgs.msg.Joy,HeartbeatMux.joy_cb)
        self.pub = rospy.Publisher('heartbeats',std_msgs.msg.Empty)

    def joy_cb(self,msg):
        if any([msg.buttons[i] for i in self.buttons]):
            self.pub.publish(std_msgs.msg.Empty())

def main():
    rospy.init_node('heartbeat_mux')

    mux = HeartbeatMux()

    rospy.spin()

if __name__ == '__main__':
    main()

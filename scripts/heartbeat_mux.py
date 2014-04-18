#!/usr/bin/env python

import sys
import collections
import threading

import rospy

import sensor_msgs.msg
import std_msgs.msg

class HeartbeatMux(object):
    def __init__(self):
        self.buttons = rospy.get_param('~buttons',[])
        self.zombie_sequence = rospy.get_param('~zombie_sequence',[])

        self.zombie_buffer = collections.deque([-1]*len(self.zombie_sequence),len(self.zombie_sequence))
        
        if len(self.buttons) == 0:
            rospy.logerr("No buttons designated to send heartbeats! Please set the ~buttons parameter to a list of button indices.")
            sys.exit(-1)
        else:
            rospy.loginfo("Using buttons: " + str(self.buttons))

        self.sub = rospy.Subscriber('joy',sensor_msgs.msg.Joy,self.joy_cb)
        self.pub = rospy.Publisher('heartbeats',std_msgs.msg.Empty)
        self.zombie_thread = None
        self.zombie = False

    def joy_cb(self,msg):
        if any([msg.buttons[i]==1 for i in self.buttons]):
            self.pub.publish(std_msgs.msg.Empty())

        bids = [i for (i,v) in enumerate(msg.buttons) if v == 1]
        if len(self.zombie_sequence) > 0 and len(bids) > 0 and (len(self.zombie_buffer) == 0 or bids[0] != list(self.zombie_buffer)[-1]):
            self.zombie_buffer.append(bids[0])
            if all([b == s for (b,s) in zip(self.zombie_buffer,self.zombie_sequence)]):
                self.start_zombie()
            else:
                self.stop_zombie()

    def start_zombie(self):
        if self.zombie_thread is None:
            rospy.loginfo("Starting zombie heartbeat thread.")
            self.zombie = True
            self.zombie_thread = threading.Thread(target=self.zombie_loop)
            self.zombie_thread.start()

    def stop_zombie(self):
        if self.zombie_thread is not None:
            self.zombie = False
            rospy.loginfo("Stopping zombie heartbeat thread.")

    def zombie_loop(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown() and self.zombie:
            self.pub.publish(std_msgs.msg.Empty())
            r.sleep()
        rospy.loginfo("Zombie heartbeat thread stopped.")
        self.zombie_thread = None


def main():
    rospy.init_node('heartbeat_mux')

    mux = HeartbeatMux()

    rospy.spin()

if __name__ == '__main__':
    main()

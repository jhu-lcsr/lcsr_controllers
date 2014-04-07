#!/usr/bin/env python

import rospy
import actionlib
import sensor_msgs.msg

from urdf_parser_py.urdf import URDF

robot = URDF()
rate = None

def state_cb(msg):
    joint_pos_map = dict(zip(msg.name, msg.position))

    warn = False

    for joint in robot.joints:
        if joint.name in joint_pos_map:
            pos = joint_pos_map[joint.name]

            lower_dist = abs(pos - joint.limit.lower)
            upper_dist = abs(joint.limit.upper - pos)

            if lower_dist < 0.1:
                if lower_dist < 0.01:
                    rospy.logerr("Joint %s is at lower limit!" % (joint.name))
                else:
                    rospy.logwarn("Joint %s is within %0.3f rad of lower limit!" % (joint.name, lower_dist))
                warn = True
            elif upper_dist < 0.1:
                if upper_dist < 0.01:
                    rospy.logerr("Joint %s is at upper limit!" % (joint.name))
                else:
                    rospy.logwarn("Joint %s is within %0.3f rad of upper limit!" % (joint.name, upper_dist))
                warn = True

    if warn:
        rospy.logwarn("---")
        rate.sleep()

def main():
    global robot, rate
    rospy.init_node('joint_limit_watcher')

    rate = rospy.Rate(5)

    urdf = rospy.get_param('robot_description')

    robot = robot.from_xml_string(urdf)

    sub = rospy.Subscriber('joint_states',sensor_msgs.msg.JointState,state_cb, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()



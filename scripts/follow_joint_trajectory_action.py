#!/usr/bin/env python

import rospy
import actionlib
import control_msgs.msg

def main():
    rospy.init_node('follow_joint_trajectory_action')

    client = actionlib.SimpleActionClient(
            'action', control_msgs.msg.FollowJointTrajectoryAction)

    client.wait_for_server()

    rospy.loginfo("Connected to server.")

    goal = control_msgs.msg.FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()

    client.send_goal(goal)

    client.wait_for_result()

    print(str(client.get_result()))

if __name__ == '__main__':
    main()



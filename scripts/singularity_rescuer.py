#!/usr/bin/env python

from __future__ import print_function

import rospy
import actionlib

import PyKDL as kdl

from actionlib_msgs.msg import GoalStatus
import conman_msgs.msg as conman_msgs
import control_msgs.msg as control_msgs
import moveit_msgs.msg as moveit_msgs
import sensor_msgs.msg as sensor_msgs
import shape_msgs.msg as shape_msgs
import tf

from tf_conversions.posemath import fromTf, toMsg

from urdf_parser_py.urdf import URDF


class SingularityRescuer(object):
    
    # States
    NOMINAL=0
    PLANNING_ESCAPE=1
    SWITCHING_TO_JOINT_CONTROL=2
    ESCAPING=3
    SWITCHING_TO_CART_CONTROL=4

    def __init__(self):

        # state
        self.state = self.NOMINAL

        # error thresholds
        self.linear_err_threshold = rospy.get_param('~linear_err_threshold')
        self.angular_err_threshold = rospy.get_param('~angular_err_threshold')

        # lists of blocks needed for each control mode
        self.cart_blocks = rospy.get_param('~cart_blocks')
        self.joint_blocks = rospy.get_param('~joint_blocks')

        # frame names
        self.root_link = rospy.get_param('~root_link')
        self.tip_link = rospy.get_param('~tip_link')
        self.target_frame = rospy.get_param('~target_frame')

        # tf
        self.listener = tf.TransformListener()

        # state feedback
        self.joint_state_sub = rospy.Subscriber(
                'joint_state',
                sensor_msgs.JointState,
                self.joint_state_cb)

        # Motion planning action
        self.move_group = actionlib.SimpleActionClient(
                'move_group',
                moveit_msgs.MoveGroupAction)

        # Joint trajectory action
        self.follow_trajectory = actionlib.SimpleActionClient(
                'follow_trajectory_action',
                control_msgs.FollowJointTrajectoryAction)

        # Controller manager actions
        self.get_blocks = actionlib.SimpleActionClient(
                'get_blocks_action',
                conman_msgs.GetBlocksAction)
        self.set_blocks = actionlib.SimpleActionClient(
                'set_blocks_action',
                conman_msgs.SetBlocksAction)

        rospy.loginfo("Waiting to connect to action servers...")

        self.move_group.wait_for_server()
        self.follow_trajectory.wait_for_server()
        self.get_blocks.wait_for_server()
        self.set_blocks.wait_for_server()

        rospy.loginfo("Action clients created.")

    def joint_state_cb(self, joint_state):
        # compute joint limit proximity
        # TODO

        # get the current tip and target poses
        tip_pose, target_pose = self.get_tip_and_target(joint_state.header.stamp)

        if not tip_pose or not target_pose:
            return

        # check if planning is needed
        planning_needed = not self.within_tolerance(tip_pose, target_pose)

        # check if we're in the nominal state
        if self.state == self.NOMINAL:
            # if the cartesian error is high enough, request a motion plan
            if planning_needed:
                # update the state
                self.state=self.PLANNING_ESCAPE
                self.move_group_goal = self.generate_planning_goal(joint_state, tip_pose, target_pose)
                self.move_group.send_goal(
                        self.move_group_goal,
                        done_cb=self.planning_escape_done)

        elif self.state == self.PLANNING_ESCAPE:
            # if the cartesian error is small enough, preempt the motion plan request
            pass
        elif self.state == self.ESCAPING:
            # do nothing
            pass

    def planning_escape_done(self, goal_state, result):
        # check if the goal succeeded
        if goal_state != GoalStatus.SUCCEEDED:
            rospy.logerr("SingularityRescuer: failed to plan an escape: %s" % (result.error_code or 'UNKNOWN'))
            self.state = self.NOMINAL
            return

        rospy.loginfo("SingularityRescuer: escape plannned in %d seconds" % result.planning_time)

        # construct a joint trajectory goal
        self.joint_trajectory_goal = control_msgs.FollowJointTrajectoryGoal(
                trajectory=result.planned_trajectory.joint_trajectory)

        # check if the end-effector is still close to the starting point of the trajectory
        # TODO

        # switch the controllers out of cartesian mode and into joint mode
        self.state=self.SWITCHING_TO_JOINT_CONTROL
        self.switching_to_joint_control_goal = conman_msgs.SetBlocksGoal(
                disable=self.cart_blocks,
                enable=self.joint_blocks,
                diff=True,
                force=True)
        self.set_blocks.send_goal(
                self.switching_to_joint_control_goal,
                done_cb=self.switching_to_joint_control_done)

    def switching_to_joint_control_done(self, goal_state, result):
        # check if the goal succeeded
        if goal_state != GoalStatus.SUCCEEDED:
            rospy.logerr("SingularityRescuer: failed while switching to joint control.")
            self.state=NOMINAL
            return

        rospy.loginfo("SingularityRescuer: joint control active, attempting to escape")
        
        # update state
        self.state=self.ESCAPING
        # send the joint trajectory command
        self.follow_trajectory.send_goal(
                self.joint_trajectory_goal,
                done_cb=self.escaping_done)

    def escaping_done(self, goal_state, result):
        # check if the goal succeeded
        if goal_state != GoalStatus.SUCCEEDED:
            rospy.logerr("SingularityRescuer: failed while trying to escape.")
        else:
            # get the current tip and target poses
            tip_pose, target_pose = self.get_tip_and_target(rospy.Time.now())

            # check if we successfully escaped the singularity
            if self.within_tolerance(tip_pose, target_pose):
                rospy.loginfo("SingularityRescuer: Completed escape.")
            else:
                rospy.logwarn("SingularityRescuer: Could not  escape.")

        # switch out of joint mode and back into cartesian mode
        self.state=self.SWITCHING_TO_CART_CONTROL
        self.switching_to_cart_control_goal = conman_msgs.SetBlocksGoal(
                disable=self.joint_blocks,
                enable=self.cart_blocks,
                diff=True,
                force=True)
        self.set_blocks.send_goal(
                self.switching_to_cart_control_goal,
                done_cb=self.switching_to_cart_control_done)

    def switching_to_cart_control_done(self, goal_state, result):
        rospy.loginfo("SingularityRescuer: cart control active.")
        self.state=self.NOMINAL

    def get_tip_and_target(self, time):
        # get current and goal frames
        try:
            # lookup transform from base to the current tip pose
            self.listener.waitForTransform(
                    self.root_link,
                    self.tip_link,
                    time,
                    rospy.Duration(0.1))
            tip_pose = self.listener.lookupTransform(
                    self.root_link,
                    self.tip_link,
                    time)
            # lookup transform from base to the goal tip pose
            self.listener.waitForTransform(
                    self.root_link,
                    self.target_frame,
                    time,
                    rospy.Duration(0.1))
            target_pose = self.listener.lookupTransform(
                    self.root_link,
                    self.target_frame,
                    time)
        except (tf.Exception, tf.ConnectivityException, tf.ExtrapolationException):
            return (None, None)

        return (tip_pose, target_pose)

    def within_tolerance(self, tip_pose, target_pose):
        # compute cartesian command error
        tip_frame = fromTf(tip_pose)
        target_frame = fromTf(target_pose)

        twist_err = kdl.diff(tip_frame, target_frame)
        linear_err = twist_err.vel.Norm()
        angular_err = twist_err.rot.Norm()

        #print("linear: %g, angular %g" % (linear_err, angular_err))

        # decide if planning is needed
        return linear_err < self.linear_err_threshold and angular_err < self.angular_err_threshold

    def generate_planning_goal(self, joint_state, tip_pose, target_pose):
        # create the motion plan request
        plan_req = moveit_msgs.MotionPlanRequest()

        # defint the motion planning workspace
        for wp in [plan_req.workspace_parameters]:
            wp.header.stamp = rospy.Time.now()
            wp.header.frame_id = self.root_link
            # TODO: this is robot-specific
            for c in [wp.min_corner]:
                c.x = -1.5
                c.y = -1.5
                c.z = 0.0
            for c in [wp.max_corner]:
                c.x = 1.5
                c.y = 1.5
                c.z = 1.5

        # configure the planning options
        plan_req.allowed_planning_time = 0.5
        plan_req.group_name = 'arm'
        #plan_req.planner_id = 'SBLkConfigDefault'
        plan_req.planner_id = 'RRTConnectkConfigDefault'
        plan_req.start_state.is_diff = False

        # add the goal constraints
        plan_req.goal_constraints.append(moveit_msgs.Constraints())

        # configure the goal constraints
        for goal_constraints in [plan_req.goal_constraints[0]]:
            # define the name and add a position and orientation constraint
            goal_constraints.name = 'goal'
            goal_constraints.position_constraints.append(moveit_msgs.PositionConstraint())
            goal_constraints.orientation_constraints.append(moveit_msgs.OrientationConstraint())

            # create the position goal constraint
            for g in [goal_constraints.position_constraints[0]]:
                g.header.stamp = rospy.Time.now()
                g.header.frame_id = self.root_link
                g.link_name = self.tip_link
                g.weight = 1.0

                for tpo in [g.target_point_offset]:
                    tpo.x = 0.0
                    tpo.y = 0.0
                    tpo.z = 0.0

                for cr in [g.constraint_region]:
                    cr.primitives.append(shape_msgs.SolidPrimitive())
                    cr.primitives[0].type = shape_msgs.SolidPrimitive.SPHERE
                    cr.primitives[0].dimensions = [0.005]
                    # set the target position
                    cr.primitive_poses = [toMsg(fromTf(target_pose))]

            # create the orientation goal constraint
            for g in [goal_constraints.orientation_constraints[0]]:
                g.header.stamp = rospy.Time.now()
                g.header.frame_id = self.root_link
                g.link_name = self.tip_link
                g.absolute_x_axis_tolerance = 0.005
                g.absolute_y_axis_tolerance = 0.005
                g.absolute_z_axis_tolerance = 0.005
                g.weight = 1.0
                # set the target orientation
                g.orientation = toMsg(fromTf(target_pose)).orientation

        # Set the start state
        plan_req.start_state.joint_state = joint_state

        plan_opts = moveit_msgs.PlanningOptions()
        plan_opts.plan_only = True

        return moveit_msgs.MoveGroupGoal(request=plan_req, planning_options=plan_opts)

def main():
    rospy.init_node('singularity_rescuer')

    sr = SingularityRescuer()

    rospy.spin()

if __name__ == '__main__':
    main()

"""
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
    rospy.init_node('joint_limit_monitor')

    rate = rospy.Rate(5)

    urdf = rospy.get_param('robot_description')

    robot = robot.from_xml_string(urdf)

    sub = rospy.Subscriber('joint_states',sensor_msgs.JointState,state_cb, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()

"""

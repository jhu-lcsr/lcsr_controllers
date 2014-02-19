Reflexxes-Based Joint Trajectory Generator
==========================================

### Sending Commands

Commands can be sent to this generator in three ways:

1. As an `Eigen::VectorXd` of joint positions to be achieved as fast as
   possible.
2. As a ROS `trajectory_msgs::JointTrajectoryPoint` message to be achieved as
   fast as possible.
3. As a ROS `trajectory_msgs::JointTrajectory` to be achieved according to the
   segment times.
4. **TBD** Via the ROS actionlib interface.

Option 1 is exclusively for in-orocos commanding and 2 and 3 can receive
streaming trajectory commands over ROS.

Options 1 and 2 will preempt the current trajectory segment being pursued.
Option 3 will attempt to splice the current list of trajectory segments
with the new list of trajectory segments.

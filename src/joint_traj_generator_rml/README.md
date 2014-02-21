Reflexxes-Based Joint Trajectory Generator
==========================================

## Sending Commands

Commands can be sent to this generator in four ways:

1. As an `Eigen::VectorXd` of joint positions to be achieved as fast as
   possible.
2. As a ROS `trajectory_msgs::JointTrajectoryPoint` message to be achieved as
   fast as possible.
3. As a ROS `trajectory_msgs::JointTrajectory` to be achieved according to the
   segment times.
4. Via the ROS actionlib interface.

### Eigen::VectorXd Streaming (Orocos only)

* **Acquisition Time:** Immediately, subject to the dynamic limits.
* **Preemption:** Preempts current trajectory.

### JointTrajectoryPoint Message (Orocos/ROS)

* **Acquisition Time:** Immediately, subject to the dynamic limits.
* **Preemption:** Preempts current trajectory.

### JointTrajectory Message (Orocos/ROS)

When receiving a `sensor_msgs/JointTrajectory` message, this controller aims to mimic the "classic" PR2 `JointTrajectoryController` behavior. This behavior is summarized as follows. Additionally, we add the notion of "flexible" points which the controller will try to achieve as quickly as possible subject to the controller's dynamic limits.

* **Acquisition Time:** The time at which the trajectory is acquired depends both on the timestamp of the header as well as the `time_from_start` members of each `JointTrajectoryPoint`
  * **header.stamp == 0.0:** Begin pursuing the new trajectory immediately.
  * **header.stamp < NOW:** If any of the new trajectory points are to be acquired before the current trajectory, preempt the current trajectory and begin pursuing them imediately. 
  * **header.stamp > NOW:** If any of the new trajectory points are to be acquired before points in the current trajectory, insert the new points into the trajectory and begin pursuing them at the future time. All points in the current trajectory after that time will be removed.

* **Preemption:** Preempts currently-buffered trajectory segments which begin*after* 
  * **header.stamp == 0.0:** Preempt the current trajectory.
  * **header.stamp < NOW:** Preempt the current trajectory.
  * **header.stamp > NOW:** Continue the current trajectory.

For each point, if the `time_from_start` is zero, then the controller will consider it's completion time "flexible." This means that it will execute it subject to the velocity, acceleration, and jerk limits given to the controller. This is useful if your high-level trajectories should be executed as quickly as possible subject to these limits.

### JointTrajectoryAction (ROS only)

***TBD***

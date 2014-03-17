LCSR Controllers
================

This repository contains Orocos-RTT-based robot controllers meant for use with
ROS and the Conman controller manager.

## Controllers

### Effort-Outputting Controllers

* Joint-Space PID Controller
* Inverse-Dynamics Controller

### Trajectory-Generators 

* Joint Trajectory Generator
  * KDL Trapezoidal Profile `lcsr_controllers::JointTrajGeneratorKDL`
  * Reflexxes Type-2 Interpolation [`lcsr_controllers::JointTrajGeneratorRML`](src/joint_traj_generator_rml/README.md)

### Trajectory Controllers

* Semi-Absolute Calibration Controller

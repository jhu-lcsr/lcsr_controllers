#ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H
#define __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <conman/hook.h>

namespace lcsr_controllers {
  class JointTrajGeneratorKDL : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    float velocity_smoothing_factor_;
    Eigen::VectorXd 
      trap_max_vels_,
      trap_max_accs_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<Eigen::VectorXd> joint_position_cmd_eig_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_position_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_velocity_out_;

    RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> joint_position_cmd_ros_in_;

  public:
    JointTrajGeneratorKDL(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Robot model
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    std::vector<KDL::VelocityProfile_Trap> trajectories_;
    std::vector<RTT::Seconds> trajectory_start_times_;
    std::vector<RTT::Seconds> trajectory_end_times_;

    // State
    Eigen::VectorXd 
      joint_position_,
      joint_position_last_,
      joint_position_cmd_,
      joint_position_sample_,
      joint_velocity_,
      joint_velocity_raw_,
      joint_velocity_sample_;

    trajectory_msgs::JointTrajectoryPoint joint_position_cmd_ros_;

    bool has_last_position_data_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H

#ifndef __LCSR_CONTROLLERS_JT_NULLSPACE_CONTROLLER_H
#define __LCSR_CONTROLLERS_JT_NULLSPACE_CONTROLLER_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <kdl_conversions/kdl_msg.h>

#include <rtt_ros_tools/throttles.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

namespace lcsr_controllers {
  /**
   * This controller accepts a pose+twist input and outputs a desired
   * joint-space torque that drives the tip_link to the desired pose 
   * with twist while controlling a redundant "elbow" joint through nullspace
   * projection. This is a controller for redundant (DOF>6) manipulators.
   */
  class JTNullspaceController : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<KDL::FrameVel> post_twist_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;

    RTT::InputPort<geometry_msgs::PoseStamped> pose_desired_in_;
    RTT::OutputPort<geometry_msgs::PoseStamped> err_pose_debug_out_;
    RTT::OutputPort<geometry_msgs::WrenchStamped> err_wrench_debug_out_;

  public:
    JTNullspaceController(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    double
      linear_p_gain_,
      linear_d_gain_,
      linear_effort_threshold_,
      linear_position_threshold_,
      linear_position_err_norm_,
      linear_effort_norm_,
      angular_p_gain_,
      angular_d_gain_,
      angular_effort_threshold_,
      angular_position_threshold_,
      angular_position_err_norm_,
      angular_effort_norm_;

    bool 
      linear_position_within_tolerance_,
      linear_effort_within_tolerance_,
      angular_position_within_tolerance_,
      angular_effort_within_tolerance_,
      within_tolerance_;

    // Solvers
    boost::scoped_ptr<KDL::ChainFkSolverVel> fk_solver_vel_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    boost::scoped_ptr<KDL::ChainDynParam> chain_dynamics_;

    // Working variables
    KDL::JntArray positions_;
    KDL::JntArray velocities_;
    KDL::JntArrayVel posvel_;
    KDL::FrameVel framevel_;
    KDL::FrameVel framevel_err_;
    KDL::FrameVel framevel_desired_;
    KDL::Jacobian jacobian_;
    KDL::JntSpaceInertiaMatrix joint_inertia_;

    Eigen::VectorXd 
      joint_position_,
      joint_velocity_,
      wrench_,
      joint_effort_,
      joint_effort_raw_,
      joint_effort_null_;

    geometry_msgs::WrenchStamped wrench_msg_;
    geometry_msgs::PoseStamped pose_err_msg_;
    geometry_msgs::PoseStamped pose_msg_;

    rtt_ros_tools::PeriodicThrottle debug_throttle_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_JT_NULLSPACE_CONTROLLER_H

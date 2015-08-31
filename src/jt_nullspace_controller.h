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

#include <rtt_tf/tf_interface.h>
#include <rtt_ros_tools/throttles.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

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
    std::string robot_description_param_;
    std::string root_link_;
    std::string tip_link_;
    std::string target_frame_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<KDL::FrameVel> pose_twist_in_;
    RTT::InputPort<Eigen::VectorXd> joint_posture_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_velocity_des_out_;
    RTT::OutputPort<Eigen::VectorXd> wrench_out_;

    RTT::InputPort<geometry_msgs::PoseStamped> pose_desired_in_;
    RTT::OutputPort<geometry_msgs::PoseStamped> err_pose_debug_out_;
    RTT::OutputPort<geometry_msgs::WrenchStamped> err_wrench_debug_out_;
    RTT::OutputPort<geometry_msgs::WrenchStamped> p_err_wrench_debug_out_;
    RTT::OutputPort<geometry_msgs::WrenchStamped> d_err_wrench_debug_out_;
    RTT::OutputPort<geometry_msgs::WrenchStamped> twist_debug_out_;
    RTT::OutputPort<sensor_msgs::JointState> effort_debug_out_;

  public:
    JTNullspaceController(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    void printH()
    {
      if(chain_dynamics_->JntToMass(positions_, joint_inertia_) != 0) {
        RTT::log(RTT::Error) << "Could not compute joint space inertia." << RTT::endlog();
        return;
      }
      Eigen::MatrixXd H = (joint_inertia_.data); // + (eye + d_gain).inverse() * motor_inertia 
      Eigen::MatrixXd H_inv = H.inverse();

      RTT::log(RTT::Info) << "H: " << H << RTT::endlog();
      RTT::log(RTT::Info) << "Hinv: " << H_inv << RTT::endlog();
    }

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    double
      manipulability_,
      singularity_avoidance_gain_,
      joint_center_gain_,
      nullspace_damping_,
      jointspace_damping_,
      nullspace_min_singular_value_,
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
      p_wrench_,
      d_wrench_,
      wrench_,
      twist_,
      joint_effort_,
      joint_effort_raw_,
      joint_effort_null_,
      joint_velocity_des_,
      joint_d_gains_,
      joint_p_gains_,
      joint_limits_min_,
      joint_limits_max_,
      joint_limits_center_,
      joint_limits_range_;

    geometry_msgs::WrenchStamped wrench_msg_;
    geometry_msgs::PoseStamped pose_err_msg_;
    geometry_msgs::PoseStamped pose_msg_;
    sensor_msgs::JointState joint_state_msg_;

    rtt_ros_tools::PeriodicThrottle debug_throttle_;

    rtt_tf::TFInterface tf_;

    RTT::Seconds
      dur_get_data_,
      dur_compute_wrench_,
      dur_compute_jac_,
      dur_compute_eff_,
      dur_compute_nullspace_,
      dur_compute_damping_,
      dur_compute_singularity_avoidance_,
      dur_compute_nullspace_basis_,
      dur_compute_joint_inertia_,
      dur_compute_resize_nullspace_basis_;

    int projector_type_;
  private:
    // Handy typedefs
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Jd;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixJ6d;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixJJd;
    // temporaries
    //Eigen::MatrixXd M;
    Eigen::MatrixXd Z;
    Matrix6d Ji;
    MatrixJJd P1,P2,P3,P4;
    MatrixJJd N;
    MatrixJJd ZZt;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_JT_NULLSPACE_CONTROLLER_H

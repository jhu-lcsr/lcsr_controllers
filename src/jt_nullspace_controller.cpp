
// NOTE: These must be included before Eigen //
//#include <unsupported/Eigen/MatrixFunctions>
// ENDNOTE //

#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_rostopic/rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <kdl_urdf_tools/tools.h>

#include "jt_nullspace_controller.h"

using namespace lcsr_controllers;

JTNullspaceController::JTNullspaceController(std::string const& name) :
  RTT::TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,fk_solver_vel_(NULL)
  ,jac_solver_(NULL)
  ,chain_dynamics_(NULL)
  // Throttles
  ,debug_throttle_(0.05)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller. Cartesian pose commands are given in this frame.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller. Cartesian pose commands are applied to this frame.");

  this->addProperty("linear_p_gain",linear_p_gain_);
  this->addProperty("linear_d_gain",linear_d_gain_);
  this->addProperty("angular_p_gain",angular_p_gain_);
  this->addProperty("angular_d_gain",angular_d_gain_);

  cartesian_effort_limits_.resize(6);
  this->addProperty("cartesian_effort_limits",cartesian_effort_limits_);

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("post_twist_in", post_twist_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_);

  // Get an instance of the rtt_rostopic service requester
  rtt_rostopic::ROSTopic rostopic;

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("err_wrench_debug_out", err_wrench_debug_out_);
  err_wrench_debug_out_.createStream(rostopic.connection("~/"+this->getName()+"/err_wrench"));

  this->ports()->addPort("pose_desired_in", pose_desired_in_);
  pose_desired_in_.createStream(rostopic.connection("~/"+this->getName()+"/pose_desired"));
}

bool JTNullspaceController::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get absoluate parameters
  rosparam->getAbsolute("robot_description");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("linear_p_gain");
  rosparam->getComponentPrivate("linear_d_gain");
  rosparam->getComponentPrivate("angular_p_gain");
  rosparam->getComponentPrivate("angular_d_gain");

  RTT::log(RTT::Debug) << "Initializing kinematic parameters from \"" << root_link_ << "\" to \"" << tip_link_ <<"\"" << RTT::endlog();

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }
  rosparam->getComponentPrivate("cartesian_effort_limits");

  // Initialize IK solver
  fk_solver_vel_.reset(
      new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  jac_solver_.reset(
      new KDL::ChainJntToJacSolver(kdl_chain_));
  chain_dynamics_.reset(
      new KDL::ChainDynParam(
          kdl_chain_, 
          KDL::Vector(0.0,0.0,0.0)));

  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_null_.resize(n_dof_);

  // Resize working vectors
  positions_.resize(n_dof_);
  velocities_.resize(n_dof_);
  posvel_.resize(n_dof_);
  jacobian_.resize(n_dof_);
  joint_inertia_.resize(n_dof_);

  // Prepare ports for realtime processing
  joint_effort_out_.setDataSample(joint_effort_);

  return true;
}

bool JTNullspaceController::startHook()
{
  return true;
}

void JTNullspaceController::updateHook()
{
  // Read in the current joint positions & velocities
  bool new_pos_data = joint_position_in_.readNewest( joint_position_ ) == RTT::NewData;
  bool new_vel_data = joint_velocity_in_.readNewest( joint_velocity_ ) == RTT::NewData;

  if(new_pos_data && new_vel_data) {
    // Get JntArray structures from pos/vel
    positions_.data = joint_position_;
    velocities_.data = joint_velocity_;
    
    posvel_.q = positions_;
    posvel_.qdot = velocities_;

    // Read the command
    RTT::FlowStatus ros_status = pose_desired_in_.readNewest(pose_msg_);
    RTT::FlowStatus rtt_status = post_twist_in_.readNewest(framevel_desired_);
    if(ros_status == RTT::NewData) {
      KDL::Frame frame;
      tf::poseMsgToKDL(pose_msg_.pose, frame);
      framevel_desired_.M.R = frame.M;
      framevel_desired_.M.w = KDL::Vector::Zero();
      framevel_desired_.p.p = frame.p;
      framevel_desired_.p.v = KDL::Vector::Zero();
    } 
    else if(rtt_status == RTT::NoData && ros_status == RTT::NoData) 
    {
      return;
    }

    // Compute forward kinematics of current pose
    fk_solver_vel_->JntToCart(posvel_, framevel_);
    // Compute the cartesian position and velocity error
    framevel_err_ = framevel_desired_ * framevel_.Inverse();

    // Rotation error
    Eigen::Vector3d r_err = Eigen::Map<Eigen::Vector3d>(framevel_err_.M.R.GetRot().data);
    // Angular velocity error
    Eigen::Vector3d w_err = Eigen::Map<Eigen::Vector3d>(framevel_err_.M.w.data);
    // Position error
    Eigen::Vector3d p_err = Eigen::Map<Eigen::Vector3d>(framevel_err_.p.p.data);
    // Velocity error
    Eigen::Vector3d v_err = Eigen::Map<Eigen::Vector3d>(framevel_err_.p.v.data);

    // Generalized force from PD control
    Eigen::Matrix<double, 6, 1> f;
    f.segment<3>(0) = linear_p_gain_*p_err + linear_d_gain_*v_err;
    f.segment<3>(3) = angular_p_gain_*r_err + angular_d_gain_*w_err;

    // Clip force by effort limits
    f = f.cwiseMin(cartesian_effort_limits_).cwiseMax(-cartesian_effort_limits_);

    // Handy typedefs
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Jd;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixJ6d;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixJJd;

    // Compute jacobian
    if(jac_solver_->JntToJac(positions_, jacobian_) != 0) {
      RTT::log(RTT::Error) << "Could not compute manipulator jacobian." << RTT::endlog();
      this->error();
      return;
    }
    Matrix6Jd J = jacobian_.data;
    MatrixJ6d J_t = J.transpose();

    // Compute joint-space inertia matrix
    if(chain_dynamics_->JntToMass(positions_, joint_inertia_) != 0) {
      RTT::log(RTT::Error) << "Could not compute joint space inertia." << RTT::endlog();
      this->error();
      return;
    }
    Eigen::MatrixXd H = joint_inertia_.data;
    Eigen::MatrixXd H_inv = H.inverse();

    // Compute dynamically-consistent pseudoinverse
    MatrixJ6d J_pinv = H_inv * J_t * ( J * H_inv * J_t ).inverse();

    // Compute nullspace projector
    MatrixJJd eye(n_dof_,n_dof_);
    eye = Eigen::MatrixXd::Identity(n_dof_,n_dof_);
    MatrixJJd JpJ = (J_pinv * J);
    MatrixJJd N = eye - JpJ;
    MatrixJJd N_t = N.transpose();

    // Compute nullspace effort (to be projected)
    // TODO: Do something smart here
    joint_effort_null_.setZero();

    // Project the hell out of it
    joint_effort_ = J_t*f + N_t*joint_effort_null_;

    // Send joint positions
    joint_effort_out_.write( joint_effort_ );

    // Debug visualization
    if(this->debug_throttle_.ready(0.05)) {
      wrench_msg_.header.frame_id = tip_link_;
      wrench_msg_.header.stamp = rtt_rosclock::host_rt_now();
      wrench_msg_.wrench.force.x = f(0);
      wrench_msg_.wrench.force.y = f(1);
      wrench_msg_.wrench.force.z = f(2);
      wrench_msg_.wrench.torque.x = f(3);
      wrench_msg_.wrench.torque.y = f(4);
      wrench_msg_.wrench.torque.z = f(5);
      err_wrench_debug_out_.write(wrench_msg_);
    }
  }
}

void JTNullspaceController::stopHook()
{
}

void JTNullspaceController::cleanupHook()
{
}

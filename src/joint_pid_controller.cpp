
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>

#include <kdl_urdf_tools/tools.h>
#include "joint_pid_controller.h"

using namespace lcsr_controllers;

JointPIDController::JointPIDController(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("p_gains",p_gains_).doc("Proportional gains.");
  this->addProperty("i_gains",i_gains_).doc("Integral gains.");
  this->addProperty("d_gains",d_gains_).doc("Derivative gains.");
  this->addProperty("i_clamps",i_clamps_).doc("Integral clamps.");
  this->addProperty("velocity_smoothing_factor",velocity_smoothing_factor_).doc("Exponential smoothing factor to use when estimating veolocity from finite differences.");
  
  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("joint_position_cmd_in", joint_position_cmd_in_);
  this->ports()->addPort("joint_velocity_cmd_in", joint_velocity_cmd_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");

  // ROS ports

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("joint_position_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_velocity_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_position_cmd_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_velocity_cmd_in", conman::Exclusivity::EXCLUSIVE);
}

bool JointPIDController::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get absoluate parameters
  rosparam->getAbsolute("robot_description");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics with root: \"" <<root_link_<< "\" and tip: \"" <<tip_link_<< "\"" << RTT::endlog();
    return false;
  }

  // Resize IO vectors
  joint_position_cmd_.resize(n_dof_);
  joint_velocity_cmd_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_p_error_.resize(n_dof_);
  joint_i_error_.resize(n_dof_);
  joint_d_error_.resize(n_dof_);
  p_gains_.resize(n_dof_);
  i_gains_.resize(n_dof_);
  d_gains_.resize(n_dof_);
  i_clamps_.resize(n_dof_);

  p_gains_.setZero();
  i_gains_.setZero();
  d_gains_.setZero();
  i_clamps_.setZero();

  rosparam->getComponentPrivate("p_gains");
  rosparam->getComponentPrivate("i_gains");
  rosparam->getComponentPrivate("d_gains");
  rosparam->getComponentPrivate("i_clamps");
  rosparam->getComponentPrivate("velocity_smoothing_factor");

  // Prepare ports for realtime processing
  joint_effort_out_.setDataSample(joint_effort_);

  return true;
}

bool JointPIDController::startHook()
{
  // Zero the command
  joint_effort_.setZero();

  // Zero the errors
  joint_p_error_.setZero();
  joint_i_error_.setZero();
  joint_d_error_.setZero();

  // Reset the last position flag
  has_last_position_data_ = false;

  // TODO: Check sizes of all vectors

  return true;
}

void JointPIDController::updateHook()
{

  // Get the current and the time since the last update
  const RTT::Seconds 
    time = conman_hook_->getTime(), 
    period = conman_hook_->getPeriod();

  // Read in the current joint positions & velocities
  Eigen::VectorXd pos, vel;

  RTT::FlowStatus 
    pos_status = joint_position_in_.readNewest( pos ), 
    vel_status = joint_velocity_in_.readNewest( vel );

  if(pos_status == RTT::NewData && pos.size() == n_dof_ ) { joint_position_ = pos; }
  if(vel_status == RTT::NewData && vel.size() == n_dof_ ) { joint_velocity_raw_ = vel; }

  // If we don't get any position update, we don't write any new data to the ports
  if(pos_status != RTT::NewData) {
    return;
  }

  // Check the minimum requirements to compute the control command
  if(vel_status == RTT::NewData || has_last_position_data_) {
    // Trust a supplied velocity, or compute it from an exponentially-smothed finite difference
    if(vel_status == RTT::NewData) {
      // Trust the velocity input
      joint_velocity_ = joint_velocity_raw_;
    } else {
      // Estimate the joint velocity if we don't get a joint velocity estimate
      joint_velocity_ = 
        (velocity_smoothing_factor_*(joint_position_ - joint_position_last_)/period)
        + (1-velocity_smoothing_factor_)*(joint_velocity_);
    }

    // Read in the current commanded joint positions and velocities
    // These commands can be sparse and not return new information each update tick
    Eigen::VectorXd pos, vel;
    RTT::FlowStatus 
      pos_cmd_status = joint_position_cmd_in_.readNewest( pos ), 
      vel_cmd_status = joint_velocity_cmd_in_.readNewest( vel );

    if(pos_cmd_status == RTT::NewData && pos.size() == n_dof_ ) { joint_position_cmd_ = pos; }
    else if(pos_cmd_status == RTT::NoData) { joint_position_cmd_ = joint_position_; }
    if(vel_cmd_status == RTT::NewData && vel.size() == n_dof_ ) { joint_velocity_cmd_ = vel; }
    else if(vel_cmd_status == RTT::NoData) { joint_velocity_cmd_.setZero(); }

    joint_p_error_ = joint_position_cmd_ - joint_position_;
    joint_d_error_ = joint_velocity_cmd_ - joint_velocity_;
    joint_i_error_ = 
      (joint_i_error_ + period*joint_p_error_).array()
      .max(i_clamps_.array())
      .min(-i_clamps_.array());

    // Compute the command
    joint_effort_ = (
          p_gains_.array()*joint_p_error_.array()
        + i_gains_.array()*joint_i_error_.array()                  
        + d_gains_.array()*joint_d_error_.array()
        ).matrix();

    // Send joint efforts
    joint_effort_out_.write(joint_effort_);
  }

  // Save the last joint position
  joint_position_last_ = joint_position_;
  has_last_position_data_ = true;
}

void JointPIDController::stopHook()
{
}

void JointPIDController::cleanupHook()
{
}

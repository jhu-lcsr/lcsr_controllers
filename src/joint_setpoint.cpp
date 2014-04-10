
#include "joint_setpoint.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_urdf_tools/tools.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>

using namespace lcsr_controllers;

JointSetpoint::JointSetpoint(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,robot_description_param_("/robot_description")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("robot_description_param",robot_description_param_).doc("The ROS parameter name for the WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("joint_position",joint_position_cmd_).doc("The joint position setpoint.");
  this->addProperty("joint_velocity",joint_velocity_cmd_).doc("The joint velocity setpoint.");
  
  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("joint_position_out", joint_position_out_);
  this->ports()->addPort("joint_velocity_out", joint_velocity_out_);

  this->addOperation("hold",&JointSetpoint::hold, this).doc("Hold the current position.");
}

bool JointSetpoint::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("robot_description_param");
  rosparam->getParam(robot_description_param_, "robot_description");
  if(robot_description_.length() == 0) {
    RTT::log(RTT::Error) << "No robot description! Reading from parameter \"" << robot_description_param_ << "\"" << RTT::endlog();
    return false;
  }

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

  return true;
}

bool JointSetpoint::startHook()
{
  joint_position_in_.clear();
  joint_velocity_in_.clear();
  
  hold_current_ = true;

  return true;
}

void JointSetpoint::hold()
{
  hold_current_ = true;
}

void JointSetpoint::updateHook()
{
  // Read in the current joint positions & velocities
  RTT::FlowStatus 
    pos_status = joint_position_in_.readNewest( joint_position_ ), 
    vel_status = joint_velocity_in_.readNewest( joint_velocity_ );

  // If we don't get any position update, we don't write any new data to the ports
  if(pos_status != RTT::NewData || vel_status != RTT::NewData) {
    return;
  }

  if(joint_position_.size() != n_dof_ || joint_velocity_.size() != n_dof_ ) {
    this->error();
    return;
  }

  if(hold_current_) {
    joint_position_cmd_ = joint_position_;
    joint_velocity_cmd_.setZero();
    hold_current_ = false;
  }

  joint_position_out_.write(joint_position_cmd_);
  joint_velocity_out_.write(joint_velocity_cmd_);
}

void JointSetpoint::stopHook()
{
}

void JointSetpoint::cleanupHook()
{
}

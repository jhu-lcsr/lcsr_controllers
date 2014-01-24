
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>

#include <rtt_ros_tools/tools.h>
#include <kdl_urdf_tools/tools.h>
#include "id_controller_kdl.h"

using namespace lcsr_controllers;

IDControllerKDL::IDControllerKDL(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,gravity_(3,0.0)
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,id_solver_(NULL)
  ,ext_wrenches_()
  ,positions_()
  ,accelerations_()
  ,torques_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  // TODO: Get gravity from world frame
  this->addProperty("gravity",gravity_)
    .doc("The gravity vector in the root link frame.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("end_effector_cg_in", end_effector_cg_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  
  // Initialize properties from rosparam
}

bool IDControllerKDL::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get absoluate parameters
  rosparam->getAbsolute("robot_description");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("wrench_link");
  rosparam->getComponentPrivate("gravity");

  RTT::log(RTT::Debug) << "Initializing kinematic and dynamic parameters from \"" << root_link_ << "\" to \"" << tip_link_ <<"\"" << RTT::endlog();

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Create inverse dynamics chainsolver
  id_solver_.reset(
      new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));

  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_acceleration_.resize(n_dof_);
  joint_effort_.resize(n_dof_);

  // Resize working vectors
  positions_.resize(n_dof_);
  velocities_.resize(n_dof_);
  accelerations_.resize(n_dof_);
  ext_wrenches_.resize(kdl_chain_.getNrOfSegments());
  torques_.resize(n_dof_);

  // Prepare ports for realtime processing
  joint_effort_out_.setDataSample(joint_effort_);

  return true;
}

bool IDControllerKDL::startHook()
{
  // Zero accelerations
  joint_acceleration_.setZero();
  // Zero out torque data
  torques_.data.setZero();
  return true;
}

void IDControllerKDL::updateHook()
{
  // Read in the current joint positions & velocities
  bool new_pos_data = joint_position_in_.readNewest( joint_position_ ) == RTT::NewData;
  bool new_vel_data = joint_velocity_in_.readNewest( joint_velocity_ ) == RTT::NewData;
  bool new_ee_data = end_effector_cg_in_.readNewest( end_effector_cg_ ) == RTT::NewData;

  if(new_ee_data) {
    // Compute the external wrench on the tip link assuming a 
    // Set the wrench (in root_link_ coordinates)
    ext_wrenches_.back();
  } else { 
    // Zero the last wrench
    ext_wrenches_.back() = KDL::Wrench::Zero();
  }

  if(new_pos_data && new_vel_data) {
    positions_.data = joint_position_;
    velocities_.data = joint_velocity_;

    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
          positions_,
          velocities_,
          accelerations_,
          ext_wrenches_,
          torques_) != 0)
    {
      RTT::log(RTT::Error) << "Could not compute joint torques!" << RTT::endlog();
    }

    // Store the effort command
    joint_effort_ = torques_.data;

    // Send joint positions
    joint_effort_out_.write( joint_effort_ );
  }
}

void IDControllerKDL::stopHook()
{
}

void IDControllerKDL::cleanupHook()
{
}

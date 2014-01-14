
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

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
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  
  // Initialize properties from rosparam
  rtt_ros_tools::load_rosparam_and_refresh(this);
}

bool IDControllerKDL::configureHook()
{
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    ROS_ERROR("Could not initialize robot kinematics!");
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
  joint_position_in_.readNewest( joint_position_ );
  joint_velocity_in_.readNewest( joint_velocity_ );

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
    ROS_ERROR("Could not compute joint torques!");
  }

  // Store the effort command
  joint_effort_ = torques_.data;

  // Send joint positions
  joint_effort_out_.write( joint_effort_ );
}

void IDControllerKDL::stopHook()
{
}

void IDControllerKDL::cleanupHook()
{
}

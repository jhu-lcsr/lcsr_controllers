
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <ocl/Component.hpp>

#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <kdl_urdf_tools/tools.h>
#include "coulomb_compensator.h"

using namespace lcsr_controllers;

// Simple exponential sigmoid function for approximating a step function
// x: value
// s_neg: scale for x<0
// s_pos: scale for x>0
// k: slope at 0
static inline double sigm(
    const double x,
    const double s_neg, 
    const double s_pos, 
    const double k)
{
  const double s = (x>0) ? s_pos : s_neg;
  const double v = s*(2.0/(1.0 + exp(-2.0*k*x/s)) - 1.0);
  return v;
}

CoulombCompensator::CoulombCompensator(std::string const& name) :
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
  ,ros_publish_throttle_(0.02)
  ,zero_slope_(1.0)
  ,warn_flag_(false)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The URDF xml string.");
  this->addProperty("robot_description_param",robot_description_param_)
    .doc("The ROS parameter name for the URDF xml string.");

  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("friction_coefficients_neg",friction_coefficients_neg_)
    .doc("Friction coefficients for velocities < 0");
  this->addProperty("friction_coefficients_pos",friction_coefficients_pos_)
    .doc("Friction coefficients for velocities > 0.");
  this->addProperty("zero_slope",zero_slope_)
    .doc("The slope of the step function approximation at zero velocity.");

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_)
    .doc("Input port: nx1 vector of joint position. (n joints)");
  this->ports()->addPort("framevel_des_in", framevel_des_in_)
    .doc("Input port: Desired frame and twist.");
  this->ports()->addPort("wrench_des_in", wrench_des_in_)
    .doc("Input port: Desired wrench (mutually exclusive with framevel_des_in.");
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of desired joint efforts. (n joints)");

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("joint_state_des_out", joint_state_des_out_);
  joint_state_des_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/joint_state"));
}

bool CoulombCompensator::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("zero_slope");

  rosparam->getComponentPrivate("robot_description_param");
  rosparam->getParam(robot_description_param_, "robot_description");
  if(robot_description_.length() == 0) {
    RTT::log(RTT::Error) << "No robot description! Reading from parameter \"" << robot_description_param_ << "\"" << RTT::endlog();
    return false;
  }

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  RTT::log(RTT::Debug) << "Initializing kinematic and dynamic parameters from \"" << root_link_ << "\" to \"" << tip_link_ <<"\"" << RTT::endlog();
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Get joint names
  joint_state_des_.name.clear();
  joint_state_des_.name.reserve(n_dof_);
  for(std::vector<KDL::Segment>::iterator it = kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      ++it)
  {
    if(it->getJoint().getType() != KDL::Joint::None) {
      joint_state_des_.name.push_back(it->getJoint().getName());
    }
  }

  // Resize working variables
  joint_position_.resize(n_dof_);
  joint_limits_min_.resize(n_dof_);
  joint_limits_max_.resize(n_dof_);
  joint_velocity_des_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  friction_coefficients_neg_.resize(n_dof_);
  friction_coefficients_pos_.resize(n_dof_);

  rosparam->getComponentPrivate("friction_coefficients_neg");
  rosparam->getComponentPrivate("friction_coefficients_pos");

  if(friction_coefficients_neg_.size() != n_dof_ || friction_coefficients_pos_.size() != n_dof_) {
    RTT::log(RTT::Error) << "Wrong number of friction coefficients!" << RTT::endlog();
    return false;
  }

  // Get joint limits from URDF model
  {
    unsigned int i=0;
    for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        it++)
    {
      if(it->getJoint().getType() != KDL::Joint::None) {
        joint_limits_min_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->lower;
        joint_limits_max_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->upper;
        i++;
      }
    }
  }

  // Initialize velocity IK solver
#if 1
  kdl_ik_solver_vel_.reset(
      new KDL::ChainIkSolverVel_pinv_givens(kdl_chain_));
#else 
  kdl_ik_solver_vel_.reset(
      new KDL::ChainIkSolverVel_pinv_givens(
        kdl_chain_,
        1.0E-6,
        150));
#endif

  joint_state_des_.position.resize(n_dof_);
  joint_state_des_.velocity.resize(n_dof_);
  joint_state_des_.effort.resize(n_dof_);

  wrench_des_.resize(6);

  return true;
}

bool CoulombCompensator::startHook()
{
  // Zero out data
  KDL::SetToZero(joint_position_);
  return true;
}

void CoulombCompensator::updateHook()
{
  // Read in the current joint position & velocities
  bool new_pos_data = joint_position_in_.readNewest( joint_position_.data ) == RTT::NewData;
  bool new_framevel_des_data = framevel_des_in_.readNewest( framevel_des_ ) == RTT::NewData;
  bool new_wrench_des_data = wrench_des_in_.readNewest( wrench_des_ ) == RTT::NewData;

  if(!(new_pos_data && (new_framevel_des_data || new_wrench_des_data))) {
    return;
  }

  if(new_framevel_des_data) {
    // Construct twist from framevel
    tip_twist_des_.vel = framevel_des_.p.v;
    tip_twist_des_.rot = framevel_des_.M.w;
  } else if(new_wrench_des_data && wrench_des_.size() == 6) {
    for(unsigned i=0; i<6; i++) {
      tip_twist_des_[i] = wrench_des_(i);
    }
  } else {
    return;
  }

  // Get the des joint velocity
  bool ik_ret = kdl_ik_solver_vel_->CartToJnt(
      joint_position_, 
      tip_twist_des_, 
      joint_velocity_des_);

  if(ik_ret < 0) {
    return;
  }

  // Compute the colomb force for each joint
  for(unsigned i=0; i<n_dof_; i++) {
    joint_effort_(i) = sigm(
        joint_velocity_des_.data(i), 
        friction_coefficients_neg_(i),
        friction_coefficients_pos_(i),
        zero_slope_);
  }

  // Send friction compenstation effort
  joint_effort_out_.write( joint_effort_ );

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
    // Publish controller des state
    joint_state_des_.header.stamp = rtt_rosclock::host_now();
    std::copy(
        joint_position_.data.data(), 
        joint_position_.data.data() + n_dof_, 
        joint_state_des_.position.begin());
    std::copy(
        joint_velocity_des_.data.data(), 
        joint_velocity_des_.data.data() + n_dof_, 
        joint_state_des_.velocity.begin());
    std::copy(
        joint_effort_.data(), 
        joint_effort_.data() + n_dof_, 
        joint_state_des_.effort.begin());
    joint_state_des_out_.write(joint_state_des_);
  }
}

void CoulombCompensator::stopHook()
{
  joint_position_in_.clear();
  framevel_des_in_.clear();
}

void CoulombCompensator::cleanupHook()
{
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::CoulombCompensator)

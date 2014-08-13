#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <ocl/Component.hpp>

#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <tf_conversions/tf_kdl.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <kdl_urdf_tools/tools.h>
#include "cartesian_logistic_servo.h"

using namespace lcsr_controllers;

CartesianLogisticServo::CartesianLogisticServo(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,robot_description_param_("/robot_description")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  // Working variables
  ,n_dof_(0)
  ,ros_publish_throttle_(0.02)
  ,warn_flag_(false)
  ,max_linear_rate_(0.0)
  ,max_angular_rate_(0.0)
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
  this->addProperty("target_frame",target_frame_)
    .doc("The target frame to track with tip_link.");
  this->addProperty("max_linear_rate",max_linear_rate_);
  this->addProperty("max_angular_rate",max_angular_rate_);

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");

  this->ports()->addPort("framevel_out", framevel_out_port_)
    .doc("Output port: KDL::FrameVel of frame that moves subject to rate limits");
}

bool CartesianLogisticServo::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("target_frame");
  rosparam->getComponentPrivate("max_linear_rate");
  rosparam->getComponentPrivate("max_angular_rate");

  rosparam->getComponentPrivate("robot_description_param");
  rosparam->getParam(robot_description_param_, "robot_description");
  if(robot_description_.length() == 0) {
    RTT::log(RTT::Error) << "No robot description! Reading from parameter \"" << robot_description_param_ << "\"" << RTT::endlog();
    return false;
  }

  if(this->hasPeer("tf")) {
    TaskContext* tf_task = this->getPeer("tf");
    tf_lookup_transform_ = tf_task->getOperation("lookupTransform"); // void reset(void)
    tf_broadcast_transform_ = tf_task->getOperation("broadcastTransform"); // void reset(void)

    if(!tf_lookup_transform_.ready()) {
      RTT::log(RTT::Error) << "Could not get operation `lookupTransform`" << RTT::endlog();
      return false;
    }
    if(!tf_broadcast_transform_.ready()) {
      RTT::log(RTT::Error) << "Could not get operation `broadcastTransform`" << RTT::endlog();
      return false;
    }
  } else {
    ROS_ERROR("CartesianLogisticServo controller is not connected to tf!");
    return false;
  }

  // ROS topics
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

  // Initialize FK solver
  fk_solver_vel_.reset(
      new KDL::ChainFkSolverVel_recursive(kdl_chain_));

  // Initialize jacobian solver
  jac_solver_.reset(
      new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize working variables
  positions_.resize(n_dof_);
  joint_limits_min_.resize(n_dof_);
  joint_limits_max_.resize(n_dof_);

  // Get joint limits from URDF model
  // TODO: use these to schedule different rates (eg slower rate when near a
  // joint limit)
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

  return true;
}

bool CartesianLogisticServo::startHook()
{
  try{
    tip_frame_msg_ = tf_lookup_transform_(root_link_,target_frame_);
  } catch (std::exception &ex) {
    RTT::log(RTT::Warning)<<"Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what()<<RTT::endlog();
  }

  // Zero velocity estimate
  positions_.qdot.data.setZero();

  // Initialize TF message
  target_frame_limited_msg_.header.frame_id = root_link_;
  target_frame_limited_msg_.child_frame_id = target_frame_+"_limited";

  // TODO: get last update time from Conman
  last_update_time_ = rtt_rosclock::rtt_now();
  return true;
}

static inline double sigm(const double x, const double s)
{
  return s*(2.0/(1.0 + exp(-2.0*x/s)) - 1.0);
}

void CartesianLogisticServo::updateHook()
{
  // Compute the inverse kinematics solution
  update_time_ = rtt_rosclock::rtt_now();
  ros::Duration period = update_time_ - last_update_time_;
  last_update_time_ = update_time_;

  // Read in the current joint positions
  RTT::FlowStatus positions_data = positions_in_port_.readNewest( positions_.q.data );
  if(positions_data != RTT::NewData) {
    return;
  }

  // Compute the current target frame
  fk_solver_vel_->JntToCart(positions_, framevel_limited_);
  frame_limited_ = framevel_limited_.GetFrame();

  // Get transform from the root link frame to the target frame
  try{
    tip_frame_msg_ = tf_lookup_transform_("/"+root_link_,target_frame_);
    warn_flag_ = false;
  } catch (std::exception &ex) {
    if(!warn_flag_) {
      RTT::log(RTT::Warning) << "Could not look up transform from \""<<root_link_<<
        "\" to \""<<target_frame_<<"\": "<<ex.what() << RTT::endlog();
      warn_flag_ = true;
    }
    return;
  }

  tf::transformMsgToKDL(tip_frame_msg_.transform, tip_frame_des_);

  // Get the twist required to achieve the goal in one control cycle
  tip_frame_twist_ = KDL::diff(frame_limited_, tip_frame_des_, 0.001);

  // TODO: Add option to apply joint velocity limits to cartesian velocities
  // based on the jacobian

  // Compute the scaled twists
  tip_frame_twist_.vel =
    tip_frame_twist_.vel 
    / tip_frame_twist_.vel.Norm()
    * sigm(tip_frame_twist_.vel.Norm(), max_linear_rate_);

  tip_frame_twist_.rot =
    tip_frame_twist_.rot 
    / tip_frame_twist_.rot.Norm()
    * sigm(tip_frame_twist_.rot.Norm(), max_angular_rate_);

  // Reintegrate the twist
  if(period.toSec() > 1E-8) {
    frame_limited_.Integrate(frame_limited_.M.Inverse()*tip_frame_twist_, 1.0/period.toSec());
    framevel_limited_ = frame_limited_;
  } else {
    RTT::log(RTT::Warning) << "CartesianLogisticServo: Period went backwards or is exceptionally small. Not changing output pose." << RTT::endlog();
  }

  // Send position target
  framevel_out_port_.write( framevel_limited_ );

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
    tf::transformKDLToMsg(frame_limited_, target_frame_limited_msg_.transform);
    target_frame_limited_msg_.header.stamp = rtt_rosclock::host_now();
    tf_broadcast_transform_(target_frame_limited_msg_);
  }
}

void CartesianLogisticServo::stopHook()
{
  positions_in_port_.clear();
}

void CartesianLogisticServo::cleanupHook()
{
}

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::CartesianLogisticServo)

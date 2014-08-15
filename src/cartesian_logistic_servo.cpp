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
  this->addProperty("linear_position_threshold",linear_position_threshold_);
  this->addProperty("linear_p_gain",linear_p_gain_);
  this->addProperty("angular_position_threshold",angular_position_threshold_);
  this->addProperty("angular_p_gain",angular_p_gain_);

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
  rosparam->getComponentPrivate("linear_position_threshold");
  rosparam->getComponentPrivate("angular_position_threshold");
  rosparam->getComponentPrivate("linear_p_gain");
  rosparam->getComponentPrivate("angular_p_gain");

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

  // Compute the current tip frame
  fk_solver_vel_->JntToCart(positions_, tip_framevel_cur_);

  // Get transform from the root link frame to the target frame
  try{
    tip_frame_msg_ = tf_lookup_transform_("/"+root_link_,target_frame_);
    tf::transformMsgToKDL(tip_frame_msg_.transform, tip_frame_des_);
    warn_flag_ = false;
  } catch (std::exception &ex) {
    if(!warn_flag_) {
      RTT::log(RTT::Warning) << "Could not look up transform from \""<<root_link_<<
        "\" to \""<<target_frame_<<"\": "<<ex.what() << RTT::endlog();
      warn_flag_ = true;
    }
    return;
  }

  // Compute the cartesian position and velocity error
  KDL::Twist frame_err = KDL::diff(
      tip_framevel_cur_.GetFrame(), 
      tip_framevel_des_.GetFrame());

  KDL::Twist twist_err = KDL::diff(
      tip_framevel_cur_.GetTwist(), 
      tip_framevel_des_.GetTwist());

  // Rotation error
  Eigen::Vector3d r_err = Eigen::Map<Eigen::Vector3d>(frame_err.rot.data);
  // Angular velocity error
  Eigen::Vector3d w_err = Eigen::Map<Eigen::Vector3d>(twist_err.rot.data);
  // Position error
  Eigen::Vector3d p_err = Eigen::Map<Eigen::Vector3d>(frame_err.vel.data);
  // Velocity error
  Eigen::Vector3d v_err = Eigen::Map<Eigen::Vector3d>(twist_err.vel.data);

  // Check pos/vel tolerances
  bool linear_position_within_tolerance = p_err.norm() < linear_position_threshold_;
  bool angular_position_within_tolerance = r_err.norm() < angular_position_threshold_;

  // Re-set the integrated frame if it gets too far away
  if(!linear_position_within_tolerance || !angular_position_within_tolerance) {
    tip_framevel_cmd_ = tip_framevel_cur_;
  }

  // Get the twist error between the command and the integrated frame
  tip_frame_error_ = KDL::diff(tip_framevel_cmd_.GetFrame(), tip_frame_des_);

  // TODO: Add option to apply joint velocity limits to cartesian velocities
  // based on the jacobian

  // Compute the scaled twists
  tip_frame_twist_.vel = linear_p_gain_ * tip_frame_error_.vel;
  tip_frame_twist_.vel = tip_frame_twist_.vel / tip_frame_twist_.vel.Norm() * sigm(tip_frame_twist_.vel.Norm(), max_linear_rate_);

  tip_frame_twist_.rot = angular_p_gain_ * tip_frame_error_.rot;
  tip_frame_twist_.rot = tip_frame_twist_.rot / tip_frame_twist_.rot.Norm() * sigm(tip_frame_twist_.rot.Norm(), max_angular_rate_);

  // Reintegrate the twist
  if(period.toSec() > 1E-8) {
    tip_frame_cmd_ = tip_framevel_cmd_.GetFrame();
    tip_frame_cmd_.Integrate(tip_framevel_cmd_.GetFrame().M.Inverse()*tip_frame_twist_, 1.0/period.toSec());
    // Set command framevel with zero feed-forward velocity
    tip_framevel_cmd_ = tip_frame_cmd_;
  } else {
    RTT::log(RTT::Warning) << "CartesianLogisticServo: Period went backwards or is exceptionally small. Not changing output pose." << RTT::endlog();
  }

  // Send position target
  framevel_out_port_.write( tip_framevel_cmd_ );

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
    tf::transformKDLToMsg(tip_frame_cmd_, target_frame_limited_msg_.transform);
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

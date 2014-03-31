
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <tf_conversions/tf_kdl.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <kdl_urdf_tools/tools.h>
#include "ik_controller.h"

using namespace lcsr_controllers;

IKController::IKController(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,positions_()
  ,torques_()
  ,kp_(7,0.0)
  ,kd_(7,0.0)
  ,ros_publish_throttle_(0.02)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  this->addProperty("kd",kd_);
  this->addProperty("kp",kp_);

  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("target_frame",target_frame_)
    .doc("The target frame to track with tip_link.");
  //this->addProperty("compensate_end_effector",compensate_end_effector_)
  //  .doc("Will compute a wrench on the tip link if true.");

  // Configure data ports
  this->ports()->addPort("positions_in", positions_in_port_)
    .doc("Input port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("positions_out", positions_out_port_)
    .doc("Output port: nx1 vector of desired joint positins. (n joints)");
  this->ports()->addPort("torques_out", torques_out_port_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");
  this->ports()->addPort("trajectories_out", trajectories_out_port_)
    .doc("Output port: nx1 vector of joint trajectories. (n joints)");

  this->addOperation("testIK", &IKController::test_ik, this, RTT::OwnThread)
    .doc("Test the IK computation.");

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("trajectories_debug_out", trajectories_debug_out_);
  trajectories_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/trajectories"));

  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);
}

bool IKController::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get absoluate parameters
  rosparam->getAbsolute("robot_description");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("target_frame");
  rosparam->getComponentPrivate("kp");
  rosparam->getComponentPrivate("kd");

  if(this->hasPeer("tf")) {
    TaskContext* tf_task = this->getPeer("tf");
    tf_lookup_transform_ = tf_task->getOperation("lookupTransform"); // void reset(void)
  } else {
    ROS_ERROR("IKController controller is not connected to tf!");
    return false;
  }

  // ROS topics
  if(!joint_state_desired_out_.createStream(rtt_roscomm::topic("~" + this->getName() + "/joint_state_desired")))
  {
    RTT::log(RTT::Error) << "ROS Topics could not be streamed..." <<RTT::endlog();
    return false;
  }

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

  if(kp_.size() < n_dof_ || kd_.size() < n_dof_) {
    RTT::log(RTT::Error) << "Not enough gains!" << RTT::endlog();
  }

  // Get joint names
  joint_state_desired_.name.clear();
  joint_state_desired_.name.reserve(n_dof_);
  for(std::vector<KDL::Segment>::iterator it = kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      ++it)
  {
    joint_state_desired_.name.push_back(it->getJoint().getName());
  }

  // Resize working variables
  positions_.resize(n_dof_);
  positions_des_.resize(n_dof_);
  joint_limits_min_.resize(n_dof_);
  joint_limits_max_.resize(n_dof_);
  torques_.resize(n_dof_);

  // Get joint limits from URDF model
  {
    unsigned int i=0;
    for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        it++)
    {
      joint_limits_min_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->lower;
      joint_limits_max_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->upper;
      i++;
    }
  }

  // Construct trajectory message
  trajectory_.joint_names.clear();
  trajectory_.points.clear();

  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      it++)
  {
    trajectory_.joint_names.push_back(it->getJoint().getName());
  }

  trajectory_msgs::JointTrajectoryPoint single_point;
  single_point.time_from_start = ros::Duration(0.0);
  single_point.positions.resize(n_dof_);
  single_point.velocities.resize(n_dof_);
  std::fill(single_point.positions.begin(),single_point.positions.end(),0.0);
  std::fill(single_point.velocities.begin(),single_point.velocities.end(),0.0);
  trajectory_.points.push_back(single_point);


  // Initialize IK solver
  kdl_fk_solver_pos_.reset(
      new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  kdl_ik_solver_vel_.reset(
      new KDL::ChainIkSolverVel_wdls(
        kdl_chain_,
        1.0E-6,
        150));
#ifdef USE_NR_JL
  kdl_ik_solver_top_.reset(
      new KDL::ChainIkSolverPos_NR_JL(
        kdl_chain_,
        joint_limits_min_,
        joint_limits_max_,
        *kdl_fk_solver_pos_,
        *kdl_ik_solver_vel_,
        250,
        1.0E-6));
#endif

#if 1
  kdl_ik_solver_top_.reset(
      new KDL::ChainIkSolverPos_LMA(
        kdl_chain_,
        1E-5,
        500,
        1E-15));
#endif

  // Zero out torque data
  torques_.data.setZero();
  positions_.q.data.setZero();
  positions_.qdot.data.setZero();
  positions_des_.q.data.setZero();
  positions_des_.qdot.data.setZero();

  // Prepare ports for realtime processing
  positions_out_port_.setDataSample(positions_des_.q.data);
  torques_out_port_.setDataSample(torques_.data);
  trajectories_out_port_.setDataSample(trajectory_);

  return true;
}

void IKController::test_ik() { this->compute_ik(true); }

void IKController::compute_ik(bool debug)
{
  // Read in the current joint positions
  positions_in_port_.readNewest( positions_.q.data );

  // Get transform from the root link frame to the target frame
  try{
    tip_frame_msg_ = tf_lookup_transform_("/"+root_link_,target_frame_);
  } catch (std::exception &ex) {
    RTT::log(RTT::Error) << "Could not look up transform from \""<<root_link_<<
      "\" to \""<<target_frame_<<"\": "<<ex.what() << RTT::endlog();
    this->stop();
  }
  tf::transformMsgToTF(tip_frame_msg_.transform, tip_frame_tf_);
  tf::TransformTFToKDL(tip_frame_tf_,tip_frame_des_);

  // Compute joint coordinates of the target tip frame
  KDL::JntArray ik_hint(n_dof_);
  if(0) {
    // TODO: parameterize this
    ik_hint.data = positions_.q.data;
  } else {
    ik_hint.data = (joint_limits_min_.data + joint_limits_max_.data)/2.0;
  }

  kdl_ik_solver_top_->CartToJnt(ik_hint, tip_frame_des_, positions_des_.q);

  if(debug) {
    RTT::log(RTT::Debug)<<"Unwrapped angles: "<<(positions_des_.q.data.transpose)()<<RTT::endlog();
  }

  // Unwrap angles
  for(unsigned int i=0; i<n_dof_; i++) {
    if(positions_des_.q(i) > 0) {
      positions_des_.q(i) = fmod(positions_des_.q(i)+M_PI,2.0*M_PI)-M_PI;
    } else {
      positions_des_.q(i) = fmod(positions_des_.q(i)-M_PI,2.0*M_PI)+M_PI;
    }
  }

  if(debug) {
    RTT::log(RTT::Debug)<<"Wrapped angles: "<<(positions_des_.q.data.transpose())<<RTT::endlog();
  }

  // Servo in jointspace to the appropriate joint coordinates
  for(unsigned int i=0; i<n_dof_; i++) {
    torques_(i) =
      kp_[i]*(positions_des_.q(i) - positions_.q(i))
      + kd_[i]*(positions_des_.qdot(i) - positions_.qdot(i));
  }

  if(debug) {
    ROS_INFO_STREAM("Current position: "<<positions_.q.data.transpose());
    ROS_INFO_STREAM("Current velocity: "<<positions_.qdot.data.transpose());
    ROS_INFO_STREAM("IK result: "<<positions_des_.q.data.transpose());
    ROS_INFO_STREAM("Displacement: "<<(positions_des_.q.data-positions_.q.data).transpose());
    ROS_INFO_STREAM("Torques: "<<torques_.data.transpose());
  }
}

bool IKController::startHook()
{
  try{
    tip_frame_msg_ = tf_lookup_transform_(root_link_,target_frame_);
  } catch (std::exception &ex) {
    RTT::log(RTT::Error)<<"Could not look up transform from \""<<root_link_<<"\" to \""<<target_frame_<<"\": "<<ex.what()<<RTT::endlog();
    return false;
  }
  return true;
}

void IKController::updateHook()
{
  // Compute the inverse kinematics solution
  this->compute_ik(false);

  // Send position target
  positions_out_port_.write( positions_des_.q.data );

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
  // Send traj target
  if(trajectories_out_port_.connected()) {
    trajectory_.header.stamp = ros::Time(0,0);

    for(size_t i=0; i<n_dof_; i++) {
      trajectory_.points[0].positions[i] = positions_des_.q(i);
      trajectory_.points[0].velocities[i] = positions_des_.qdot(i);
    }

    trajectories_out_port_.write( trajectory_ );
  }


    // Publish controller desired state
    joint_state_desired_.header.stamp = rtt_rosclock::host_rt_now();
    joint_state_desired_.position.resize(n_dof_);
    joint_state_desired_.velocity.resize(n_dof_);
    std::copy(positions_des_.q.data.data(), positions_des_.q.data.data() + n_dof_, joint_state_desired_.position.begin());
    std::copy(positions_des_.qdot.data.data(), positions_des_.qdot.data.data() + n_dof_, joint_state_desired_.velocity.begin());
    joint_state_desired_out_.write(joint_state_desired_);
  }
}

void IKController::stopHook()
{
  positions_in_port_.clear();
}

void IKController::cleanupHook()
{
}



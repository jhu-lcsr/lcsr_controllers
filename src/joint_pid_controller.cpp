#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl_urdf_tools/tools.h>

#include <rtt_rosparam/rosparam.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>

#include "joint_pid_controller.h"
#include "friction/joint_friction_compensator_hss.h"

using namespace lcsr_controllers;

JointPIDController::JointPIDController(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,robot_description_param_("/robot_description")
  ,root_link_("")
  ,tip_link_("")
  ,tolerance_violations_(0)
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,ros_publish_throttle_(0.02)
  ,compensate_friction_(false)
  ,static_eps_(0.0)
  ,verbose_(false)
  ,chain_dynamics_(NULL)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("robot_description_param",robot_description_param_).doc("The ROS parameter name for the WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("p_gains",p_gains_).doc("Proportional gains.");
  this->addProperty("i_gains",i_gains_).doc("Integral gains.");
  this->addProperty("d_gains",d_gains_).doc("Derivative gains.");
  this->addProperty("i_clamps",i_clamps_).doc("Integral clamps.");
  this->addProperty("position_tolerance",position_tolerance_).doc("Maximum position error.");
  this->addProperty("velocity_tolerance",velocity_tolerance_).doc("Maximum velocity error.");
  this->addProperty("tolerance_violations",tolerance_violations_).doc("Number of position or velocity tolerance violations.");
  this->addProperty("compensate_friction",compensate_friction_).doc("Compensate for static friction if true.");
  this->addProperty("static_effort",static_effort_).doc("Static friction effort.");
  this->addProperty("static_deadband",static_deadband_).doc("Static friction deadband.");
  this->addProperty("static_eps",static_eps_).doc("Static friction velocity deadband.");
  this->addProperty("verbose",verbose_).doc("Verbose output.");
  
  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("joint_position_cmd_in", joint_position_cmd_in_);
  this->ports()->addPort("joint_velocity_cmd_in", joint_velocity_cmd_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");

  // ROS ports
  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);

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

  chain_dynamics_.reset(
      new KDL::ChainDynParam(
          kdl_chain_, 
          KDL::Vector(0.0,0.0,0.0)));
  
  // Get joint names
  joint_state_desired_.name.clear();
  joint_state_desired_.name.reserve(n_dof_);
  for(std::vector<KDL::Segment>::iterator it = kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      ++it)
  {
    joint_state_desired_.name.push_back(it->getJoint().getName());
  }

  // ROS topics
  if(!joint_state_desired_out_.createStream(rtt_roscomm::topic("~" + this->getName() + "/joint_state_desired")))
  {
    RTT::log(RTT::Error) << "ROS Topics could not be streamed..." <<RTT::endlog();
    return false;
  }

  position_tolerance_.conservativeResize(n_dof_);
  velocity_tolerance_.conservativeResize(n_dof_);

  // Resize IO vectors
  joint_inertia_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  joint_velocity_cmd_.resize(n_dof_);
  joint_acceleration_cmd_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_p_error_.resize(n_dof_);
  joint_p_error_last_.resize(n_dof_);
  joint_i_error_.resize(n_dof_);
  joint_d_error_.resize(n_dof_);
  p_gains_.resize(n_dof_);
  i_gains_.resize(n_dof_);
  d_gains_.resize(n_dof_);
  i_clamps_.resize(n_dof_);

  static_effort_.resize(n_dof_);
  static_effort_.setZero();
  static_deadband_.resize(n_dof_);
  static_deadband_.setZero();

  p_gains_.setZero();
  i_gains_.setZero();
  d_gains_.setZero();
  i_clamps_.setZero();

  rosparam->getComponentPrivate("p_gains");
  rosparam->getComponentPrivate("i_gains");
  rosparam->getComponentPrivate("d_gains");
  rosparam->getComponentPrivate("i_clamps");
  rosparam->getComponentPrivate("position_tolerance");
  rosparam->getComponentPrivate("velocity_tolerance");
  rosparam->getComponentPrivate("compensate_friction");
  rosparam->getComponentPrivate("static_effort");
  rosparam->getComponentPrivate("static_deadband");
  rosparam->getComponentPrivate("static_eps");
  rosparam->getComponentPrivate("verbose");

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
  joint_p_error_last_.setZero();
  joint_i_error_.setZero();
  joint_d_error_.setZero();

  joint_position_in_.clear();
  joint_velocity_in_.clear();
  joint_position_cmd_in_.clear();
  joint_velocity_cmd_in_.clear();
  // TODO: Check sizes of all vectors

  return true;
}

void JointPIDController::updateHook()
{
  static ros::Time last_time = rtt_rosclock::rtt_now();
  const ros::Time time = rtt_rosclock::rtt_now();
  const RTT::Seconds period = (time - last_time).toSec();
  last_time = time;
  // Get the current and the time since the last update
  /*
   *const RTT::Seconds 
   *  time = conman_hook_->getTime(), 
   *  period = conman_hook_->getPeriod();
   */

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

  // Read in the current commanded joint positions and velocities
  // These commands can be sparse and not return new information each update tick
  RTT::FlowStatus 
    pos_cmd_status = joint_position_cmd_in_.readNewest( joint_position_cmd_ ), 
    vel_cmd_status = joint_velocity_cmd_in_.readNewest( joint_velocity_cmd_ ),
    accel_cmd_status = joint_acceleration_cmd_in_.readNewest( joint_acceleration_cmd_ );

  if(pos_cmd_status != RTT::NewData || vel_cmd_status != RTT::NewData) {
    return;
  }

  if(accel_cmd_status != RTT::NewData) {
    joint_acceleration_cmd_ = Eigen::VectorXd::Zero(n_dof_);
  }

  if(joint_position_cmd_.size() != n_dof_ || joint_velocity_cmd_.size() != n_dof_ || joint_acceleration_cmd_.size() != n_dof_) {
    this->error();
    return;
  }

  joint_p_error_ = joint_position_cmd_ - joint_position_;
  if(1) {
    joint_d_error_ = joint_velocity_cmd_ - joint_velocity_;
  } else {
    joint_d_error_ = (joint_p_error_ - joint_p_error_last_) / period;
    joint_p_error_last_ = joint_p_error_;
  }
  joint_i_error_ = 
    ((joint_i_error_ + period*joint_p_error_).array()
    .max(-i_clamps_.array()))
    .min(i_clamps_.array());

  // Determine if any of the joint tolerances have been violated (this means we need to recompute the traj)
  int current_tolerance_violations = 0;
  for(int i=0; i<n_dof_; i++) 
  {
    if(fabs(joint_p_error_(i)) > position_tolerance_(i)) {
      if(verbose_) RTT::log(RTT::Warning) << "["<<this->getName() <<"] Joint " << i << " position error tolerance violated ("<<fabs(joint_p_error_[i])<<" > "<<position_tolerance_[i]<<")" << RTT::endlog(); 
      current_tolerance_violations++;
    }
    if(fabs(joint_d_error_(i)) > velocity_tolerance_(i))  {
      if(verbose_) RTT::log(RTT::Warning) << "["<<this->getName() <<"] Joint " << i << " velocity error tolerance violated ("<<fabs(joint_d_error_[i])<<" > "<<velocity_tolerance_[i]<<")" << RTT::endlog(); 
      current_tolerance_violations++;
    }
  }

  if(current_tolerance_violations > 0) {
    if(tolerance_violations_ == 0) {
      RTT::log(RTT::Warning) << "PID tolerances violated by current command. (This warning will only be printed once until the command returns to within tolerances.)" << RTT::endlog();
    }
    tolerance_violations_ += current_tolerance_violations;
    joint_effort_.setZero();
  } else {

    // Compute the command
    if(compensate_friction_) {
      for(unsigned i=0; i<n_dof_; i++) {
        joint_effort_(i) =
          JointFrictionCompensatorHSS::Compensate(
              static_effort_(i),
              static_deadband_(i),
              p_gains_(i),
              joint_p_error_(i),
              joint_velocity_(i),
              static_eps_)
          + i_gains_(i)*joint_i_error_(i)
          + d_gains_(i)*joint_d_error_(i);
      }
    } else {
      if(0) {

        // Compute joint-space inertia matrix
        KDL::JntArray positions;
        positions.data = joint_position_;
        if(chain_dynamics_->JntToMass(positions, joint_inertia_) != 0) {
          RTT::log(RTT::Error) << "Could not compute joint space inertia." << RTT::endlog();
          this->error();
          return;
        }
        Eigen::MatrixXd H = (joint_inertia_.data); // + (eye + d_gain).inverse() * motor_inertia 

        joint_effort_ = 
          H * (joint_acceleration_cmd_
               + (p_gains_.array()*joint_p_error_.array()
                  + i_gains_.array()*joint_i_error_.array()                  
                  + d_gains_.array()*joint_d_error_.array()).matrix());
      } else {
        joint_effort_ = 
          (p_gains_.array()*joint_p_error_.array()
           + i_gains_.array()*joint_i_error_.array()                  
           + d_gains_.array()*joint_d_error_.array()).matrix();
      }
    }
    
    // Check for old tolerance violations
    if(tolerance_violations_ > 0) {
      RTT::log(RTT::Warning) << "PID command is now within tolerances." << RTT::endlog();
      // Reset violations
      tolerance_violations_ = 0;
    }
  }

  // Send joint efforts
  joint_effort_out_.write(joint_effort_);

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
    // Publish controller desired state
    joint_state_desired_.header.stamp = rtt_rosclock::host_now();
    joint_state_desired_.position.resize(n_dof_);
    joint_state_desired_.velocity.resize(n_dof_);
    joint_state_desired_.effort.resize(n_dof_);
    std::copy(joint_position_cmd_.data(), joint_position_cmd_.data() + n_dof_, joint_state_desired_.position.begin());
    std::copy(joint_velocity_cmd_.data(), joint_velocity_cmd_.data() + n_dof_, joint_state_desired_.velocity.begin());
    std::copy(joint_effort_.data(), joint_effort_.data() + n_dof_, joint_state_desired_.effort.begin());
    joint_state_desired_out_.write(joint_state_desired_);
  }
}

void JointPIDController::stopHook()
{
  joint_position_in_.clear();
  joint_velocity_in_.clear();
  joint_position_cmd_in_.clear();
  joint_velocity_cmd_in_.clear();
}

void JointPIDController::cleanupHook()
{
}

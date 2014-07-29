

#include <iostream>
#include <algorithm>
#include <map>

#include <Eigen/Dense>

#include <rtt/internal/GlobalService.hpp>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <kdl_urdf_tools/tools.h>
#include "joint_traj_generator_rml.h"

using namespace lcsr_controllers;

size_t JointTrajGeneratorRML::TrajSegment::segment_count = 0;

JointTrajGeneratorRML::JointTrajGeneratorRML(std::string const& name) :
  TaskContext(name)
  // Properties
  ,use_rosparam_(true)
  ,use_rostopic_(true)
  ,robot_description_("")
  ,robot_description_param_("/robot_description")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,verbose_(false)
  // Trajectory state
  ,segments_()
  ,active_segment_(0,false)
  // Behavior
  ,stop_on_violation_(true)
  ,traj_mode_(INACTIVE)
  // RML
  ,rml_zero_(0)
  ,rml_true_(0)
  // Debugging
  ,ros_publish_throttle_(0.02)
{
  // Declare properties
  this->addProperty("use_rosparam",use_rosparam_).doc("Fetch parameters from rosparam when configure() is called (true by default).");
  this->addProperty("robot_description",robot_description_).doc("The URDF xml string.");
  this->addProperty("robot_description_param",robot_description_param_).doc("The ROS parameter for the URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("max_velocities",max_velocities_).doc("Maximum velocities for traj generation.");
  this->addProperty("max_accelerations",max_accelerations_).doc("Maximum accelerations for traj generation.");
  this->addProperty("max_jerks",max_jerks_).doc("Maximum jerks for traj generation.");
  this->addProperty("position_tolerance",position_tolerance_).doc("Maximum position error.");
  this->addProperty("velocity_tolerance",velocity_tolerance_).doc("Maximum velocity error.");
  this->addProperty("sampling_resolution",sampling_resolution_).doc("Sampling resolution in seconds.");
  this->addProperty("stop_on_violation",stop_on_violation_).doc("Stop the trajectory if the tolerances are violated.");
  this->addProperty("verbose",verbose_).doc("Verbose debug output control.");
  
  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_)
    .doc("Current joint position. (required)");
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_)
    .doc("Current joint velocity. (required)");
  this->ports()->addPort("joint_position_cmd_in", joint_position_cmd_in_)
    .doc("Desired joint position, to be acquired as fast as possible.");
  this->ports()->addPort("joint_position_out", joint_position_out_)
    .doc("Interpolated joint position subject to velocity and acceleration limits.");
  this->ports()->addPort("joint_velocity_out", joint_velocity_out_)
    .doc("Interpolated joint velocity subject to velocity and acceleration limits.");
  this->ports()->addPort("joint_acceleration_out", joint_acceleration_out_)
    .doc("Interpolated joint acceleration subject to velocity and acceleration limits.");

  this->addOperation("setMaxVelocity",&JointTrajGeneratorRML::setMaxVelocity,this,RTT::OwnThread);
  this->addOperation("setMaxAcceleration",&JointTrajGeneratorRML::setMaxAcceleration,this,RTT::OwnThread);
  this->addOperation("setMaxJerk",&JointTrajGeneratorRML::setMaxJerk,this,RTT::OwnThread);

  // ROS ports
  this->ports()->addPort("joint_traj_point_cmd_in", joint_traj_point_cmd_in_);
  this->ports()->addPort("joint_traj_cmd_in", joint_traj_cmd_in_);
  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);

  // Add action server ports to this task's root service
  rtt_action_server_.addPorts(this->provides(), true, "~"+this->getName()+"/action/");

  // Bind action server goal and cancel callbacks (see below)
  rtt_action_server_.registerGoalCallback(boost::bind(&JointTrajGeneratorRML::goalCallback, this, _1));
  rtt_action_server_.registerCancelCallback(boost::bind(&JointTrajGeneratorRML::cancelCallback, this, _1));

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("joint_position_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_velocity_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_position_cmd_in", conman::Exclusivity::EXCLUSIVE);
}

bool JointTrajGeneratorRML::configureHook()
{
  // ROS topics
  if(   !joint_traj_cmd_in_.createStream(rtt_roscomm::topic("~" + this->getName() + "/joint_traj_cmd"))
     || !joint_traj_point_cmd_in_.createStream(rtt_roscomm::topic("~" + this->getName() + "/joint_traj_point_cmd"))
     || !joint_state_desired_out_.createStream(rtt_roscomm::topic("~" + this->getName() + "/joint_state_desired")))
  {
    RTT::log(RTT::Error) << "ROS Topics could not be streamed..." <<RTT::endlog();
    return false;
  }

  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam;

  if(use_rosparam_) {
    // Get the rosparam service provider
    rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
    
    // Only get kinematics from robot description if n_dof_ hasn't been set
    rosparam->getComponentPrivate("n_dof");
    rosparam->getComponentPrivate("root_link");
    rosparam->getComponentPrivate("tip_link");

    rosparam->getComponentPrivate("robot_description_param");
    rosparam->getParam(robot_description_param_, "robot_description");
    if(robot_description_.length() == 0) {
      RTT::log(RTT::Error) << "No robot description! Reading from parameter \"" << robot_description_param_ << "\"" << RTT::endlog();
      return false;
    }
  }

  // Initialize kinematics (KDL tree, KDL chain, and #DOF) from urdf
  if(robot_description_.length() > 0) {
    urdf::Model urdf_model;
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
            robot_description_, root_link_, tip_link_,
            n_dof_, kdl_chain, kdl_tree, urdf_model))
    {
      RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
      return false;
    }

    // Get joint names
    joint_names_.clear();
    joint_names_.reserve(n_dof_);
    int j=0;
    for(std::vector<KDL::Segment>::iterator it = kdl_chain.segments.begin();
        it != kdl_chain.segments.end();
        ++it)
    {
      joint_names_.push_back(it->getJoint().getName());
      joint_name_index_map_[joint_names_.back()] = j;
      j++;
    }
  } else {
    RTT::log(RTT::Error) << "URDF string is empty" << RTT::endlog();
    return false;
  }

  // Require some number of joints
  if(n_dof_ == 0) {
    RTT::log(RTT::Error) << "Number of degrees-of-freedom is zero. Please set this manually, via rosparam, or use an URDF with a root and tip link." << RTT::endlog();
    return false;
  }


  rml_zero_ = RMLDoubleVector(n_dof_);
  rml_zero_.Set(0.0);

  rml_true_ = RMLBoolVector(n_dof_);
  rml_true_.Set(true);

  // Get individual joint properties from urdf and parameter server
  position_tolerance_.conservativeResize(n_dof_);
  velocity_tolerance_.conservativeResize(n_dof_);
  max_velocities_.conservativeResize(n_dof_);
  max_accelerations_.conservativeResize(n_dof_);
  max_jerks_.conservativeResize(n_dof_);

  if(use_rosparam_) {
    rosparam->getComponentPrivate("position_tolerance");
    rosparam->getComponentPrivate("velocity_tolerance");
    rosparam->getComponentPrivate("max_velocities");
    rosparam->getComponentPrivate("max_accelerations");
    rosparam->getComponentPrivate("max_jerks");
    rosparam->getComponentPrivate("sampling_resolution");
    rosparam->getComponentPrivate("verbose");
    rosparam->getComponentPrivate("stop_on_violation");
  }
  
  // Resize IO vectors
  joint_zero_.resize(n_dof_);
  joint_zero_.setConstant(0.0);
  joint_position_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  joint_position_sample_.resize(n_dof_);
  joint_position_err_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_velocity_last_.resize(n_dof_);
  joint_velocity_sample_.resize(n_dof_);
  joint_velocity_err_.resize(n_dof_);
  joint_acceleration_.resize(n_dof_);
  joint_acceleration_sample_.resize(n_dof_);

  position_tolerance_violations_.assign(n_dof_,false);
  velocity_tolerance_violations_.assign(n_dof_,false);

  index_permutation_.resize(n_dof_);
  active_segment_ = TrajSegment(n_dof_,false);

  // Configure RML structures
  return this->configureRML(rml_, rml_in_, rml_out_, rml_flags_);
}

bool JointTrajGeneratorRML::configureRML(
    boost::shared_ptr<ReflexxesAPI> &rml,
    boost::shared_ptr<RMLPositionInputParameters> &rml_in,
    boost::shared_ptr<RMLPositionOutputParameters> &rml_out,
    RMLPositionFlags &rml_flags) const
{
  // Create trajectory generator
  rml.reset(new ReflexxesAPI(n_dof_, sampling_resolution_));
  rml_in.reset(new RMLPositionInputParameters(n_dof_));
  rml_out.reset(new RMLPositionOutputParameters(n_dof_));
  
  // Hold fixed at final point once trajectory is complete
  rml_flags.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
  rml_flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

  for(size_t j = 0; j < n_dof_; j++)
  { 
    // Get RML parameters from RTT Properties
    rml_in->MaxVelocityVector->VecData[j] =  max_velocities_[j];
    rml_in->MaxAccelerationVector->VecData[j] = max_accelerations_[j];
    rml_in->MaxJerkVector->VecData[j] = max_jerks_[j];

    // Enable this joint
    rml_in->SelectionVector->VecData[j] = true;
  }

  // Check if the reflexxes config is valud
  if(rml_in->CheckForValidity()) {
    RTT::log(RTT::Info) << ("RML INPUT Configuration Valid.") << RTT::endlog();
    RMLLog(RTT::Debug, rml_in);
  } else {
    RTT::log(RTT::Error) << ("RML INPUT Configuration Invalid!") << RTT::endlog();
    RTT::log(RTT::Error) << ("NOTE: MaxVelocityVector, MaxAccelerationVector, and MaxJerkVector must all be non-zero for a solution to exist.") << RTT::endlog();
    RMLLog(RTT::Error, rml_in);
    return false;
  }

  return true;
}

bool JointTrajGeneratorRML::startHook()
{
  segments_.clear();
  rtt_action_server_.start();

  joint_position_.setZero();
  joint_velocity_.setZero();
  joint_velocity_last_.setZero();
  joint_acceleration_.setZero();

  joint_position_in_.clear();
  joint_velocity_in_.clear();
  joint_position_cmd_in_.clear();
  joint_traj_point_cmd_in_.clear();
  joint_traj_cmd_in_.clear();

  // Enable all joints
  rml_in_->SetSelectionVector(rml_true_);

  return true;
}

bool JointTrajGeneratorRML::SpliceTrajectory(
    JointTrajGeneratorRML::TrajSegments &current_segments,
    const JointTrajGeneratorRML::TrajSegments &new_segments)
{
  // Make sure there are new segments
  if(new_segments.begin() == new_segments.end()) {
    return false;
  }

  // Determine where the segments should begin to be inserted in the current trajectory via binary search
  std::pair<TrajSegments::iterator, TrajSegments::iterator> insertion_range = 
    std::equal_range(
        current_segments.begin(),
        current_segments.end(), 
        new_segments.front(),
        TrajSegment::StartTimeCompare);

  // Remove all segments with start times after the start time of this trajectory
  current_segments.erase(insertion_range.first, current_segments.end());

  // Add the new segments to the end of the trajectory
  current_segments.insert(
      current_segments.end(), 
      new_segments.begin(), 
      new_segments.end());

  for(TrajSegments::iterator it = current_segments.begin();
      it != current_segments.end();
      ++it)
  {
    it->queued = true;
  }

  return true;
}

bool JointTrajGeneratorRML::TrajectoryMsgToSegments(
    const trajectory_msgs::JointTrajectory &msg,
    const std::vector<size_t> &ip,
    const size_t n_dof,
    const ros::Time new_traj_start_time,
    TrajSegments &segments,
    GoalHandle *gh,
    size_t *gh_required)
{
  // Clear the output segment list
  segments.clear();
  
  // Make sure the traj isn't empty
  if(msg.points.size() == 0) {
    return false;
  }

  // It's assumed that the header timestamp is when this new
  // trajectory should start being executed. This also means that if there
  // is a queued trajectory which contains this start time that that
  // trajectory should not be disturbed until that start time. We achieve
  // this by checking the next segment point's start time. If the start time is
  // 0.0 then it will not be activated until the end-time of the active segment
  // goal. If the start time is non-zero, then it will become active once
  // the start time has passed.

  // By default, start the trajectory now (header stamp is zero)
  //ros::Time new_traj_start_time = rtt_now;

  // Convert the ROS joint trajectory to a set of Eigen TrajSegment structures
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = msg.points.begin();
      it != msg.points.end();
      ++it) 
  {
    // Get a reference to the new segment
    TrajSegment new_segment(n_dof);

    // Create and add the new segment 
    if(it->time_from_start.isZero()) {
      new_segment.flexible = true;
    }

    // Add actionlib goal information
    if(gh != NULL && gh->isValid()) {
      // Store the pointers
      new_segment.gh = gh;
      new_segment.gh_required = gh_required;

      // Increment required counter
      (*gh_required)++;
    }

    // Compute the start time for the new segment. If this is the first
    // point, then it's the trajectory start time. Otherwise, it's the end
    // time of the preceeding point.
    if(it == msg.points.begin()) {
      new_segment.start_time = new_traj_start_time;
    } else {
      new_segment.start_time = segments.back().goal_time;
    }
    new_segment.goal_time = new_traj_start_time + it->time_from_start;
    new_segment.expected_time = new_segment.goal_time;

    // Copy in the data (permuted appropriately)
    for(int j=0; j<n_dof; j++) {
      if(j < it->positions.size()) new_segment.goal_positions(ip[j]) = it->positions[j];
      if(j < it->velocities.size()) new_segment.goal_velocities(ip[j]) = it->velocities[j];
      if(j < it->accelerations.size()) new_segment.goal_accelerations(ip[j]) = it->accelerations[j];
    }

    segments.push_back(new_segment);
  }

  return true;
}

void JointTrajGeneratorRML::handleSampleError(const std::runtime_error &err)
{
  RTT::log(RTT::Error) << "Error while sampling trajectory: " << err.what() << RTT::endlog();

  // Enter error state
  this->error();
}

void JointTrajGeneratorRML::handleRMLResult(int rml_result) const
{
  try {
    switch(rml_result) 
    {
      case ReflexxesAPI::RML_WORKING:
      case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        // S'all good.
        break;
      case ReflexxesAPI::RML_ERROR_INVALID_INPUT_VALUES:
        throw std::runtime_error("Reflexxes error RML_ERROR_INVALID_INPUT_VALUES in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_CALCULATION:
        throw std::runtime_error("Reflexxes error RML_ERROR_EXECUTION_TIME_CALCULATION in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_SYNCHRONIZATION:
        throw std::runtime_error("Reflexxes error RML_ERROR_SYNCHRONIZATION in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_NUMBER_OF_DOFS:
        throw std::runtime_error("Reflexxes error RML_ERROR_NUMBER_OF_DOFS in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_NO_PHASE_SYNCHRONIZATION:
        throw std::runtime_error("Reflexxes error RML_ERROR_NO_PHASE_SYNCHRONIZATION in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_NULL_POINTER:
        throw std::runtime_error("Reflexxes error RML_ERROR_NULL_POINTER in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_EXECUTION_TIME_TOO_BIG:
        throw std::runtime_error("Reflexxes error RML_ERROR_EXECUTION_TIME_TOO_BIG in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR_USER_TIME_OUT_OF_RANGE:
        throw std::runtime_error("Reflexxes error RML_ERROR_USER_TIME_OUT_OF_RANGE in JointTrajGeneratorRML::sampleTrajectory");
      case ReflexxesAPI::RML_ERROR:
        throw std::runtime_error("Reflexxes error RML_ERROR in JointTrajGeneratorRML::sampleTrajectory");
      default:
        throw std::runtime_error("Reflexxes error in JointTrajGeneratorRML::sampleTrajectory");
    };
  } catch(...) {
    RTT::log(RTT::Error) << "Reflexxes error code: "<<rml_result<<". Entering error state and not writing a desired position." << RTT::endlog();
    throw;
  }
}

void JointTrajGeneratorRML::computeTrajectory(
    const ros::Time rtt_now,
    const Eigen::VectorXd &init_position,
    const Eigen::VectorXd &init_velocity,
    const Eigen::VectorXd &init_acceleration,
    const ros::Duration duration,
    const Eigen::VectorXd &goal_position,
    const Eigen::VectorXd &goal_velocity,
    boost::shared_ptr<ReflexxesAPI> rml,
    boost::shared_ptr<RMLPositionInputParameters> rml_in,
    boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    RMLPositionFlags &rml_flags) const
{
  // Update RML input parameters
  rml_in->SetMaxVelocityVector(&max_velocities_[0]);
  rml_in->SetMaxAccelerationVector(&max_accelerations_[0]);
  rml_in->SetMaxJerkVector(&max_jerks_[0]);

  // Set initial params
  for(size_t i=0;i<n_dof_;i++) { 
    rml_in->SetSelectionVectorElement(true,i);
  }

  // Override initial state if necessary
  rml_in->SetCurrentPositionVector(init_position.data());
  rml_in->SetCurrentVelocityVector(init_velocity.data());
  rml_in->SetCurrentAccelerationVector(init_acceleration.data());

  // Set goal params
  rml_in->SetTargetPositionVector(goal_position.data());
  rml_in->SetTargetVelocityVector(goal_velocity.data());

  // Compute the trajectory
  if(verbose_) RTT::log(RTT::Debug) << ("RML Recomputing trajectory...") << RTT::endlog(); 

  // Set desired execution time for this trajectory (definitely > 0)
  rml_in->SetMinimumSynchronizationTime(std::max(0.0,duration.toSec()));

  // Verbose logging
  if(verbose_) RTT::log(RTT::Debug) << "RML IN: time: "<<rml_in->GetMinimumSynchronizationTime() << RTT::endlog();
  if(verbose_) RMLLog(RTT::Info, rml_in);

  // Compute trajectory
  int rml_result = rml->RMLPosition(
      *rml_in.get(), 
      rml_out.get(), 
      rml_flags);

  // Get expected time
  if(verbose_) RTT::log(RTT::Debug) << "RML OUT: time: "<<rml_out->GetGreatestExecutionTime() << RTT::endlog();

  // Throw exception on result
  this->handleRMLResult(rml_result);
}

void JointTrajGeneratorRML::computeTrajectory(
    const ros::Time rtt_now,
    const ros::Duration duration,
    const Eigen::VectorXd &goal_position,
    const Eigen::VectorXd &goal_velocity,
    boost::shared_ptr<ReflexxesAPI> rml,
    boost::shared_ptr<RMLPositionInputParameters> rml_in,
    boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    RMLPositionFlags &rml_flags) const
{
  this->computeTrajectory(
      rtt_now,
      Eigen::Map<Eigen::VectorXd>(rml_out->NewPositionVector->VecData, n_dof_),
      Eigen::Map<Eigen::VectorXd>(rml_out->NewVelocityVector->VecData, n_dof_),
      Eigen::Map<Eigen::VectorXd>(rml_out->NewAccelerationVector->VecData, n_dof_),
      duration,
      goal_position,
      goal_velocity,
      rml, rml_in, rml_out, rml_flags);
}

bool JointTrajGeneratorRML::updateSegments(
    const ros::Time rtt_now,
    const Eigen::VectorXd &joint_position,
    const Eigen::VectorXd &joint_velocity,
    JointTrajGeneratorRML::TrajSegments &segments) const
{
  // Find segments which should be removed
  // - segments which are not flexible
  // - segments whose expected goal times are in the past
  // - segments before segments whose start times are now or in the past

  bool recompute_trajectory = false;
  bool remove_old_segments = false;
  TrajSegments::iterator erase_it = segments.begin();

  for(TrajSegments::iterator it = segments.begin(); it != segments.end(); ++it) 
  {
    // Peek at the next segment
    TrajSegments::iterator next = it;  ++next;

    // Mark this segment for removal if it's expired or if we need to start processing the next one
    // This only applies to non-flexible segments
    if(  
        (it->achieved) ||                                  // Has already been achieved
        (it->gh !=NULL && (!it->gh->isValid() || it->gh->getGoalStatus().status != actionlib_msgs::GoalStatus::ACTIVE)) ||
        //(it->achieved && next->flexible) ||
        (!it->flexible && it->expected_time < rtt_now) ||  // Should have finished earlier than now
        (next != segments.end() && !next->flexible && next->start_time <= rtt_now)) // Next segment should have started
    {
      // Mark trivial segments as achieved
      if(!it->achieved) {
        // Check each dof
        bool within_tolerance = false;
        for(int i=0; i<n_dof_; i++) 
        {
          if(std::abs(it->goal_positions(i) - joint_position[i]) <= position_tolerance_[i]) {
            within_tolerance = true;
          }
          if(std::abs(it->goal_velocities(i) - joint_velocity[i]) <= velocity_tolerance_[i])  {
            within_tolerance = true;
          }
        }

        // Achieve it if it's within tolerance
        if(within_tolerance) {
          it->achieved = true;
        }
      }

      // Update the erasure iterator
      erase_it = it;
      remove_old_segments = true;

      if(verbose_) {
        RTT::log(RTT::Debug) << "Segment ("<<it->id<<") needs to be removed." <<RTT::endlog();
        RTT::log(RTT::Debug) << " - goal status: "<< ((it->gh == NULL) ? (-1) : int(it->gh->getGoalStatus().status)) << RTT::endlog();
        RTT::log(RTT::Debug) << " - achieved: "<<it->achieved  << RTT::endlog();
        RTT::log(RTT::Debug) << " - flexible: "<<it->flexible  <<  RTT::endlog();
        RTT::log(RTT::Debug) << " - start_time: "<<it->start_time  <<  RTT::endlog();
        RTT::log(RTT::Debug) << " - goal_time: "<<it->goal_time  <<  RTT::endlog();
        RTT::log(RTT::Debug) << " - expected_time: "<<it->expected_time  <<  RTT::endlog();
      }
    }
  }

  // Remove the segments whose end times are in the past, if necessary
  if(remove_old_segments) {
    if(verbose_) RTT::log(RTT::Debug) << "Removing old segments..." << RTT::endlog();
    segments.erase(segments.begin(), ++erase_it);
    recompute_trajectory =  true; 
    if(segments.empty()) {
      if(verbose_) RTT::log(RTT::Debug) << "The trajectory is now empty." << RTT::endlog();
    } 
  }

  // Check if there are any segments which may need to be activated
  if(!segments.empty()) {
    // Get the active segment
    TrajSegments::iterator front_segment = segments.begin();

    // Check if we should be processing the front segment 
    // (if it has a flexible start time or should have been started by now)
    if(!front_segment->active) {
       if((front_segment->flexible || front_segment->start_time <= rtt_now)) 
       {
         // Activate the front segment if it's not already active
         if(verbose_) RTT::log(RTT::Debug) << "Front segment ("<<front_segment->id<<") is not active. Activating..." << RTT::endlog();
         front_segment->active = true;
         recompute_trajectory = true;
       } else {
         //if(verbose_) RTT::log(RTT::Debug) << "The front segment is neither flexible nor ready." << RTT::endlog();
       }
    }
  }

  return recompute_trajectory;
}

bool JointTrajGeneratorRML::tolerancesViolated(
    const Eigen::VectorXd &joint_position,
    const Eigen::VectorXd &joint_velocity,
    const Eigen::VectorXd &joint_acceleration,
    const boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    std::vector<bool> &position_tolerance_violations,
    std::vector<bool> &velocity_tolerance_violations) const
{

  bool tolerances_violated = false;

  // Determine if any of the joint tolerances have been violated (this means we need to recompute the traj)
  for(int i=0; i<n_dof_; i++) 
  {
    double position_tracking_error = std::abs(rml_out->GetNewPositionVectorElement(i) - joint_position[i]);
    double velocity_tracking_error = std::abs(rml_out->GetNewVelocityVectorElement(i) - joint_velocity[i]);

    if(position_tracking_error > position_tolerance_[i]) {
      tolerances_violated = true;

      if(verbose_ && !position_tolerance_violations[i]) {
        RTT::log(RTT::Warning) << "["<<this->getName() <<"] Joint " << i << 
          " position error tolerance violated ( |"<<rml_out->GetNewPositionVectorElement(i)<<" - "<<joint_position[i]<<"| = "<<position_tracking_error<<" > "<<position_tolerance_[i]<<")" 
          << RTT::endlog(); 
        position_tolerance_violations[i] = true;
      }
    } else {
      position_tolerance_violations[i] = false;
    }

    if(velocity_tracking_error > velocity_tolerance_[i])  {
      tolerances_violated = true;

      if(verbose_ && !velocity_tolerance_violations[i]) {
        RTT::log(RTT::Warning) << "["<<this->getName() <<"] Joint " << i << 
          " velocity error tolerance violated ( |"<<rml_out->GetNewVelocityVectorElement(i)<<" - "<<joint_velocity[i]<<"| = "<<velocity_tracking_error<<" > "<<velocity_tolerance_[i]<<")" 
          << RTT::endlog(); 
        velocity_tolerance_violations[i] = true;
      }
    } else {
      velocity_tolerance_violations[i] = false;
    }
  }

  return tolerances_violated;
}

bool JointTrajGeneratorRML::computeTrajectory(
    const ros::Time rtt_now,
    const JointTrajGeneratorRML::TrajSegments::iterator active_segment,
    boost::shared_ptr<ReflexxesAPI> rml,
    boost::shared_ptr<RMLPositionInputParameters> rml_in,
    boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    RMLPositionFlags &rml_flags) const
{
#if 0
  if(tolerances_violated)
  {
    if(verbose_) RTT::log(RTT::Debug) << ("Tolerances violated.") << RTT::endlog(); 
    if(stop_on_violation_) {
      rml_in->SetCurrentPositionVector(joint_position.data());
      rml_in->SetCurrentVelocityVector(rml_zero_);
      rml_in->SetCurrentAccelerationVector(rml_zero_);
      active_segment->goal_positions = joint_position;
      active_segment->goal_velocities.setConstant(0.0);
      // Compute the trajectory
      if(verbose_) RTT::log(RTT::Debug) << ("Terminating trajectory.") << RTT::endlog(); 
      return false;
    } else {
      rml_in->SetCurrentPositionVector(joint_position.data());
      rml_in->SetCurrentVelocityVector(joint_velocity.data());
      rml_in->SetCurrentAccelerationVector(rml_zero_);
    }

    rml_in->SetTargetPositionVector(active`_segment->goal_positions.data());
    rml_in->SetTargetVelocityVector(active_segment->goal_velocities.data());
  }
  else
#endif

  // Compute a new trajectory based on initial and goal states
  this->computeTrajectory(
      rtt_now,
      active_segment->goal_time - rtt_now,
      active_segment->goal_positions,
      active_segment->goal_velocities,
      rml, rml_in, rml_out, rml_flags);

  // Store segment start time and expected time
  active_segment->start_time = rtt_now;
  active_segment->expected_time = rtt_now + ros::Duration(rml_out->GetGreatestExecutionTime());

  // If the goal time is zero, then it should be executed as fast as possible, subject to the constraints
  if(active_segment->flexible) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "Extending trajectory segment time to: "<<rml_out->GetGreatestExecutionTime() << RTT::endlog();
    active_segment->goal_time = active_segment->expected_time;
  }
  else if(active_segment->expected_time > active_segment->goal_time) 
  {
    RTT::log(RTT::Error) << "Dropping trajectory segment because segment cannot be reached in the desired time. " 
      << "Desired: " << active_segment->goal_time - active_segment->start_time 
      << " Required: " << active_segment->expected_time - active_segment->start_time
      << RTT::endlog();
    return false;
  }

  return true;
} 

bool JointTrajGeneratorRML::sampleTrajectory(
    const ros::Time rtt_now,
    const ros::Time last_segment_start_time,
    boost::shared_ptr<ReflexxesAPI> rml,
    boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    Eigen::VectorXd &joint_position_sample,
    Eigen::VectorXd &joint_velocity_sample,
    Eigen::VectorXd &joint_acceleration_sample) const
{
  // Sample the already computed trajectory
  int rml_result = rml->RMLPositionAtAGivenSampleTime(
      std::max(0.0, (rtt_now - last_segment_start_time).toSec()),
      rml_out.get());

  this->handleRMLResult(rml_result);

  // Only set a non-zero effort command if the 
  if(rml_result == ReflexxesAPI::RML_WORKING || rml_result == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
    // Get the new sampled reference
    for(size_t i=0; i<n_dof_; i++) {
      joint_position_sample(i) = rml_out->GetNewPositionVectorElement(i);
      joint_velocity_sample(i) = rml_out->GetNewVelocityVectorElement(i);
      joint_acceleration_sample(i) = rml_out->GetNewAccelerationVectorElement(i);
    }

    // Return true if the segment is complete
    if(rml_result == ReflexxesAPI::RML_FINAL_STATE_REACHED) {
      return true;
    }
  }

  this->handleRMLResult(rml_result);

  return false;
}

bool JointTrajGeneratorRML::readCommands(
    const ros::Time &rtt_now) 
{
  // If the trajectory should be continued or stopped (empty traj command)
  bool continue_traj = true;

  // Read in any newly commanded joint positions 
  RTT::FlowStatus point_status = joint_position_cmd_in_.readNewest( joint_position_cmd_ );
  RTT::FlowStatus traj_point_status = joint_traj_point_cmd_in_.readNewest( joint_traj_point_cmd_ );
  RTT::FlowStatus traj_status = joint_traj_cmd_in_.readNewest( joint_traj_cmd_ );

  // Check if there's a new desired point
  if(point_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New point." <<RTT::endlog();
    continue_traj = this->insertSegments(
        joint_position_cmd_,
        rtt_now, 
        segments_, 
        index_permutation_);
  } 
  // Check if there's a new desired trajectory point
  else if(traj_point_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New trajectory point." <<RTT::endlog();
    continue_traj = this->insertSegments(
        joint_traj_point_cmd_, 
        rtt_now, 
        segments_, 
        index_permutation_);
  }
  // Check if there's a new desired trajectory
  else if(traj_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New trajectory message." <<RTT::endlog();
    continue_traj = this->insertSegments(
        joint_traj_cmd_, 
        rtt_now, 
        segments_, 
        index_permutation_);
  }

  return continue_traj;
}

bool JointTrajGeneratorRML::insertSegments(
        const Eigen::VectorXd &point,
        const ros::Time &time,
        TrajSegments &segments,
        std::vector<size_t> &index_permutation) const
{
  // Check the size of the jointspace command
  if(point.size() == n_dof_) {
    // Handle a position given as an Eigen vector
    TrajSegment segment(n_dof_,true);

    segment.start_time = time;
    segment.goal_positions = point;
    segments.clear();
    segments.push_back(segment);
  } else {
    RTT::log(RTT::Debug) << "Received trajectory of invalid size." <<RTT::endlog();
    return false;
  }

  return true;
}


bool JointTrajGeneratorRML::insertSegments(
    const trajectory_msgs::JointTrajectoryPoint &traj_point,
    const ros::Time &time,
    TrajSegments &segments,
    std::vector<size_t> &index_permutation) const
{
  // Clear the segments, since this starts immediately
  segments.clear();
  // Create a unary trajectory with zero as the desired start time (start immediately)
  trajectory_msgs::JointTrajectory unary_joint_traj;
  unary_joint_traj.points.push_back(traj_point);

  return this->insertSegments(unary_joint_traj, time, segments, index_permutation);
}

bool JointTrajGeneratorRML::insertSegments(
    const trajectory_msgs::JointTrajectory &trajectory,
    const ros::Time &time,
    TrajSegments &segments,
    std::vector<size_t> &index_permutation,
    GoalHandle *gh,
    size_t *gh_required) const
{
  // Check if the traj is empty
  if(trajectory.points.size() == 0) {
    if(verbose_) RTT::log(RTT::Debug) << "Received empty trajectory, stopping arm." <<RTT::endlog();
    return false;
  } else {
    // Create a new list of segments to be spliced in
    TrajSegments new_segments;
    // By default, set the start time to now
    ros::Time new_traj_start_time = time;

    // If the header stamp is non-zero, then determine which points we should pursue
    if(trajectory.header.stamp.isZero()) {
      segments.clear();
    } else {
      // Offset the NTP-corrected time to get the RTT-time
      // Correct the timestamp so that its relative to the realtime clock
      // TODO: make it so this can be disabled or make two different ports
      try {
        new_traj_start_time = rtt_rosclock::rtt_now() + (trajectory.header.stamp - rtt_rosclock::host_now());
      } catch(std::runtime_error &err) {
        RTT::log(RTT::Info) << "Header Stamp: " << trajectory.header.stamp <<RTT::endlog();
        RTT::log(RTT::Info) << "RTT Now: " << rtt_rosclock::rtt_now() <<RTT::endlog();
        RTT::log(RTT::Info) << "Host Now: " << rtt_rosclock::host_now() <<RTT::endlog();
        RTT::log(RTT::Error) << "error: " << err.what() << RTT::endlog();
        throw;
      }
    }

    // Get the proper index permutation
    this->getIndexPermutation(trajectory.joint_names, index_permutation);

    // Convert the trajectory message to a list of segments for splicing
    TrajectoryMsgToSegments(
        trajectory, 
        index_permutation,
        n_dof_, 
        new_traj_start_time, 
        new_segments,
        gh, 
        gh_required);

    // Update the trajectory
    SpliceTrajectory(segments, new_segments);
  }

  return true;
}

void JointTrajGeneratorRML::updateHook()
{
  // Get the current and the time since the last update
  /*
   *const RTT::Seconds 
   *  time = conman_hook_->getTime(), 
   *  period = conman_hook_->getPeriod();
   */

  ros::Time rtt_now = rtt_rosclock::rtt_now();

  // Read in the current joint positions & velocities
  RTT::FlowStatus position_status = joint_position_in_.readNewest(joint_position_);
  RTT::FlowStatus velocity_status = joint_velocity_in_.readNewest(joint_velocity_);

  // If we don't get a position or velocity update, we don't write any new data to the ports
  if(position_status == RTT::OldData || velocity_status == RTT::OldData
     || joint_position_.size() != n_dof_
     || joint_velocity_.size() != n_dof_) 
  {
    // Return if we didn't get state input
    return;
  }

  //joint_velocity_ = 0.98*joint_velocity_last_ + 0.02*joint_velocity_;
  joint_acceleration_ = 0.1*joint_acceleration_ + 0.9*(joint_velocity_ - joint_velocity_last_);
  joint_velocity_last_ = joint_velocity_;

  // Check tolerances if in following mode
  if(traj_mode_ == FOLLOWING) 
  {
    // Sample the pre-computed trajectory the current time to get the expected joint state
    rml_->RMLPositionAtAGivenSampleTime(
        std::max(0.0,(rtt_now - last_segment_start_time_).toSec()),
        rml_out_.get());

    // Check error tolerances
    bool tolerances_violated = this->tolerancesViolated(
        joint_position_, 
        joint_velocity_, 
        joint_acceleration_,
        rml_out_,
        position_tolerance_violations_, 
        velocity_tolerance_violations_);

    // Sample the trajectory as it stands if the tolerances have not been violated
    if(tolerances_violated) 
    { 
      if(stop_on_violation_) {
        RTT::log(RTT::Warning) << "JointTrajGeneratorRML: Tolerances violated, stopping execution and dropping trajectory." << RTT::endlog();
        traj_mode_ = INACTIVE;
      } else {
        RTT::log(RTT::Warning) << "JointTrajGeneratorRML: Tolerances violated, attemping to recover..." << RTT::endlog();
        // Create traj segment with goal @ current position @ zero velocity
        TrajSegment recovery_segment(n_dof_, true);
        recovery_segment.goal_positions = joint_position_;

        // Deactivate the active traj segment
        if(!segments_.empty() && segments_.begin()->active) {
          segments_.begin()->active;
        }
        
        // Insert the recovery segment
        segments_.push_front(recovery_segment);

        // Switch to recovering mode
        traj_mode_ = RECOVERING;
      }
    } 
    else
    {
      // Read the command inputs
      bool continue_traj = this->readCommands(rtt_now);

      // Handle actionlib goal
      if(current_gh_.isValid()) 
      {
        switch(current_gh_.getGoalStatus().status) {
          case actionlib_msgs::GoalStatus::PENDING:
            {
              // Update the trajectory
              RTT::log(RTT::Debug) << "New trajectory action goal." <<RTT::endlog();

              // Reset the segment counter
              gh_segments_required_ = 0;

              // Insert the segments
              continue_traj = this->insertSegments(
                  current_gh_.getGoal()->trajectory, 
                  rtt_now, 
                  segments_, 
                  index_permutation_,
                  &current_gh_, &gh_segments_required_);

              // Accept the goal
              current_gh_.setAccepted();
              break;
            }
          case actionlib_msgs::GoalStatus::RECALLING:
          case actionlib_msgs::GoalStatus::PREEMPTING:
            {
              RTT::log(RTT::Debug) << "Trajectory action goal has been preempted." <<RTT::endlog();
              // Preempt the trajectory
              current_gh_.setCanceled();
              // Hold current position
              continue_traj = false;
              break;
            }
          case actionlib_msgs::GoalStatus::ACTIVE:
            {
              if(segments_.empty()) {
                RTT::log(RTT::Debug) << "Trajectory action goal has failed." <<RTT::endlog();
                current_gh_.setAborted();
              }
              break;
            }
          case actionlib_msgs::GoalStatus::PREEMPTED:
          case actionlib_msgs::GoalStatus::SUCCEEDED:
          case actionlib_msgs::GoalStatus::ABORTED:
          case actionlib_msgs::GoalStatus::REJECTED:
          case actionlib_msgs::GoalStatus::RECALLED:
            // replace the current goalhandle with an invalid one
            // NO! if we do this, we won't send any samples afterwards
            current_gh_ = GoalHandle();
            break;
          default:
            RTT::log(RTT::Warning) << "Trajectory action goal is in a bad state." <<RTT::endlog();
            break;
        };
      }     

      // Check if the trajectory should be continued
      if(!continue_traj) {
        traj_mode_ = INACTIVE;
      }

    }
  }

  // Switch behavior based on the current mode
  switch(traj_mode_)
  {
    case INACTIVE:
      // Seed the trajectory generator with the current position of the arm
      // NOTE: This ignores all inputs until in the "following" mode
      {
        // Clear trajectory
        segments_.clear();

        // Run RML once with the current joint state as both the initial state and goal
        this->computeTrajectory(
            rtt_now,
            joint_position_,
            joint_zero_,
            joint_zero_,
            ros::Duration(0.0),
            joint_position_,
            joint_zero_,
            rml_, rml_in_, rml_out_, rml_flags_);

        last_segment_start_time_ = rtt_now;

        // Sample the trajectory
        this->sampleTrajectory(
            rtt_now, 
            last_segment_start_time_,
            rml_, rml_out_,
            joint_position_sample_, 
            joint_velocity_sample_, 
            joint_acceleration_sample_);

        // Switch to following mode
        traj_mode_ = FOLLOWING;

        break;
      }

    case FOLLOWING:
      // Sample the active trajectory with tolerance checking
      {
        // The trajectory needs to be recomputed whenever the front segment changes
        bool recompute_trajectory = true;

        // Recompute the trajectory until the front segment is achievable
        // Normally this will only run once, unless multiple segments in the traj have invalid constraints on them
        while(recompute_trajectory) 
        {
          // Update / prune list of active segments 
          // This handles segments from a high-level specification
          // It "activates" segments when they are ready to be pursued
          recompute_trajectory = this->updateSegments(
              rtt_now, 
              joint_position_,
              joint_velocity_,
              segments_);

          // Recompute the trajectory if needed
          // This handles segments based on joint velocity/accel limits
          if(recompute_trajectory) 
          {
            // Check if we should pursue a new front segment (it exists and updateSegments has marked it as active)
            if(!segments_.empty() && segments_.begin()->active) 
            {
              // Recompute the trajectory
              bool active_segment_feasible = this->computeTrajectory(
                  rtt_now,
                  segments_.begin(),
                  rml_,
                  rml_in_,
                  rml_out_,
                  rml_flags_);

              // Check if the new point is achievable, otherwise, remove it and re-update the segments
              if(active_segment_feasible) { 
                // End the loop if the active segment is feasible
                recompute_trajectory = false;
              } else {
                // Remove the segment
                segments_.pop_front();
              }
            } 
            else
            {
              // Hold the current position
              this->computeTrajectory(
                  rtt_now,
                  ros::Duration(0.0),
                  joint_position_,
                  joint_zero_,
                  rml_,
                  rml_in_,
                  rml_out_,
                  rml_flags_);
            }

            // Store the last segment start time
            last_segment_start_time_ = segments_.begin()->start_time;
          }
        }

        // Sample current trajectory as computed above
        try {
          bool segment_complete = this->sampleTrajectory(
              rtt_now, 
              last_segment_start_time_,
              rml_, rml_out_,
              joint_position_sample_, 
              joint_velocity_sample_, 
              joint_acceleration_sample_);

          // Pop the segment if it's complete
          if(!segments_.empty() && segment_complete) {
            segments_.pop_front();
          }

        } catch (std::runtime_error &err) {
          // Handle the error in a nice way
          RMLLog(RTT::Error, rml_in_);
          this->handleSampleError(err);
          return;
        }

        break;
      }

    case RECOVERING:
      // Sample the active trajectory without tolerance checking
      // NOTE: This ignores all inputs until in the "following" mode
      {
        bool segment_complete = this->sampleTrajectory(
            rtt_now, 
            last_segment_start_time_,
            rml_, rml_out_,
            joint_position_sample_, 
            joint_velocity_sample_, 
            joint_acceleration_sample_);

        // Pop the segment if it's complete
        if(segment_complete) {
          if(!segments_.empty()) {
            segments_.pop_front();
          }

          traj_mode_ = FOLLOWING;
        }

        break;
      }
  };

  // Send instantaneous joint position and velocity commands
  joint_position_out_.write(joint_position_sample_);
  joint_velocity_out_.write(joint_velocity_sample_);
  joint_acceleration_out_.write(joint_acceleration_sample_);

  // Compute error
  joint_position_err_ = joint_position_sample_ - joint_position_;
  joint_velocity_err_ = joint_velocity_sample_ - joint_velocity_;

  // Publish debug traj to ros
  if(ros_publish_throttle_.ready(0.02)) 
  {
    // Publish controller desired state
    joint_state_desired_.header.stamp = rtt_rosclock::host_now();
    joint_state_desired_.position.resize(n_dof_);
    joint_state_desired_.velocity.resize(n_dof_);
    std::copy(joint_position_sample_.data(), joint_position_sample_.data() + n_dof_, joint_state_desired_.position.begin());
    std::copy(joint_velocity_sample_.data(), joint_velocity_sample_.data() + n_dof_, joint_state_desired_.velocity.begin());
    joint_state_desired_out_.write(joint_state_desired_);

    // Publish action feedback
    if(current_gh_.isValid() && current_gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
      feedback_.header = joint_state_desired_.header;

      feedback_.joint_names = joint_names_;

      feedback_.desired.positions.resize(n_dof_);
      feedback_.desired.velocities.resize(n_dof_);
      feedback_.actual.positions.resize(n_dof_);
      feedback_.actual.velocities.resize(n_dof_);
      feedback_.error.positions.resize(n_dof_);
      feedback_.error.velocities.resize(n_dof_);

      std::copy(joint_position_sample_.data(), joint_position_sample_.data() + n_dof_, feedback_.desired.positions.begin());
      std::copy(joint_velocity_sample_.data(), joint_velocity_sample_.data() + n_dof_, feedback_.desired.velocities.begin());

      std::copy(joint_position_.data(), joint_position_.data() + n_dof_, feedback_.actual.positions.begin());
      std::copy(joint_velocity_.data(), joint_velocity_.data() + n_dof_, feedback_.actual.velocities.begin());

      std::copy(joint_position_err_.data(), joint_position_err_.data() + n_dof_, feedback_.error.positions.begin());
      std::copy(joint_velocity_err_.data(), joint_velocity_err_.data() + n_dof_, feedback_.error.velocities.begin());

      current_gh_.publishFeedback(feedback_);
    }
  }
}

void JointTrajGeneratorRML::stopHook()
{
  // TODO: rtt_action_server_.stop();
  // Clear data buffers (this will make them return OldData if nothing new is written to them)
  joint_position_in_.clear();
  joint_velocity_in_.clear();
  // Clear segments / abort goal
  segments_.clear();
}

void JointTrajGeneratorRML::cleanupHook()
{
}

void JointTrajGeneratorRML::errorHook()
{
  // Clear segments / abort goal
  segments_.clear();
}


template<class T>
std::ostream& operator<< (std::ostream& stream, const RMLVector<T>& rml_vec) {
  stream<<"[ ";
  for(int i=0; i<rml_vec.VectorDimension; i++) { stream<<(rml_vec.VecData[i])<<", "; }
  stream<<"]";
  return stream;
}


void JointTrajGeneratorRML::RMLLog(
    const RTT::LoggerLevel level,
    const boost::shared_ptr<RMLPositionInputParameters> rml_in) 
{
  RTT::log(level) << "RML INPUT: "<< RTT::endlog();
  RTT::log(level) << " - NumberOfDOFs:               "<<rml_in->NumberOfDOFs << RTT::endlog();
  RTT::log(level) << " - MinimumSynchronizationTime: "<<rml_in->MinimumSynchronizationTime << RTT::endlog();

  RTT::log(level) << " - SelectionVector: "<<*(rml_in->SelectionVector) << RTT::endlog();

  RTT::log(level) << " - CurrentPositionVector:     "<<*(rml_in->CurrentPositionVector) << RTT::endlog();
  RTT::log(level) << " - CurrentVelocityVector:     "<<*(rml_in->CurrentVelocityVector) << RTT::endlog();
  RTT::log(level) << " - CurrentAccelerationVector: "<<*(rml_in->CurrentAccelerationVector) << RTT::endlog();

  RTT::log(level) << " - MaxVelocityVector:     "<<*(rml_in->MaxVelocityVector) << RTT::endlog();
  RTT::log(level) << " - MaxAccelerationVector: "<<*(rml_in->MaxAccelerationVector) << RTT::endlog();
  RTT::log(level) << " - MaxJerkVector:         "<<*(rml_in->MaxJerkVector) << RTT::endlog();

  RTT::log(level) << " - TargetPositionVector:            "<<*(rml_in->TargetPositionVector) << RTT::endlog();
  RTT::log(level) << " - TargetVelocityVector:            "<<*(rml_in->TargetVelocityVector) << RTT::endlog();

  RTT::log(level) << " - AlternativeTargetVelocityVector: "<<*(rml_in->AlternativeTargetVelocityVector) << RTT::endlog();
}

void JointTrajGeneratorRML::goalCallback(JointTrajGeneratorRML::GoalHandle gh)
{
  RTT::log(RTT::Info) << "Recieved action goal." << RTT::endlog();

  // Always preempt the current goal and accept the new one
  if(current_gh_.isValid() && current_gh_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
    current_gh_.setCanceled();
  }
  current_gh_ = gh;
  RTT::log(RTT::Debug) << "Action goal status is :" << (int)current_gh_.getGoalStatus().status << RTT::endlog();
}

void JointTrajGeneratorRML::cancelCallback(JointTrajGeneratorRML::GoalHandle gh)
{
  RTT::log(RTT::Info) << "Recieved action preemption." << RTT::endlog();
}

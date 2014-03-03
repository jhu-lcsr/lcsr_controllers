

#include <iostream>
#include <algorithm>
#include <map>

#include <Eigen/Dense>

#include <rtt/internal/GlobalService.hpp>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rostopic/rostopic.h>

#include <kdl_urdf_tools/tools.h>
#include "joint_traj_generator_rml.h"

using namespace lcsr_controllers;

JointTrajGeneratorRML::JointTrajGeneratorRML(std::string const& name) :
  TaskContext(name)
  // Properties
  ,use_rosparam_(true)
  ,use_rostopic_(true)
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,verbose_(false)
  // Trajectory state
  ,segments_()
  // Debuggin
  ,ros_publish_throttle_(0.02)
{
  // Declare properties
  this->addProperty("use_rosparam",use_rosparam_).doc("Fetch parameters from rosparam when configure() is called (true by default).");
  this->addProperty("robot_description",robot_description_).doc("The URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("max_velocities",max_velocities_).doc("Maximum velocities for traj generation.");
  this->addProperty("max_accelerations",max_accelerations_).doc("Maximum accelerations for traj generation.");
  this->addProperty("max_jerks",max_jerks_).doc("Maximum jerks for traj generation.");
  this->addProperty("position_tolerance",position_tolerance_).doc("Maximum position error.");
  this->addProperty("velocity_tolerance",velocity_tolerance_).doc("Maximum velocity error.");
  this->addProperty("sampling_resolution",sampling_resolution_).doc("Sampling resolution in seconds.");
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

  this->addOperation("setMaxVelocity",&JointTrajGeneratorRML::setMaxVelocity,this,RTT::OwnThread);
  this->addOperation("setMaxAcceleration",&JointTrajGeneratorRML::setMaxAcceleration,this,RTT::OwnThread);
  this->addOperation("setMaxJerk",&JointTrajGeneratorRML::setMaxJerk,this,RTT::OwnThread);

  // ROS ports
  this->ports()->addPort("joint_traj_point_cmd_in", joint_traj_point_cmd_in_);
  this->ports()->addPort("joint_traj_cmd_in", joint_traj_cmd_in_);
  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("joint_position_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_velocity_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_position_cmd_in", conman::Exclusivity::EXCLUSIVE);
}

bool JointTrajGeneratorRML::configureHook()
{
  // ROS topics
  rtt_rostopic::ROSTopic rostopic;
  if(rostopic.ready()) {
    RTT::log(RTT::Info) << "ROS Topics ready..." <<RTT::endlog();
  } else {
    RTT::log(RTT::Error) << "ROS Topics not ready..." <<RTT::endlog();
    return false;
  }

  if(   !joint_traj_cmd_in_.createStream(rostopic.connection("~" + this->getName() + "/joint_traj_cmd"))
     || !joint_traj_point_cmd_in_.createStream(rostopic.connection("~" + this->getName() + "/joint_traj_point_cmd"))
     || !joint_state_desired_out_.createStream(rostopic.connection("~" + this->getName() + "/joint_state_desired")))
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
    rosparam->getAbsolute("robot_description");
    rosparam->getComponentPrivate("n_dof");
    rosparam->getComponentPrivate("root_link");
    rosparam->getComponentPrivate("tip_link");
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
  }

  // Require some number of joints
  if(n_dof_ == 0) {
    RTT::log(RTT::Error) << "Number of degrees-of-freedom is zero. Please set this manually, via rosparam, or use an URDF with a root and tip link." << RTT::endlog();
    return false;
  }


  // Get individual joint properties from urdf and parameter server
  joint_names_.resize(n_dof_);

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
  }
  
  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  joint_position_sample_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_velocity_sample_.resize(n_dof_);

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

  return true;
}

bool JointTrajGeneratorRML::TrajectoryMsgToSegments(
    const trajectory_msgs::JointTrajectory &msg,
    const size_t n_dof,
    const ros::Time new_traj_start_time,
    TrajSegments &segments)
{
  // Clear the output segment list
  segments.clear();

  // TODO: Permute the joint names properly
  
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

    // Copy in the data
    if(it->positions.size() == n_dof) std::copy(it->positions.begin(), it->positions.end(), new_segment.goal_positions.data());
    if(it->velocities.size() == n_dof) std::copy(it->velocities.begin(), it->velocities.end(), new_segment.goal_velocities.data());
    if(it->accelerations.size() == n_dof) std::copy(it->accelerations.begin(), it->accelerations.end(), new_segment.goal_accelerations.data());

    segments.push_back(new_segment);
  }

  return true;
}

bool JointTrajGeneratorRML::sampleTrajectory(
    const ros::Time rtt_now,
    const Eigen::VectorXd &joint_position,
    const Eigen::VectorXd &joint_velocity,
    boost::shared_ptr<ReflexxesAPI> rml,
    boost::shared_ptr<RMLPositionInputParameters> rml_in,
    boost::shared_ptr<RMLPositionOutputParameters> rml_out,
    RMLPositionFlags &rml_flags,
    JointTrajGeneratorRML::TrajSegments &segments,
    Eigen::VectorXd &joint_position_sample,
    Eigen::VectorXd &joint_velocity_sample) const
{
  if( joint_position.size() != n_dof_ ||
      joint_velocity.size() != n_dof_ ||
      joint_position_sample.size() != n_dof_ ||
      joint_velocity_sample.size() != n_dof_)
  {
    throw std::runtime_error("Bad vector passed to JointTrajGeneratorRML::sampleTrajectory.");
  }

  // Return if there are no segments to pursue
  if(segments.empty()) {
    return false;
  }

  // Initialize flag indicating that the trajectory needs to be recomputed
  bool recompute_trajectory = false;

  {
    // Find segments which should be removed
    // - segments which are not flexible
    // - segments whose expected goal times are in the past
    // - segments before segments whose start times are now or in the past
    bool remove_old_segments = false;
    TrajSegments::iterator erase_it = segments.begin();
    for(TrajSegments::iterator it = segments.begin(); it != segments.end(); ++it) 
    {
      // Peek at the next segment
      TrajSegments::iterator next = it;  ++next;

      // Mark this segment for removal if it's expired or if we need to start processing the next one
      // This only applies to non-flexible segments
      if( next != segments.end() && 
          ((!it->flexible && it->expected_time < rtt_now) || 
           (it->achieved && next->flexible) ||
           (!next->flexible && next->start_time <= rtt_now)))
      {
        erase_it = it;
        remove_old_segments = true;
      }
    }

    // Check if there's a new active segment
    if(remove_old_segments) {
      if(verbose_) RTT::log(RTT::Debug) << "Removing old segments..." << RTT::endlog();
      // Remove the segments whose end times are in the past
      segments.erase(segments.begin(), ++erase_it);
      recompute_trajectory = true;
    }
  }

  // Make sure there are still segments to process
  if(segments.empty()) {
    if(verbose_) RTT::log(RTT::Debug) << "After removing segments the trajectory is empty." << RTT::endlog();
    return false;
  }

  // Check if we should be processing the front segment
  // Once the initial segment has been processed, this should always be true until it's stopped
  if(!segments.front().flexible && segments.front().start_time > rtt_now) {
    if(verbose_) RTT::log(RTT::Debug) << "The front segment is neither flexible nor ready." << RTT::endlog();
    return false;
  }

  // Get a reference to the active segment
  TrajSegments::iterator active_segment = segments.begin();

  if(!active_segment->active) {
    if(verbose_) RTT::log(RTT::Debug) << "Front segment is not active. Activating..." << RTT::endlog();
    recompute_trajectory = true;
    active_segment->active = true;
  }

  // Determine if any of the joint tolerances have been violated (this means we need to recompute the traj)
  if(!recompute_trajectory) {
    for(int i=0; i<n_dof_; i++) 
    {
      double position_tracking_error = std::abs(rml_out->GetNewPositionVectorElement(i) - joint_position[i]);
      double velocity_tracking_error = std::abs(rml_out->GetNewVelocityVectorElement(i) - joint_velocity[i]);

      if(position_tracking_error > position_tolerance_[i]) {
        RTT::log(RTT::Debug) << "Joint " << i << " position error tolerance violated ("<<position_tracking_error<<" > "<<position_tolerance_[i]<<")" << RTT::endlog(); 
        recompute_trajectory = true;
      }
      if(velocity_tracking_error > velocity_tolerance_[i])  {
        RTT::log(RTT::Debug) << "Joint " << i << " velocity error tolerance violated ("<<velocity_tracking_error<<" > "<<velocity_tolerance_[i]<<")" << RTT::endlog(); 
        recompute_trajectory = true;
      }
    }
  }

  // Initialize RML result
  int rml_result = 0;

  // Compute RML traj after the start time and if there are still points in the queue
  if(recompute_trajectory) 
  {
    // Compute the trajectory
    if(verbose_) RTT::log(RTT::Debug) << ("RML Recomputing trajectory...") << RTT::endlog(); 

    // Set segment start time
    active_segment->start_time = rtt_now;

    // Update RML input parameters
    for(size_t i=0; i<n_dof_; i++) {
      rml_in->MaxVelocityVector->VecData[i] =  max_velocities_[i];
      rml_in->MaxAccelerationVector->VecData[i] = max_accelerations_[i];
      rml_in->MaxJerkVector->VecData[i] = max_jerks_[i];

      rml_in->SetCurrentPositionVectorElement(joint_position(i), i);
      rml_in->SetCurrentVelocityVectorElement(joint_velocity(i), i);
      rml_in->SetCurrentAccelerationVectorElement(0.0, i);

      rml_in->SetTargetPositionVectorElement(active_segment->goal_positions(i), i);
      rml_in->SetTargetVelocityVectorElement(active_segment->goal_velocities(i), i);

      // Enable this joint
      rml_in->SetSelectionVectorElement(true,i);
    }

    // Set desired execution time for this trajectory (definitely > 0)
    rml_in->SetMinimumSynchronizationTime(
        std::max(0.0,(active_segment->goal_time - active_segment->start_time).toSec()));

    if(verbose_) RTT::log(RTT::Debug) << "RML IN: time: "<<rml_in->GetMinimumSynchronizationTime() << RTT::endlog();

    // Compute trajectory
    rml_result = rml->RMLPosition(
        *rml_in.get(), 
        rml_out.get(), 
        rml_flags);

    // Get expected time
    if(verbose_) RTT::log(RTT::Debug) << "RML OUT: time: "<<rml_out->GetGreatestExecutionTime() << RTT::endlog();
    active_segment->expected_time = active_segment->start_time + ros::Duration(rml_out->GetGreatestExecutionTime());

    // If the goal time is zero, then it should be executed as fast as possible , subject to the constraints
    if(active_segment->flexible) {
      active_segment->goal_time = active_segment->expected_time;
    } else if(active_segment->expected_time > active_segment->goal_time) {
      RTT::log(RTT::Error) << "Dropping active segment because it cannot be reached in the desired time. " 
        << "Desired: " << active_segment->goal_time - active_segment->start_time 
        << " Required: " << active_segment->expected_time - active_segment->start_time
        << RTT::endlog();
      segments.pop_front();
      return false;
    }
  } 
  else 
  {
    // Sample the already computed trajectory
    rml_result = rml->RMLPositionAtAGivenSampleTime(
        (rtt_now - active_segment->start_time).toSec(),
        rml_out.get());
  }

  // Only set a non-zero effort command if the 
  try {
    switch(rml_result) 
    {
      case ReflexxesAPI::RML_WORKING:
        // S'all good.
        break;
      case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        // Mark the active segment achieved
        if(!active_segment->achieved) {
          if(verbose_) RTT::log(RTT::Debug) << "Segment complete." << RTT::endlog();
          active_segment->achieved = true;
          active_segment->flexible = true;
        }
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
    RMLLog(RTT::Error, rml_in);
    throw;
  }

  // Get the new sampled reference
  for(size_t i=0; i<n_dof_; i++) {
    joint_position_sample(i) = rml_out->GetNewPositionVectorElement(i);
    joint_velocity_sample(i) = rml_out->GetNewVelocityVectorElement(i);
  }

  return true;
}

void JointTrajGeneratorRML::updateHook()
{
  // Get the current and the time since the last update
  const RTT::Seconds 
    time = conman_hook_->getTime(), 
    period = conman_hook_->getPeriod();

  ros::Time rtt_now = rtt_rosclock::rtt_now();

  // Read in the current joint positions & velocities
  RTT::FlowStatus position_status = joint_position_in_.readNewest(joint_position_);
  RTT::FlowStatus velocity_status = joint_velocity_in_.readNewest(joint_velocity_);

  // If we don't get a position or velocity update, we don't write any new data to the ports
  if(position_status == RTT::OldData || velocity_status == RTT::OldData) 
  {
    return;
  }

  // Read in any newly commanded joint positions 
  RTT::FlowStatus point_status = joint_position_cmd_in_.readNewest( joint_position_cmd_ );
  RTT::FlowStatus traj_point_status = joint_traj_point_cmd_in_.readNewest( joint_traj_point_cmd_ );
  RTT::FlowStatus traj_status = joint_traj_cmd_in_.readNewest( joint_traj_cmd_ );

  // Do nothing and generate no output if there's no command
  if(point_status == RTT::NoData && traj_status == RTT::NoData && traj_point_status == RTT::NoData)
  {
    return;
  }

  // Flag to force recomputation of the trajectory
  bool recompute_trajectory = false;

  // Check if there's a new desired point
  if(point_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New point." <<RTT::endlog();

    // Check the size of the jointspace command
    if(joint_position_cmd_.size() == n_dof_) {
      // Handle a position given as an Eigen vector
      TrajSegment segment(n_dof_,true);

      segment.start_time = rtt_now;
      segment.goal_positions = joint_position_;
      segments_.clear();
      segments_.push_back(segment);

      // Set the recompute flag
      recompute_trajectory = true;
    } else {
      //TODO: Report warning
    }
  } 
  // Check if there's a new desired trajectory point
  else if(traj_point_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New trajectory point." <<RTT::endlog();

    // Create a unary trajectory
    trajectory_msgs::JointTrajectory unary_joint_traj;
    unary_joint_traj.points.push_back(joint_traj_point_cmd_);

    // Convert the trajectory message to a list of segments for splicing
    segments_.clear();
    TrajectoryMsgToSegments(
        unary_joint_traj, 
        n_dof_, 
        rtt_now, 
        segments_);
    
    // Set the recompute flag
    recompute_trajectory = true;
  }
  // Check if there's a new desired trajectory
  else if(traj_status == RTT::NewData) 
  {
    if(verbose_) RTT::log(RTT::Debug) << "New trajectory." <<RTT::endlog();

    // Create a new list of segments to be spliced in
    TrajSegments new_segments;
    // By default, set the start time to now
    ros::Time new_traj_start_time = rtt_now;

    // If the header stamp is non-zero, then determine which points we should pursue
    if(!joint_traj_cmd_.header.stamp.isZero()) {
      // Offset the NTP-corrected time to get the RTT-time
      // Correct the timestamp so that its relative to the realtime clock
      // TODO: make it so this can be disabled or make two different ports
      new_traj_start_time = joint_traj_cmd_.header.stamp - ros::Duration(rtt_rosclock::host_rt_offset_from_rtt());
    }

    // Convert the trajectory message to a list of segments for splicing
    TrajectoryMsgToSegments(
        joint_traj_cmd_, 
        n_dof_, 
        new_traj_start_time, 
        new_segments);
    
    // Update the trajectory
    SpliceTrajectory(segments_, new_segments);
  }

  // Sample the trajectory from segments_
  try { 
    bool new_sample = 
      this->sampleTrajectory(
          rtt_now,
          joint_position_, joint_velocity_,
          rml_, rml_in_, rml_out_, rml_flags_,
          segments_,
          joint_position_sample_, joint_velocity_sample_);

    if(new_sample) {
      // Send instantaneous joint position and velocity commands
      joint_position_out_.write(joint_position_sample_);
      joint_velocity_out_.write(joint_velocity_sample_);
      
      // Publish debug traj to ros
      if(ros_publish_throttle_.ready(0.02)) 
      {
        joint_state_desired_.header.stamp = rtt_rosclock::host_rt_now();
        joint_state_desired_.position.resize(n_dof_);
        joint_state_desired_.velocity.resize(n_dof_);
        std::copy(joint_position_sample_.data(), joint_position_sample_.data() + n_dof_, joint_state_desired_.position.begin());
        std::copy(joint_velocity_sample_.data(), joint_velocity_sample_.data() + n_dof_, joint_state_desired_.velocity.begin());
        joint_state_desired_out_.write(joint_state_desired_);
      }
    }
  } catch (std::runtime_error &err) {
    this->error();
    return;
  }
}

void JointTrajGeneratorRML::stopHook()
{
  // Clear data buffers (this will make them return OldData if nothing new is written to them)
  joint_position_in_.clear();
  joint_velocity_in_.clear();
  segments_.clear();
}

void JointTrajGeneratorRML::cleanupHook()
{
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


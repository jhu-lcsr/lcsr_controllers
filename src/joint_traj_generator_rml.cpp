

#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <rtt/internal/GlobalService.hpp>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_rostopic/rostopic.h>

#include <kdl_urdf_tools/tools.h>
#include "joint_traj_generator_kdl.h"

using namespace lcsr_controllers;

JointTrajGeneratorRML::JointTrajGeneratorRML(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,ros_publish_throttle_(0.02)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");
  this->addProperty("max_velocities",max_velocities_).doc("Maximum velocities for trap generation.");
  this->addProperty("max_accelerations",max_accelerations_).doc("Maximum acceperations for trap generation.");
  this->addProperty("position_tolerance",position_tolerance_).doc("Maximum position error.");
  this->addProperty("velocity_smoothing_factor",velocity_smoothing_factor_).doc("Exponential smoothing factor to use when estimating veolocity from finite differences.");
  this->addProperty("sampling_resolution",sampling_resolution);
  
  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("joint_position_cmd_in", joint_position_cmd_eig_in_);
  this->ports()->addPort("joint_position_out", joint_position_out_)
    .doc("Output port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("joint_velocity_out", joint_velocity_out_)
    .doc("Output port: nx1 vector of joint velocities. (n joints)");

  // ROS ports
  this->ports()->addPort("joint_position_cmd_ros_in", joint_position_cmd_ros_in_);
  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("joint_position_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_velocity_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_position_cmd_in", conman::Exclusivity::EXCLUSIVE);
  conman_hook_->setInputExclusivity("joint_position_cmd_ros_in", conman::Exclusivity::EXCLUSIVE);
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

  if(!joint_position_cmd_ros_in_.createStream(rostopic.connection("~" + this->getName() + "/joint_position_cmd"))
     || !joint_state_desired_out_.createStream(rostopic.connection("~" + this->getName() + "/joint_state_desired")))
  {
    RTT::log(RTT::Error) << "ROS Topics could not be streamed..." <<RTT::endlog();
    return false;
  }

  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  rosparam->getAbsolute("robot_description");
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("max_velocities");
  rosparam->getComponentPrivate("max_accelerations");
  rosparam->getComponentPrivate("position_tolerance");
  rosparam->getComponentPrivate("velocity_smoothing_factor");
  rosparam->getComponentPrivate("sampling_resolution");

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_position_last_.resize(n_dof_);
  joint_position_cmd_.resize(n_dof_);
  joint_position_sample_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_velocity_raw_.resize(n_dof_);
  joint_velocity_sample_.resize(n_dof_);

  trajectory_start_times_.resize(n_dof_);
  trajectory_end_times_.resize(n_dof_);
  
  position_tolerance_.resize(n_dof_);
  max_velocities_.resize(n_dof_);
  max_accelerations_.resize(n_dof_);

  // Create trajectory generators
  trajectories_.resize(n_dof_);
  for(unsigned i=0; i<n_dof_; i++){
    trajectory_start_times_[i] = 0.0;
    trajectory_end_times_[i] = 0.0;
    trajectories_[i] = KDL::VelocityProfile_Trap(max_velocities_[i], max_accelerations_[i]);
  }

  // Create trajectory generator
  rml_.reset(new ReflexxesAPI(n_dof_, sampling_resolution_));
  rml_in_.reset(new RMLPositionInputParameters(n_dof_));
  rml_out_.reset(new RMLPositionOutputParameters(n_dof_));

  // Get individual joint properties from urdf and parameter server
  joint_names_.resize(n_dof_);
  pids_.resize(n_dof_);
  joints_.resize(n_dof_);
  urdf_joints_.resize(n_dof_);
  position_tolerances_.resize(n_dof_);
  max_jerks_.resize(n_dof_);
  commanded_efforts_.resize(n_dof_);


  size_t j = 0;
  for(std::vector<KDL::Segment>::iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      ++it)
  { 
    // Get the URDF joint
    urdf_joints_[j] = urdf_model.getJoint(it->getJoint().getName());

    // Get RML parameters from URDF
    rml_in_->MaxVelocityVector->VecData[j] = urdf_joints_[j]->limits->velocity;
    rml_in_->MaxAccelerationVector->VecData[j] = max_accelerations_[j];
    rml_in_->MaxJerkVector->VecData[j] = max_jerks_[j];

    // Enable this joint
    rml_in_->SelectionVector->VecData[j] = true;
    
    j++;
  }

  // Check if the reflexxes config is valud
  if(rml_in_->CheckForValidity()) {
    RTT::log(RTT::Info) << ("RML INPUT Configuration Valid.") << RTT::endlog();
    this->rml_debug(RTT::Debug);
  } else {
    RTT::log(RTT::Error) << ("RML INPUT Configuration Invalid!") << RTT::endlog();
    this->rml_debug(RTT::Error);
    return false;
  }

  return true;
}

bool JointTrajGeneratorRML::startHook()
{
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
  RTT::FlowStatus new_position_status = joint_position_in_.readNewest(joint_position_);
  RTT::FlowStatus new_velocity_status = joint_velocity_in_.readNewest(joint_velocity_);

  // If we don't get any position update, we don't write any new data to the ports
  if(new_position_data == RTT::OldData || new_velocity_status == RTT::OldData) {
    return;
  }

  // Read in any newly commanded joint positions 
  RTT::FlowStatus point_status = joint_position_cmd_in_.readNewest( joint_position_cmd_ );
  RTT::FlowStatus traj_status = joint_traj_cmd_in_.readNewest( joint_traj_cmd_ );

  bool new_reference = false;
  if(point_status == RTT::NewData) 
  {
    // Handle a position given as an Eigen vector
    ViaPoint via(n_dof_);
    via.time = rtt_now;
    if(joint_position_cmd_.size() == n_dof_) {
      via.positions = joint_position_;
    }
    vias_.clear();
    vias_.push_back(via);

  } 
  else if(traj_status == RTT::NewData) 
  {
    // TODO: Permute the joint names properly
    
    // Make sure the traj isn't empty
    if(joint_position_cmd_.points.size() > 0) {
      // Compute the earliest time that a point from this new trajectory should be achieved
      // It's assumed that the header timestamp is when this new trajectory
      // should start being executed. This also means that if there is a queued
      // trajectory which contains this start time that that trajectory should
      // not be disturbed until that start time. We achieve this by checking
      // the next via point's start time. If the start time is 0.0 then it will
      // not be activated until the end-time of the active via goal. If the start
      // time is non-zero, then it will become active once the start time has passed.
      // 
      ros::Time new_traj_start_time =
        + (joint_traj_cmd_.header.stamp.isZero()) ? rtt_rosclock::host_rt_now() : joint_traj_cmd_.header.stamp
        - rtt_rosclock::host_rt_offset_from_rtt();

      ros::Time earliest_end_time = 
        new_traj_start_time
        + joint_traj_cmd_.points[0].time_from_start;

      ViaPoint earliest_via(n_dof_, earliest_end_time);
      
      // Determine where the points should begin to be inserted
      std::pair<Vias::iterator, Vias::iterator> insertion_range = 
        std::equal_range(vias_.begin(), vias_.end(), earliest_via, ViaPoint::Lesser);

      // Remove all vias with completion times after the earliest via time in the new trajectory
      vias_.erase(insertion_range.first, vias_.end());
      vias_.reserve(vias_.size() + joint_traj_cmd_.points.size());

      // Convert the ROS joint trajectory to a set of Eigen ViaPoint structures
      for(std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator it = joint_traj_cmd_.points.begin();
          it != joint_traj_cmd_.points.end();
          ++it) 
      {
        // Create a new via point
        ros::Time new_via_start_time;

        // Get the start time for the new via
        if(it == joint_traj_cmd_.points.begin()) {
          new_via_start_time = new_traj_start_time;
        } else {
          if(vias_.size() > 0) {
            new_via_start_time = vias_.back().end_time;
          } else {
            // This shouldn't ever happen, this should either be the first
            // point in the traj or the traj should already exist
            this->error();
          }
        }
        
        // Add the new via and copy the data
        vias_.push_back(Via(n_dof_, new_via_start_time, new_traj_start_time + it->time_from_start));
        Via &new_via = vias_.back();
        std::copy(it->positions.begin(), it->positions.end(), new_via.positions.data());
        std::copy(it->velocities.begin(), it->velocities.end(), new_via.velocities.data());
        std::copy(it->accelerations.begin(), it->accelerations.end(), new_via.accelerations.data());
      }
    }
  }
  else if(point_status == RTT::NoData && traj_status == RTT::NoData {
    // Initialize the samples with the current positions
    joint_position_sample_ = joint_position_;
    joint_velocity_sample_ = joint_velocity_;
  }

  // Find vias whose end-times are in the past
  Vias::iterator erase_it = vias_.begin();
  for(Vias::iterator it = vias_.begin(); it != vias_.end(); ++it) 
  {
    // Peek at the next
    Vias::iterator next = it;  ++next;
    
    // Mark this via for removal if it's expired or if we need to start processing the next one
    if(it->end_time <= rtt_now || (next != vias_.end() && next->start_time <= rtt_now)) {
      erase_it = it;
      new_via_goal_ = true;
    }
  }
  // Remove the vias whose end times are in the past
  vias_.erase(vias_.begin(), erase_it);

  // Check if there are any more vias to process and if we should start processing them or wait
  if(vias_.size() > 0 && vias_.front().start_time <= rtt_now) 
  {
    // Get a reference to the active via
    Via &active_via = vias_.front();

    // Check for a new reference
    if(new_via_goal_) 
    {
      // Reset new reference flag
      new_via_goal_ = false;
      // Set flag to recompute trajectory
      recompute_trajectory_ = true;
      // Store the actual start time
      active_via.start_time = rtt_now;

      RTT::log(RTT::Debug) << ("Received new via goal.") << RTT::endlog();
    }

    // Determine if any of the joint tolerances have been violated (this means we need to recompute the traj)
    for(int i=0; i<n_dof_; i++) 
    {
      double tracking_error = std::abs(rml_out_->GetNewPositionVectorElement(i) - joint_position_[i]);
      if(tracking_error > position_tolerances_[i]) {
        recompute_trajectory_ = true;
        //ROS_WARN_STREAM("Tracking for joint "<<i<<" outside of tolerance! ("<<tracking_error<<" > "<<position_tolerances_[i]<<")");
      }
    }

    // Initialize RML result
    int rml_result = 0;

    // Compute RML traj after the start time and if there are still points in the queue
    if(recompute_trajectory_) 
   {
      // Get reference to the active trajectory point
      const trajectory_msgs::JointTrajectoryPoint &active_traj_point = commanded_trajectory.points[point_index_];

      // Compute the trajectory
      RTT::log(RTT::Debug) << ("RML Recomputing trajectory...") << RTT::endlog(); 

      // Update RML input parameters
      for(size_t i=0; i<n_dof_; i++) {
        rml_in_->SetCurrentPositionVectorElement(joint_position_(i), i);
        rml_in_->SetCurrentVelocityVectorelement(joint_velocity_(i), i);
        rml_in_->SetCurrentAccelerationVectorElement(0.0, i);

        rml_in_->SetTargetPositionVectorElement(active_via.positions(i), i);
        rml_in_->SetTargetVelocityVectorelement(actuve_via.velocities(i), i);

        rml_in_->SetSelectionVectorElement(true,i);
      }

      // Store the traj start time
      traj_start_time_ = rtt_now;

      // Set desired execution time for this trajectory (definitely > 0)
      rml_in_->SetMinimumSynchronizationTime(
          std::max(0.0,(active_via.end_time - active_via.start_time).toSec()));

      RTT::log(RTT::Debug) << "RML IN: time: "<<rml_in_->GetMinimumSynchronizationTime() << RTT::endlog();

      // Hold fixed at final point once trajectory is complete
      rml_flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
      rml_flags_.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

      // Compute trajectory
      rml_result = rml_->RMLPosition(
          *rml_in_.get(), 
          rml_out_.get(), 
          rml_flags_);

      // Disable recompute flag
      recompute_trajectory_ = false;
    } 
    else 
    {
      // Sample the already computed trajectory
      rml_result = rml_->RMLPositionAtAGivenSampleTime(
          (rtt_now - active_via.start_time).toSec(),
          rml_out_.get());
    }

    // Only set a non-zero effort command if the 
    switch(rml_result) 
    {
      case ReflexxesAPI::RML_WORKING:
        // S'all good.
        break;
      case ReflexxesAPI::RML_FINAL_STATE_REACHED:
        // NOTE: We don't need to do this any more
        //recompute_trajectory_ = true;
        break;
      default:
        RTT::log(RTT::Error) << "Reflexxes error code: "<<rml_result<<". Not writing a desired position." << RTT::endlog();
        this->error();
        return;
        break;
    };

    // Get the new sampled reference
    for(size_t i=0; i<n_dof_; i++) {
      joint_position_sample_(i) = rml_out_->GetNewPositionVectorElement(i);
      joint_velocity_sample_(i) = rml_out_->GetNewVelocityVectorElement(i);
    }
  }

  // Send instantaneous joint position and velocity commands
  joint_position_out_.write(joint_position_sample_);
  joint_velocity_out_.write(joint_velocity_sample_);
  
  // Publish debug traj to ros
  if(ros_publish_throttle_.ready()) 
  {
    joint_state_desired_.header.stamp = rtt_rosclock::host_rt_now();
    joint_state_desired_.position.resize(n_dof_);
    joint_state_desired_.velocity.resize(n_dof_);
    std::copy(joint_position_sample_.data(), joint_position_sample_.data() + n_dof_, joint_state_desired_.position.begin());
    std::copy(joint_velocity_sample_.data(), joint_velocity_sample_.data() + n_dof_, joint_state_desired_.velocity.begin());
    joint_state_desired_out_.write(joint_state_desired_);
  }
}

void JointTrajGeneratorRML::stopHook()
{
  // Clear data buffers (this will make them return OldData if nothing new is written to them)
  joint_position_in_.getManager()->clear();
  joint_velocity_sample_.getManager()->clear();
}

void JointTrajGeneratorRML::cleanupHook()
{
}

void JointTrajGeneratorRML::rml_debug(const RTT::LoggerLevel level) {
  RTT::log(level) << "RML INPUT: "<< RTT::endlog();
  RTT::log(level) << " - NumberOfDOFs: "<<rml_in_->NumberOfDOFs << RTT::endlog();
  RTT::log(level) << " - MinimumSynchronizationTime: "<<rml_in_->MinimumSynchronizationTime);
  RTT::log(level) << " - SelectionVector: "<<(*rml_in_->SelectionVector) << RTT::endlog();
  RTT::log(level) << " - CurrentPositionVector: "<<(*rml_in_->CurrentPositionVector) << RTT::endlog();
  RTT::log(level) << " - CurrentVelocityVector: "<<(*rml_in_->CurrentVelocityVector) << RTT::endlog();
  RTT::log(level) << " - CurrentAccelerationVector: "<<(*rml_in_->CurrentAccelerationVector) << RTT::endlog();
  RTT::log(level) << " - MaxAccelerationVector: "<<(*rml_in_->MaxAccelerationVector) << RTT::endlog();
  RTT::log(level) << " - MaxJerkVector: "<<(*rml_in_->MaxJerkVector) << RTT::endlog();
  RTT::log(level) << " - TargetVelocityVector: "<<(*rml_in_->TargetVelocityVector) << RTT::endlog();

  RTT::log(level) << " - MaxVelocityVector: "<<(*rml_in_->MaxVelocityVector) << RTT::endlog();
  RTT::log(level) << " - TargetPositionVector: "<<(*rml_in_->TargetPositionVector) << RTT::endlog();
  RTT::log(level) << " - AlternativeTargetVelocityVector: "<<(*rml_in_->AlternativeTargetVelocityVector) << RTT::endlog();
}



#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <angles/angles.h>

#include <rtt_rosparam/rosparam.h>

#include <kdl_urdf_tools/tools.h>

#include "semi_absolute_calibration_controller.h"

using namespace lcsr_controllers;

SemiAbsoluteCalibrationController::SemiAbsoluteCalibrationController(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
{
  // Declare properties
  this->addProperty("robot_description",robot_description_).doc("The WAM URDF xml string.");
  this->addProperty("root_link",root_link_).doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_).doc("The tip link for the controller.");

  this->addProperty("static_thresholds",static_thresholds_);
  this->addProperty("upper_limits",upper_limits_);
  this->addProperty("lower_limits",lower_limits_);
  this->addProperty("limit_search_directions",limit_search_directions_);
  this->addProperty("home_positions",home_positions_);
  this->addProperty("resolver_offsets_at_home",resolver_offsets_at_home_);
  this->addProperty("resolver_ranges",resolver_ranges_);
  
  // Operations
  this->provides()->addOperation(
      "calibrate",
      &SemiAbsoluteCalibrationController::calibrate_cb,
      this,RTT::OwnThread);

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("resolver_offset_in", resolver_offset_in_);

  this->ports()->addPort("joint_position_estimate_out", joint_position_estimate_out_);
  this->ports()->addPort("joint_position_desired_out", joint_position_desired_out_);

  // ROS ports
  this->ports()->addPort("calibration_state", calibration_state_out_);

  // Load Conman interface
  conman_hook_ = conman::Hook::GetHook(this);
  conman_hook_->setInputExclusivity("joint_position_in", conman::Exclusivity::EXCLUSIVE);
}

bool SemiAbsoluteCalibrationController::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  rosparam->getAbsolute("robot_description");
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("upper_limits");
  rosparam->getComponentPrivate("lower_limits");
  rosparam->getComponentPrivate("limit_search_directions");
  rosparam->getComponentPrivate("home_positions");
  rosparam->getComponentPrivate("resolver_offsets_at_home");
  rosparam->getRelative("resolver_ranges");

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Initialize state vectors
  calibration_states_.assign(n_dof_,UNCALIBRATED);
  position_history_.assign(n_dof_,std::list<double>());
  static_detection_armed_.assign(n_dof_,false);

  // Resize IO vectors
  joint_position_raw_.resize(n_dof_);
  joint_position_desired_.resize(n_dof_);
  joint_position_estimate_.resize(n_dof_);
  joint_position_correction_.resize(n_dof_);

  // Assign joint names
  joint_names_.resize(n_dof_);
  for(unsigned i=0; i<n_dof_; i++){
    joint_names_[i] = kdl_chain_.getSegment(i).getJoint().getName();
  }


  // Prepare ports for realtime processing
  joint_position_desired_out_.setDataSample(joint_position_desired_);
  joint_position_estimate_out_.setDataSample(joint_position_estimate_);

  return true;
}

bool SemiAbsoluteCalibrationController::startHook()
{
  // Start calibration in idle step
  calibration_step_ = IDLE;
  // Initialize with no correction
  joint_position_correction_.setZero();
  return true;

}

void SemiAbsoluteCalibrationController::updateHook()
{
  // Get the current and the time since the last update
  const RTT::Seconds 
    time = conman_hook_->getTime(), 
    period = conman_hook_->getPeriod();

  // Read in the current raw joint positions & velocities
  bool new_position_data = (joint_position_in_.readNewest(joint_position_raw_) == RTT::NewData);
  bool new_resolver_data = (resolver_offset_in_.readNewest(resolver_offset_) == RTT::NewData);

  // If we don't get any position update, we don't write any new data to the ports
  if(!new_position_data || !new_resolver_data) {
    return;
  }

  // Detect if the joint positions have been corrected at the source
  // This should prevent the estimate from jumping when the calibration is "committed"
  static const double correction_threshold = 0.01;
  if(((joint_position_raw_ - joint_position_estimate_).array() < correction_threshold).all()) {
    // Zero-out the correction
    joint_position_correction_.setZero();
  }
  
  // Correct the joint position if necessary
  joint_position_estimate_ = joint_position_raw_ + joint_position_correction_;

  // Compute commands for each joint
  for(unsigned jid=0; jid < n_dof_; jid++) {
    switch(calibration_states_[jid]) 
    {
      case UNCALIBRATED:
        {
          // Hold fixed in the current position (we don't have anything better to do)
          joint_position_desired_[jid] = joint_position_raw_[jid];
          break;
        }
      case CALIBRATED:
        {
          // Hold in the calibrated position
          joint_position_desired_[jid] = calibrated_positions_[jid];

          if(calibration_step_ == START_ASSUME_CALIBRATION_POSTURE) {
            // Clear the position history buffer
            position_history_[jid].clear();
            static_detection_armed_[jid] = true;
          }
          break;
        }
      case CALIBRATING:
        {
          // All actively calibrating joints are in the same step at a time
          switch(calibration_step_) 
          {
            case IDLE:
              { break; }
            case START_ASSUME_CALIBRATION_POSTURE:
              {
                // Hold this position until the joints are 
                joint_position_desired_[jid] = joint_position_raw_[jid];
              }
            case ASSUME_CALIBRATION_POSTURE: break; // 
            case START_LIMIT_SEARCH: 
              {
                // Determine the desired joint position, guaranteed to reach a joint limit in this dimension
                if(limit_search_directions_[jid] > 0.0) {
                  joint_position_desired_[jid] = joint_position_raw_[jid] + upper_limits_[jid] - lower_limits_[jid];
                } else {
                  joint_position_desired_[jid] = joint_position_raw_[jid] - upper_limits_[jid] + lower_limits_[jid];
                }

                // Clear the position history buffer
                position_history_[jid].clear();
                static_detection_armed_[jid] = false;
              }
            case LIMIT_SEARCH: break; // Find the positive or negative limit of this joint by driving towards it
            case START_APPROACH_CALIB_REGION:
              {
                // Update the estimate to the coarse position
                if(limit_search_directions_[jid] > 0.0) {
                  joint_position_estimate_[jid] = upper_limits_[jid];
                } else {
                  joint_position_estimate_[jid] = lower_limits_[jid];
                }

                // Set the desired position to the homing position
                joint_position_desired_[jid] = home_positions_[jid];

                // Clear the position buffer
                position_history_[jid].clear();
                static_detection_armed_[jid] = false;
                break;
              }
            case APPROACH_CALIB_REGION: break; // Drive near to the homing (calibrated) position
            case START_GO_HOME:
              {
                // Set the precise position estimate by adding the difference between the known resolver offset
                // TODO: deal with differential
                joint_position_estimate_[jid] += 
                  resolver_ranges_[jid]/2.0/M_PI *
                  angles::shortest_angular_distance(
                      2.0*M_PI/resolver_ranges_[jid]*resolver_offsets_at_home_[jid], 
                      2.0*M_PI/resolver_ranges_[jid]*resolver_offset_[jid]);
                // Set the desired position to the homing position (again)
                joint_position_desired_[jid] = home_positions_[jid];
                // Clear the position buffer
                position_history_[jid].clear();
                static_detection_armed_[jid] = false;
                break;
              }
            case GO_HOME: break; // Drive to the precise homing (calibrated) position
            case END_CALIBRATION: 
              {
                // Mark this joint as calibrated
                calibration_states_[jid] = CALIBRATED; 
                break;
              }
          };
          break;
        }
    };
  }
    
  // Flag to determine if all calibrating joints are static
  bool all_joints_static = true;

  // Determine if all the joints are static
  for(unsigned jid=0; jid < n_dof_; jid++) {
    bool joint_is_static = false;

    // We've reached the limit if the last 10 samples are within the static threshold
    double min_pos = *std::min_element(position_history_[jid].begin(), position_history_[jid].end());
    double max_pos = *std::max_element(position_history_[jid].begin(), position_history_[jid].end());

    // Only store the last 100 samples
    // TODO: Make this configurable / based on time instead of # of samples?
    static const size_t enough_samples = 100;
    position_history_[jid].push_back(joint_position_estimate_[jid]);
    while(position_history_[jid].size() > enough_samples) {
      position_history_[jid].pop_front();
    }

    // If we have enough samples for this joint, check if it's static
    if(position_history_[jid].size() == enough_samples) {
      // If the span of values in the buffer is greater than the static thresh for this joint
      // We want to ensure that the joint moves enough before we start checking if it's static
      // This is akin to the way the bomb worked in the Keanu Reeves movie "Speed"
      if(max_pos - min_pos > static_thresholds_[jid]) {
        static_detection_armed_[jid] = true;
        joint_is_static = false;
      } else if(static_detection_armed_[jid] && max_pos - min_pos < static_thresholds_[jid]) {
        joint_is_static = true;
      } 
    }

    // Determine if all the joints are static
    // TODO: Make this configurable 
    static const double close_enough = 0.05;
    switch(calibration_states_[jid]) {
      case UNCALIBRATED: break;
      case CALIBRATED:
      case CALIBRATING:
        all_joints_static = all_joints_static && joint_is_static;
        // Unless we're searching for the joint limit, require that the
        // difference between the actual and commanded position is small enough
        if(calibration_step_ != LIMIT_SEARCH) {
          all_joints_static = all_joints_static && fabs(joint_position_desired_[jid] - joint_position_estimate_[jid]) < close_enough;
        }
        break;
    };
  }

  // Compute step transitions
  switch(calibration_step_) {
    // No-op
    case IDLE: break;

    // Drive the calibrated joints to a specific joint position before calibrating the uncalibrated joints
    case START_ASSUME_CALIBRATION_POSTURE:
      calibration_step_ = ASSUME_CALIBRATION_POSTURE; break;
    case ASSUME_CALIBRATION_POSTURE:
      calibration_step_ = all_joints_static ? START_LIMIT_SEARCH : ASSUME_CALIBRATION_POSTURE; break;

    // Drive towards the soft joint limit
    case START_LIMIT_SEARCH: 
      calibration_step_ = LIMIT_SEARCH; break;
    case LIMIT_SEARCH:
      calibration_step_ = all_joints_static ? START_APPROACH_CALIB_REGION : LIMIT_SEARCH; break;

    // Drive towards the approximate home position
    case START_APPROACH_CALIB_REGION:
      calibration_step_ = APPROACH_CALIB_REGION; break;
    case APPROACH_CALIB_REGION:
      calibration_step_ = all_joints_static ? START_GO_HOME : APPROACH_CALIB_REGION; break;

    // Drive to the precise home position
    case START_GO_HOME:
      calibration_step_ = GO_HOME; break;
    case GO_HOME:
      calibration_step_ = all_joints_static ? END_CALIBRATION : GO_HOME; break;

    // Stop
    case END_CALIBRATION:
      calibration_step_ = IDLE;
      break;
  };

  // Write the desired joint position
  joint_position_desired_out_.write(joint_position_desired_);
  // Write the (calibrated) joint position
  joint_position_estimate_out_.write(joint_position_estimate_);

  // Store the latest joint position correction
  joint_position_correction_ = joint_position_estimate_ - joint_position_raw_;
}

void SemiAbsoluteCalibrationController::stopHook()
{
}

void SemiAbsoluteCalibrationController::cleanupHook()
{
}

bool SemiAbsoluteCalibrationController::calibrate_cb(
    lcsr_controllers::Calibrate::Request &req,
    lcsr_controllers::Calibrate::Response &resp)
{
  if(calibration_step_ != IDLE) {
    RTT::log(RTT::Error) << "Please wait for the active joints to finish calibrating..." << RTT::endlog();
    resp.ok = false;
  } else if(req.calibration_posture.size() == n_dof_) {
    RTT::log(RTT::Error) << "Incorrect joint calibration posture" << RTT::endlog();
    resp.ok = false;
  } else {
    // Determine which joints should calibrate, by name
    for(std::vector<std::string>::iterator req_name = req.joints_to_calibrate.begin();
        req_name != req.joints_to_calibrate.end();
        ++req_name)
    {
      for(unsigned int i=0; i<n_dof_; i++) {
        if(joint_names_[i] == *req_name) {
          calibration_states_[i] = CALIBRATING;
          resp.ok = true;
          break;
        }
      }
    }

    // Store the desired positions for the calibrated joints
    for(unsigned int i=0; i<n_dof_; i++) {
      calibrated_positions_[i] = req.calibration_posture[i];
    }

    // Begin calibration
    calibration_step_ = START_ASSUME_CALIBRATION_POSTURE;
  }

  return resp.ok;
}

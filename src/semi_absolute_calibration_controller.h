#ifndef __LCSR_CONTROLLERS_SEMI_ABSOLUTE_CALIBRATON_CONTROLLER_H
#define __LCSR_CONTROLLERS_SEMI_ABSOLUTE_CALIBRATON_CONTROLLER_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <lcsr_controllers/Calibrate.h>
#include <lcsr_controllers/SemiAbsoluteCalibrationState.h>

#include <conman/hook.h>

namespace lcsr_controllers {
  class SemiAbsoluteCalibrationController : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::vector<double> 
      static_thresholds_,
      upper_limits_,
      lower_limits_,
      limit_search_directions_,
      home_positions_,
      resolver_ranges_,
      resolver_offsets_at_home_;
    std::vector<std::string> joint_names_;

    std::vector<bool> static_detection_armed_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> resolver_offset_in_;
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;

    RTT::OutputPort<Eigen::VectorXd> joint_position_desired_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_position_estimate_out_;
    RTT::OutputPort<SemiAbsoluteCalibrationState> calibration_state_out_;

  public:
    SemiAbsoluteCalibrationController(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Robot model
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    typedef enum {
      // Not actively calibrating
      IDLE = 0,                        
      START_ASSUME_CALIBRATION_POSTURE,
      ASSUME_CALIBRATION_POSTURE,
      START_LIMIT_SEARCH,           // 
      LIMIT_SEARCH,
      START_APPROACH_CALIB_REGION,
      APPROACH_CALIB_REGION,
      START_GO_HOME,
      GO_HOME,
      END_CALIBRATION
    } calibration_step_t;

    typedef enum {
      UNCALIBRATED = 0,
      CALIBRATING = 1,
      CALIBRATED = 2
    } calibration_state_t;

    std::vector<calibration_state_t> calibration_states_;
    calibration_step_t calibration_step_;
    std::vector<std::list<double> > position_history_;

    // State
    Eigen::VectorXd 
      joint_position_raw_,
      joint_position_desired_,
      joint_position_correction_,
      joint_position_estimate_,
      resolver_offset_;

    Eigen::VectorXd
      calibrated_positions_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;

    // Calibration command cb
    bool calibrate_cb(
        lcsr_controllers::Calibrate::Request &req,
        lcsr_controllers::Calibrate::Response &resp);
  };
}


#endif // ifndef __LCSR_CONTROLLERS_SEMI_ABSOLUTE_CALIBRATON_CONTROLLER_H

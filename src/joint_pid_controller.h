#ifndef __LCSR_CONTROLLERS_JOINT_PID_CONTROLLER_H
#define __LCSR_CONTROLLERS_JOINT_PID_CONTROLLER_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <conman/hook.h>

namespace lcsr_controllers {
  class JointPIDController : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    Eigen::VectorXd
      p_gains_,
      i_gains_,
      d_gains_,
      i_clamps_;
    float velocity_smoothing_factor_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<Eigen::VectorXd> joint_position_cmd_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_cmd_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;

  public:
    JointPIDController(std::string const& name);
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

    // State
    Eigen::VectorXd 
      joint_p_error_,
      joint_d_error_,
      joint_i_error_,
      joint_position_,
      joint_position_last_,
      joint_position_cmd_,
      joint_velocity_,
      joint_velocity_raw_,
      joint_velocity_cmd_,
      joint_effort_;

    bool has_last_position_data_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_PID_CONTROLLER_H

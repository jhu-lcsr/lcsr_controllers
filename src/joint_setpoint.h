#ifndef __LCSR_CONTROLLERS__JOINT_SETPOINT_H
#define __LCSR_CONTROLLERS__JOINT_SETPOINT_H

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include <Eigen/Dense>

namespace lcsr_controllers {
  class JointSetpoint : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string robot_description_param_;
    std::string root_link_;
    std::string tip_link_;
    Eigen::VectorXd joint_position_;
    Eigen::VectorXd joint_velocity_;
    Eigen::VectorXd joint_position_cmd_;
    Eigen::VectorXd joint_velocity_cmd_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;

    RTT::OutputPort<Eigen::VectorXd> joint_position_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_velocity_out_;

  public:
    JointSetpoint(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    void hold();
  private:

    // Robot model
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    bool hold_current_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS__JOINT_SETPOINT_H

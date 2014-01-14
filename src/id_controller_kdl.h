#ifndef __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H
#define __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace lcsr_controllers {
  class IDControllerKDL : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::vector<double> gravity_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;

  public:
    IDControllerKDL(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Working variables
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;

    KDL::JntArray positions_;
    KDL::JntArray velocities_;
    KDL::JntArray accelerations_;
    KDL::Wrenches ext_wrenches_;
    KDL::JntArray torques_;

    Eigen::VectorXd 
      joint_position_,
      joint_velocity_,
      joint_acceleration_,
      joint_effort_;

  };
}


#endif // ifndef __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H

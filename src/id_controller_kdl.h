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
#include <kdl/chainfksolverpos_recursive.hpp>

#include <rtt_ros_tools/throttles.h>

#include <geometry_msgs/WrenchStamped.h>
#include <visualization_msgs/Marker.h>
#include <telemanip_msgs/AttachedInertia.h>

namespace lcsr_controllers {
  class IDControllerKDL : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_param_;
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::string wrench_link_;
    Eigen::VectorXd gravity_;
    bool compensate_gravity_;
    bool compensate_coriolis_;
    bool compensate_inertial_;
    bool compensate_end_effector_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<Eigen::VectorXd> end_effector_masses_in_;
    RTT::InputPort<telemanip_msgs::AttachedInertia> end_effector_inertias_in_;

    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;

    RTT::OutputPort<geometry_msgs::WrenchStamped> ext_wrenches_debug_out_;
    RTT::OutputPort<visualization_msgs::Marker> cogs_debug_out_;

  public:
    IDControllerKDL(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    // Solvers
    boost::scoped_ptr<KDL::ChainIdSolver_RNE> id_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

    // Working variables
    KDL::JntArray positions_;
    KDL::JntArray velocities_;
    KDL::JntArray accelerations_;
    KDL::Wrenches ext_wrenches_;
    KDL::JntArray torques_;

    Eigen::VectorXd 
      joint_position_,
      joint_velocity_,
      joint_acceleration_,
      end_effector_mass_,
      joint_effort_;

    KDL::RigidBodyInertia ee_inertia;
    KDL::Wrench ee_wrench;

    geometry_msgs::WrenchStamped wrench_msg_;
    std::vector<visualization_msgs::Marker> cogs_msgs_;

    rtt_ros_tools::PeriodicThrottle debug_throttle_;
    telemanip_msgs::AttachedInertia end_effector_inertia_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H

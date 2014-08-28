#ifndef __LCSR_CONTROLLERS_COULOMB_COMPENSATOR_H
#define __LCSR_CONTROLLERS_COULOMB_COMPENSATOR_H

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>

//#include <rtt_ros_tools/throttles.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>

namespace lcsr_controllers {
  class CoulombCompensator : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string robot_description_param_;
    std::string root_link_;
    std::string tip_link_;
    Eigen::VectorXd friction_coefficients_neg_;
    Eigen::VectorXd friction_coefficients_pos_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<KDL::FrameVel> framevel_des_in_;
    RTT::OutputPort<Eigen::VectorXd> joint_effort_out_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_des_out_;

  public:
    CoulombCompensator(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    bool compute_ik(
        const KDL::JntArrayVel &position,
        const KDL::Frame &tip_frame_des,
        KDL::JntArray &ik_hint,
        KDL::JntArrayVel &position_des,
        KDL::JntArrayVel &position_des_last,
        const bool debug, 
        const ros::Duration dt);
  private:

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Chain kdl_chain_;
    KDL::Tree kdl_tree_;
    //urdf::Model urdf_model_;
    double zero_slope_;

    // Working variables
    KDL::JntArray joint_position_;
    KDL::FrameVel framevel_des_;
    KDL::Twist tip_twist_des_;
    KDL::JntArray joint_velocitiy_des_;
    Eigen::VectorXd joint_effort_;

    // Joint limits
    KDL::JntArray joint_limits_min_;
    KDL::JntArray joint_limits_max_;

    // KDL IK solver which accounts for joint limits
    boost::shared_ptr<KDL::ChainIkSolverVel> kdl_ik_solver_vel_;

    sensor_msgs::JointState joint_state_des_;

    rtt_ros_tools::PeriodicThrottle ros_publish_throttle_;

    bool warn_flag_;

  };
}


#endif // ifndef __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H

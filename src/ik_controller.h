#ifndef __LCSR_CONTROLLERS_IK_CONTROLLER_H
#define __LCSR_CONTROLLERS_IK_CONTROLLER_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
//#include <kdl/chainidsolver_recursive_newton_euler.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
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
#include <tf/tf.h>

//#include <geometry_msgs/WrenchStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>

namespace lcsr_controllers {
  class IKController : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    std::string target_frame_;

    std::vector<double> kp_; // proportional gains
    std::vector<double> kd_; // derivative gains

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> positions_in_port_;
    RTT::OutputPort<Eigen::VectorXd> positions_out_port_;
    RTT::OutputPort<Eigen::VectorXd> torques_out_port_;
    RTT::OutputPort<trajectory_msgs::JointTrajectory> trajectories_out_port_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_desired_out_;

    // RTT Debug Ports
    RTT::OutputPort<Eigen::VectorXd> torques_debug_out_;
    RTT::OutputPort<trajectory_msgs::JointTrajectory> trajectories_debug_out_;

    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&,
      const std::string&)> tf_lookup_transform_;

  public:
    IKController(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

    void test_ik();
    void compute_ik(bool debug);
  private:

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Chain kdl_chain_;
    KDL::Tree kdl_tree_;
    //urdf::Model urdf_model_;


    // Working variables
    KDL::JntArrayVel positions_;
    KDL::JntArrayVel positions_des_;
    KDL::JntArray torques_;

    // Joint limits
    KDL::JntArray joint_limits_min_;
    KDL::JntArray joint_limits_max_;

    // KDL IK solver which accounts for joint limits
    boost::scoped_ptr<KDL::ChainIkSolverPos> kdl_ik_solver_top_;
    boost::scoped_ptr<KDL::ChainIkSolverVel> kdl_ik_solver_vel_;

    // KDL FK solver
    boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_fk_solver_pos_;

    geometry_msgs::TransformStamped tip_frame_msg_;
    tf::Transform tip_frame_tf_;
    KDL::Frame tip_frame_;
    KDL::Frame tip_frame_des_;

    trajectory_msgs::JointTrajectory trajectory_;
    sensor_msgs::JointState joint_state_desired_;

    rtt_ros_tools::PeriodicThrottle ros_publish_throttle_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_ID_CONTROLLER_KDL_H
